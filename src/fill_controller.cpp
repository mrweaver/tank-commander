#include "fill_controller.h"

FillController::FillController(uint8_t powerPin, uint8_t dirPin, Tank* tank3)
    : m_powerPin(powerPin),
      m_dirPin(dirPin),
      m_tank3(tank3),
      m_state(FillState::DISABLED_STATE),
      m_fillReason(FillReason::NONE),
      m_keepAwake(false),
      m_fillStart_ms(0),
      m_cooldownStart_ms(0),
      m_valvePhase(ValvePhase::DONE),
      m_valvePhaseStart_ms(0),
      m_pendingState(FillState::IDLE),
      m_openStart_ms(0),
      m_sensorRecoveryStart_ms(0),
      m_sensorBadStreak(0),
      m_fault(FillFault::NONE),
      m_progressRef_pc(0),
      m_progressRef_ms(0),
      m_fillRate_pcpm(0),
      m_leakWatch(false),
      m_reclosing(false),
      m_leakAttempts(0),
      m_leakWatchStart_ms(0),
      m_leakRef_ms(0),
      m_leakRef_pc(0),
      m_leakRefValid(false),
      m_leakStart_pc(0),
      m_leakStartValid(false),
      m_leakLastAttempt_ms(0),
      m_valvePos_ms(VALVE_FULL_TRAVEL_MS),  // unknown at boot — assume open; boot close re-references
      m_moveStartPos_ms(VALVE_FULL_TRAVEL_MS),
      m_moveTarget_ms(0),
      m_moveDuration_ms(VALVE_TRAVEL_MS),
      m_moveDirOpen(false)
{
}

void FillController::begin()
{
    // Boot-safe relay init should already be done in setup() before this call,
    // but reinforce here as defense-in-depth
    allOff();

    EEPROM.begin(FILL_EEPROM_SIZE);
    loadConfig();

    // A manual OPEN override is not resumed after a reboot: the fill timers
    // restart on boot, so resuming would let a reboot loop bypass the hard max.
    if ((ValveMode)m_config.valveMode == ValveMode::OPEN)
    {
        Serial.println("Fill: OPEN override not resumed after reboot — mode -> AUTO.");
        m_config.valveMode   = (uint8_t)ValveMode::AUTO;
        m_config.fillEnabled = true;
        saveConfig();
    }

    FillState afterClose = ((ValveMode)m_config.valveMode == ValveMode::AUTO)
                               ? FillState::IDLE
                               : FillState::DISABLED_STATE;

    // A latched fill fault survives reboot — it must be cleared explicitly.
    if (m_config.latchedFault != (uint8_t)FillFault::NONE)
    {
        m_fault    = (FillFault)m_config.latchedFault;
        afterClose = FillState::FAULT_TIMEOUT;
        Serial.printf("Fill: Latched fault restored from EEPROM (code %u).\n", m_config.latchedFault);
    }

    // Drive the valve closed on every boot. De-energised relays only stop the
    // motor — a reboot mid-fill would otherwise leave the valve physically open
    // while the controller believes it is closed. Runs non-blocking from loop().
    startValveClose(afterClose);

    Serial.println("FillController initialised.");
    Serial.printf("  Mode          : %s\n",
                  m_config.valveMode == 1 ? "auto" : m_config.valveMode == 2 ? "open"
                                                                             : "closed");
    Serial.printf("  Target        : %.1f %%\n", m_config.targetLevel_pc);
    Serial.printf("  Low threshold : %.1f %%\n", m_config.lowThreshold_pc);
    Serial.printf("  Max duration  : %u min\n", m_config.maxDuration_min);
    Serial.printf("  State         : %s\n", stateStr());
}

void FillController::update()
{
    // Post-close leak check runs whenever the valve is meant to be shut
    if (m_leakWatch && (m_state == FillState::IDLE || m_state == FillState::COOLDOWN ||
                        m_state == FillState::DISABLED_STATE || m_state == FillState::FAULT_TIMEOUT))
    {
        updateLeakWatch();
        if (m_state == FillState::VALVE_OPENING)
            return;  // reclose cycle started
    }

    switch (m_state)
    {
    case FillState::DISABLED_STATE:
        // Nothing to do — waiting for enable
        break;

    case FillState::IDLE:
        // Check sensor connectivity and health — but only fault after a
        // streak of consecutive bad reads, so a single transient I2C blip
        // doesn't kill fill function.
        if (!m_tank3->connected() || !m_tank3->sensorOk())
        {
            if (m_sensorBadStreak < 0xFF) m_sensorBadStreak++;
            if (m_sensorBadStreak >= FILL_SENSOR_FAULT_THRESHOLD)
            {
                transitionTo(FillState::FAULT_SENSOR);
            }
            break;
        }
        m_sensorBadStreak = 0;
        // Check if tank level has dropped below low threshold (auto-fill)
        if (m_tank3->pc() <= m_config.lowThreshold_pc)
        {
            m_fillReason = FillReason::AUTO_LOW;
            transitionTo(FillState::VALVE_OPENING);
        }
        break;

    case FillState::VALVE_OPENING:
        updateValveOpening();
        break;

    case FillState::FILLING:
        updateFilling();
        break;

    case FillState::VALVE_CLOSING:
        updateValveClosing();
        break;

    case FillState::COOLDOWN:
        if ((millis() - m_cooldownStart_ms) >= FILL_COOLDOWN_MS)
        {
            Serial.println("Fill: Cooldown complete.");
            transitionTo(FillState::IDLE);
        }
        break;

    case FillState::FAULT_SENSOR:
        // Auto-recover only after sensor reads valid continuously for SENSOR_RECOVERY_MS (30s).
        // Checking millis() here (not iteration count) guards against loop() running
        // at ~10kHz making a 3-iteration counter meaningless.
        if (m_tank3->sensorOk())
        {
            if (m_sensorRecoveryStart_ms == 0)
                m_sensorRecoveryStart_ms = millis();  // start timer on first valid read
            if ((millis() - m_sensorRecoveryStart_ms) >= SENSOR_RECOVERY_MS)
            {
                Serial.println("Fill: Sensor recovered after 30s of valid reads.");
                transitionTo(FillState::IDLE);
            }
        }
        else
        {
            m_sensorRecoveryStart_ms = 0;  // reset timer on any bad read
        }
        break;

    case FillState::FAULT_TIMEOUT:
        // Waiting for HA clear_fault command — timeout requires manual inspection
        break;
    }
}

// Valve opening sub-state machine ////////////////////////////////////////////////

void FillController::updateValveOpening()
{
    uint32_t elapsed = millis() - m_valvePhaseStart_ms;

    switch (m_valvePhase)
    {
    case ValvePhase::SET_DIRECTION:
        if (elapsed >= VALVE_SETTLE_MS)
        {
            // Direction relay has settled — apply power
            powerOn();
            m_valvePhase         = ValvePhase::POWERING;
            m_valvePhaseStart_ms = millis();
            Serial.printf("Valve: Power ON (%s, %u ms).\n", m_moveDirOpen ? "opening" : "adjusting", m_moveDuration_ms);
        }
        break;

    case ValvePhase::POWERING:
        if (elapsed >= m_moveDuration_ms)
        {
            // Fill position reached — cut power, transition to FILLING
            powerOff();
            m_valvePos_ms = m_moveTarget_ms;
            m_valvePhase  = ValvePhase::DONE;
            Serial.printf("Valve: Move complete at %u%%, power OFF.\n", valvePosPc());
            if (m_reclosing)
                startValveClose(m_pendingState);  // reclose cycle: open end reached, close again
            else
                transitionTo(FillState::FILLING);
        }
        break;

    case ValvePhase::DONE:
        // Should not reach here, but just in case
        if (m_reclosing)
            startValveClose(m_pendingState);
        else
            transitionTo(FillState::FILLING);
        break;
    }
}

// Filling state — monitor level and timeouts /////////////////////////////////////

void FillController::updateFilling()
{
    // Periodic diagnostic log — print actual config every 60s during fill
    if ((millis() - m_lastDiagLog_ms) >= 60000UL)
    {
        const char* modeNames[] = {"closed", "auto", "open"};
        uint8_t mi              = m_config.valveMode > 2 ? 0 : m_config.valveMode;
        Serial.printf("Fill: Active config — target=%.1f%%, low=%.1f%%, mode=%s, tank3=%.1f%%\n",
                      m_config.targetLevel_pc, m_config.lowThreshold_pc, modeNames[mi], m_tank3->pc());
        m_lastDiagLog_ms = millis();
    }

    // Check sensor connectivity and health — gated by a streak so a single
    // transient bad read during fill doesn't slam the valve closed.
    if (!m_tank3->connected() || !m_tank3->sensorOk())
    {
        if (m_sensorBadStreak < 0xFF) m_sensorBadStreak++;
        if (m_sensorBadStreak >= FILL_SENSOR_FAULT_THRESHOLD)
        {
            Serial.println("Fill: Sensor fault during fill.");
            startValveClose(FillState::FAULT_SENSOR);
        }
        return;
    }
    m_sensorBadStreak = 0;

    // Safety overflow — always applies, even in OPEN mode
    if (m_tank3->pc() >= FILL_OVERFLOW_CAP_PC)
    {
        Serial.printf("Fill: Safety overflow limit reached (%.0f%%).\n", FILL_OVERFLOW_CAP_PC);
        faultClose(FillFault::OVERFLOW_CAP);
        return;
    }

    // Progress check — always applies. Valve open but level not rising means
    // no supply, a stuck valve, draw matching inflow, or an overflow/spill.
    uint32_t window_ms = millis() - m_progressRef_ms;
    if (window_ms >= FILL_PROGRESS_WINDOW_MS)
    {
        float rise_pc   = m_tank3->pc() - m_progressRef_pc;
        m_fillRate_pcpm = rise_pc / (window_ms / 60000.0f);
        Serial.printf("Fill: Progress %.2f%% in %lu s (%.2f %%/min).\n",
                      rise_pc, window_ms / 1000UL, m_fillRate_pcpm);
        if (rise_pc < FILL_PROGRESS_MIN_PC)
        {
            Serial.printf("Fill: No progress — level rose %.2f%% (< %.2f%%) in %lu min.\n",
                          rise_pc, FILL_PROGRESS_MIN_PC, window_ms / 60000UL);
            faultClose(FillFault::NO_PROGRESS);
            return;
        }
        m_progressRef_pc = m_tank3->pc();
        m_progressRef_ms = millis();
    }

    // In AUTO mode, check if target level reached
    if ((ValveMode)m_config.valveMode != ValveMode::OPEN)
    {
        if (m_tank3->pc() >= m_config.targetLevel_pc)
        {
            Serial.println("Fill: Target level reached.");
            startValveClose(FillState::IDLE);
            return;
        }
    }

    // Check session timeout — hard max always applies (even in OPEN mode)
    uint32_t elapsed_ms = millis() - m_fillStart_ms;
    uint32_t hardMax_ms = (uint32_t)FILL_HARD_MAX_MIN * 60000UL;

    if ((ValveMode)m_config.valveMode != ValveMode::OPEN)
    {
        uint32_t maxDuration_ms = (uint32_t)m_config.maxDuration_min * 60000UL;
        if (elapsed_ms >= maxDuration_ms)
        {
            Serial.printf("Fill: Timeout after %lu min (level %.1f%%, target %.1f%%).\n",
                          elapsed_ms / 60000UL, m_tank3->pc(), m_config.targetLevel_pc);
            faultClose(FillFault::TIMEOUT);
            return;
        }
    }

    if (elapsed_ms >= hardMax_ms)
    {
        Serial.printf("Fill: Hard timeout after %lu min.\n", elapsed_ms / 60000UL);
        faultClose(FillFault::TIMEOUT);
        return;
    }
}

// Valve closing sub-state machine ////////////////////////////////////////////////

void FillController::updateValveClosing()
{
    uint32_t elapsed = millis() - m_valvePhaseStart_ms;

    switch (m_valvePhase)
    {
    case ValvePhase::SET_DIRECTION:
        if (elapsed >= VALVE_SETTLE_MS)
        {
            // Direction relay has settled — apply power
            powerOn();
            m_valvePhase         = ValvePhase::POWERING;
            m_valvePhaseStart_ms = millis();
            Serial.println("Valve: Power ON (closing).");
        }
        break;

    case ValvePhase::POWERING:
        if (elapsed >= VALVE_TRAVEL_MS)
        {
            // Valve fully closed — cut power, transition to pending state
            powerOff();
            m_valvePos_ms = 0;  // end stop reached — position re-referenced
            m_valvePhase  = ValvePhase::DONE;
            Serial.println("Valve: Close complete, power OFF.");
            transitionTo(m_pendingState);
            armLeakWatch();
        }
        break;

    case ValvePhase::DONE:
        // Should not reach here, but just in case
        transitionTo(m_pendingState);
        armLeakWatch();
        break;
    }
}

// MQTT command handlers ///////////////////////////////////////////////////////////

void FillController::setEnabled(bool en)
{
    // Backward compat: delegate to valve mode
    setValveMode(en ? (uint8_t)ValveMode::AUTO : (uint8_t)ValveMode::CLOSED);
}

void FillController::setTarget(float pc)
{
    if (pc < 10.0f) pc = 10.0f;
    if (pc > FILL_TARGET_MAX_PC) pc = FILL_TARGET_MAX_PC;  // stay below the overflow cap
    m_config.targetLevel_pc = pc;
    saveConfig();
    Serial.printf("Fill: target = %.1f %%\n", pc);
}

void FillController::setLowThreshold(float pc)
{
    if (pc < 5.0f) pc = 5.0f;
    if (pc > 95.0f) pc = 95.0f;
    m_config.lowThreshold_pc = pc;
    saveConfig();
    Serial.printf("Fill: low threshold = %.1f %%\n", pc);
}

void FillController::setMaxDuration(uint16_t minutes)
{
    if (minutes < 1) minutes = 1;
    if (minutes > FILL_HARD_MAX_MIN) minutes = FILL_HARD_MAX_MIN;
    m_config.maxDuration_min = minutes;
    saveConfig();
    Serial.printf("Fill: max duration = %u min\n", minutes);
}

void FillController::setValveOpening(uint8_t pc)
{
    if (pc < FILL_OPENING_MIN_PC) pc = FILL_OPENING_MIN_PC;
    if (pc > 100) pc = 100;
    if (pc == valveOpeningPc())
        return;
    m_config.valveOpening_pc = pc;
    saveConfig();
    Serial.printf("Fill: valve opening = %u %%\n", pc);

    // Apply live: move the valve to the new position without interrupting the fill
    if (m_state == FillState::FILLING || m_state == FillState::VALVE_OPENING)
        startValveOpen();
}

void FillController::setValveMode(uint8_t mode)
{
    if (mode > 2) return;
    m_config.valveMode   = mode;
    m_config.fillEnabled = (mode == (uint8_t)ValveMode::AUTO);
    saveConfig();

    const char* modeNames[] = {"closed", "auto", "open"};
    Serial.printf("Fill: valve mode = %s\n", modeNames[mode]);

    switch ((ValveMode)mode)
    {
    case ValveMode::CLOSED:
        if (m_state == FillState::FILLING || m_state == FillState::VALVE_OPENING)
            startValveClose(FillState::DISABLED_STATE);
        else if (m_state != FillState::VALVE_CLOSING)
            transitionTo(FillState::DISABLED_STATE);
        else
            m_pendingState = FillState::DISABLED_STATE;
        break;

    case ValveMode::AUTO:
        if (m_state == FillState::DISABLED_STATE)
            transitionTo(FillState::IDLE);
        // If currently in manual OPEN override, close and go to IDLE
        if (m_state == FillState::FILLING && m_fillReason == FillReason::HA_OVERRIDE)
            startValveClose(FillState::IDLE);
        break;

    case ValveMode::OPEN:
        m_fillReason   = FillReason::HA_OVERRIDE;
        m_openStart_ms = millis();
        if (m_state == FillState::IDLE || m_state == FillState::DISABLED_STATE ||
            m_state == FillState::COOLDOWN || m_state == FillState::FAULT_SENSOR ||
            m_state == FillState::FAULT_TIMEOUT)
            transitionTo(FillState::VALVE_OPENING);
        else if (m_state == FillState::VALVE_CLOSING)
        {
            // Previously ignored: mode showed "open" but the valve stayed closed.
            // Reverse immediately (mirrors stop-during-open) — unless the close
            // was triggered by a fault, which must be allowed to complete.
            if (m_pendingState == FillState::FAULT_SENSOR || m_pendingState == FillState::FAULT_TIMEOUT)
                Serial.println("Fill: Open ignored — valve closing due to fault.");
            else
                transitionTo(FillState::VALVE_OPENING);
        }
        break;
    }
}

void FillController::commandStart()
{
    if (m_state == FillState::IDLE)
    {
        m_fillReason = FillReason::MANUAL_COMMAND;
        transitionTo(FillState::VALVE_OPENING);
    }
    else if (m_state == FillState::DISABLED_STATE)
    {
        Serial.println("Fill: Cannot start — fill is disabled.");
    }
    else
    {
        Serial.printf("Fill: Cannot start — current state: %s\n", stateStr());
    }
}

void FillController::commandStop()
{
    if (m_state == FillState::FILLING)
    {
        Serial.println("Fill: Manual stop command.");
        startValveClose(FillState::COOLDOWN);
    }
    else if (m_state == FillState::VALVE_OPENING)
    {
        // Abort opening — close instead
        Serial.println("Fill: Manual stop during valve open — closing.");
        startValveClose(FillState::COOLDOWN);
    }
}

void FillController::commandClearFault()
{
    if (m_state == FillState::FAULT_SENSOR)
    {
        if (m_tank3->sensorOk())
        {
            Serial.println("Fill: Sensor fault cleared (sensor reads valid).");
            transitionTo(FillState::IDLE);
        }
        else
        {
            Serial.println("Fill: Cannot clear sensor fault — sensor still invalid.");
        }
    }
    else if (m_state == FillState::FAULT_TIMEOUT)
    {
        Serial.println("Fill: Timeout fault cleared.");
        m_leakWatch = false;  // valve inspected — don't re-latch valve_stuck_open straight away
        transitionTo(FillState::IDLE);
    }
    else
    {
        Serial.printf("Fill: No fault to clear (state: %s).\n", stateStr());
    }
}

// State accessors /////////////////////////////////////////////////////////////////

const char* FillController::stateStr() const
{
    switch (m_state)
    {
    case FillState::DISABLED_STATE: return "disabled";
    case FillState::IDLE:           return "idle";
    case FillState::VALVE_OPENING:  return "valve_opening";
    case FillState::FILLING:        return "filling";
    case FillState::VALVE_CLOSING:  return "valve_closing";
    case FillState::COOLDOWN:       return "cooldown";
    case FillState::FAULT_SENSOR:   return "fault_sensor";
    case FillState::FAULT_TIMEOUT:  return "fault_timeout";
    default:                        return "unknown";
    }
}

const char* FillController::faultStr() const
{
    switch (m_state)
    {
    case FillState::FAULT_SENSOR:
        switch (m_tank3->sensorFault())
        {
        case SensorFault::WIRE_BREAK:    return "sensor_wire_break";
        case SensorFault::SHORT_CIRCUIT: return "sensor_short";
        default:                         return "sensor_unknown";
        }
    case FillState::FAULT_TIMEOUT:
        switch (m_fault)
        {
        case FillFault::NO_PROGRESS:  return "no_progress";
        case FillFault::OVERFLOW_CAP: return "overflow_cap";
        case FillFault::VALVE_STUCK:  return "valve_stuck_open";
        default:                      return "fill_timeout";
        }
    default: return "none";
    }
}

const char* FillController::fillReasonStr() const
{
    switch (m_fillReason)
    {
    case FillReason::NONE:           return "none";
    case FillReason::AUTO_LOW:       return "auto_low";
    case FillReason::MANUAL_COMMAND: return "manual_command";
    case FillReason::HA_OVERRIDE:    return "ha_override";
    case FillReason::VALVE_RECLOSE:  return "valve_reclose";
    default:                         return "unknown";
    }
}

const char* FillController::valveStateStr() const
{
    switch (m_state)
    {
    case FillState::VALVE_OPENING:
        if (!m_moveDirOpen && m_valvePhase != ValvePhase::DONE)
            return "adjusting";  // partial move toward closed (opening reduced mid-fill)
        switch (m_valvePhase)
        {
        case ValvePhase::SET_DIRECTION: return "opening_settle";
        case ValvePhase::POWERING:      return "opening_travel";
        default:                        return "opening";
        }
    case FillState::VALVE_CLOSING:
        switch (m_valvePhase)
        {
        case ValvePhase::SET_DIRECTION: return "closing_settle";
        case ValvePhase::POWERING:      return "closing_travel";
        default:                        return "closing";
        }
    case FillState::FILLING:        return m_valvePos_ms >= VALVE_FULL_TRAVEL_MS ? "open" : "partial";
    case FillState::IDLE:
    case FillState::DISABLED_STATE:
    case FillState::COOLDOWN:
    case FillState::FAULT_SENSOR:
    case FillState::FAULT_TIMEOUT:
        return "closed";
    default:
        return "unknown";
    }
}

uint32_t FillController::fillElapsedMin() const
{
    if (m_state == FillState::FILLING || m_state == FillState::VALVE_OPENING)
        return (millis() - m_fillStart_ms) / 60000UL;
    return 0;
}

uint8_t FillController::valveOpeningPc() const
{
    uint8_t pc = m_config.valveOpening_pc;
    return (pc == 0 || pc > 100) ? 100 : pc;  // 0 = never set (pre-v0.10 EEPROM)
}

uint8_t FillController::valvePosPc() const
{
    return (uint8_t)((currentValvePos() * 100UL + VALVE_FULL_TRAVEL_MS / 2) / VALVE_FULL_TRAVEL_MS);
}

uint16_t FillController::currentValvePos() const
{
    if (m_valvePhase != ValvePhase::POWERING)
        return m_valvePos_ms;
    uint32_t el = millis() - m_valvePhaseStart_ms;
    if (m_moveDirOpen)
        return (uint16_t)min<uint32_t>(VALVE_FULL_TRAVEL_MS, m_moveStartPos_ms + el);
    return el >= m_moveStartPos_ms ? 0 : (uint16_t)(m_moveStartPos_ms - el);
}

uint16_t FillController::openTarget_ms() const
{
    return (uint16_t)((uint32_t)VALVE_FULL_TRAVEL_MS * valveOpeningPc() / 100UL);
}

// Valve hardware control (immediate GPIO writes) /////////////////////////////////

void FillController::powerOn()
{
    digitalWrite(m_powerPin, LOW);  // Active-LOW: LOW = relay energised = 12V applied
}

void FillController::powerOff()
{
    digitalWrite(m_powerPin, HIGH);  // Active-LOW: HIGH = relay de-energised = no power
}

void FillController::dirOpen()
{
    digitalWrite(m_dirPin, LOW);  // Active-LOW: LOW = relay energised = route to 'Open' wire
}

void FillController::dirClose()
{
    digitalWrite(m_dirPin, HIGH);  // Active-LOW: HIGH = relay de-energised = route to 'Close' wire
}

void FillController::allOff()
{
    powerOff();
    dirClose();  // default direction is close (de-energised)
}

// Valve sequence initiators //////////////////////////////////////////////////////

void FillController::startValveOpen()
{
    // Freeze the position estimate, then cut power — may be reversing a
    // valve that is mid-close or re-targeting one that is mid-open.
    uint16_t pos = currentValvePos();
    powerOff();
    m_valvePos_ms        = pos;
    m_moveStartPos_ms    = pos;
    m_moveTarget_ms      = openTarget_ms();
    m_valvePhaseStart_ms = millis();
    m_state              = FillState::VALVE_OPENING;
    m_keepAwake          = true;

    if (m_moveTarget_ms >= VALVE_FULL_TRAVEL_MS)
    {
        if (pos >= VALVE_FULL_TRAVEL_MS)
        {
            m_valvePhase = ValvePhase::DONE;  // already fully open
            return;
        }
        // Full open: drive to the end stop (with margin) — re-references position
        m_moveDirOpen     = true;
        m_moveDuration_ms = VALVE_TRAVEL_MS;
    }
    else
    {
        // Partial open: run the motor for the difference from the estimated position
        uint16_t diff = (m_moveTarget_ms > pos) ? (m_moveTarget_ms - pos) : (pos - m_moveTarget_ms);
        if (diff < VALVE_MIN_MOVE_MS)
        {
            m_valvePos_ms = m_moveTarget_ms;
            m_valvePhase  = ValvePhase::DONE;  // close enough — no move
            return;
        }
        m_moveDirOpen     = (m_moveTarget_ms > pos);
        m_moveDuration_ms = diff;
    }

    // Step 1: Set direction, then wait for contacts to settle
    if (m_moveDirOpen)
        dirOpen();
    else
        dirClose();
    m_valvePhase = ValvePhase::SET_DIRECTION;
    Serial.printf("Valve: %u%% -> %u%% (%s), waiting for settle.\n",
                  (unsigned)(pos * 100UL / VALVE_FULL_TRAVEL_MS), valveOpeningPc(),
                  m_moveDirOpen ? "open" : "close");
}

void FillController::startValveClose(FillState afterClose)
{
    // Freeze the position estimate, then cut power before switching direction
    uint16_t pos = currentValvePos();
    powerOff();
    m_pendingState    = afterClose;
    m_valvePos_ms     = pos;
    m_moveStartPos_ms = pos;
    m_moveTarget_ms   = 0;
    m_moveDirOpen     = false;
    m_moveDuration_ms = VALVE_TRAVEL_MS;  // always a full close — re-references at the end stop

    // Step 1: Set direction to Close, then wait for contacts to settle
    dirClose();
    m_valvePhase         = ValvePhase::SET_DIRECTION;
    m_valvePhaseStart_ms = millis();
    m_state              = FillState::VALVE_CLOSING;
    m_keepAwake          = true;
    Serial.printf("Valve: Direction -> CLOSE, pending state: %s.\n",
                  afterClose == FillState::IDLE ? "idle" : afterClose == FillState::COOLDOWN     ? "cooldown"
                                                       : afterClose == FillState::DISABLED_STATE ? "disabled"
                                                       : afterClose == FillState::FAULT_SENSOR   ? "fault_sensor"
                                                       : afterClose == FillState::FAULT_TIMEOUT  ? "fault_timeout"
                                                                                                 : "unknown");
}

void FillController::faultClose(FillFault fault)
{
    m_fault = fault;
    startValveClose(FillState::FAULT_TIMEOUT);
}

void FillController::startReclose(FillState afterClose)
{
    // Drive the full travel both ways (ignoring any trickle opening) to give a
    // jammed ball the most force and time to move, then close as normal.
    powerOff();
    m_reclosing          = true;
    m_leakLastAttempt_ms = millis();
    m_fillReason         = FillReason::VALVE_RECLOSE;
    m_fillStart_ms       = millis();
    m_pendingState       = afterClose;
    m_moveStartPos_ms    = m_valvePos_ms;
    m_moveTarget_ms      = VALVE_FULL_TRAVEL_MS;
    m_moveDirOpen        = true;
    m_moveDuration_ms    = VALVE_TRAVEL_MS;

    dirOpen();
    m_valvePhase         = ValvePhase::SET_DIRECTION;
    m_valvePhaseStart_ms = millis();
    m_state              = FillState::VALVE_OPENING;
    m_keepAwake          = true;
    Serial.printf("Fill: Reclose attempt %u — cycling valve open then closed.\n", m_leakAttempts);
}

// Post-close leak check //////////////////////////////////////////////////////////

void FillController::armLeakWatch()
{
    // A reclose cycle keeps its attempt count and the level at the original
    // close; any other close starts afresh
    if (!m_reclosing)
    {
        m_leakAttempts   = 0;
        m_leakStartValid = false;
    }
    m_reclosing         = false;
    m_leakWatch         = true;
    m_leakWatchStart_ms = millis();
    m_leakRef_ms        = millis();
    m_leakRefValid      = false;  // reference taken on the next valid read
}

void FillController::updateLeakWatch()
{
    if (!m_tank3->connected() || !m_tank3->sensorOk())
    {
        // No trustworthy level — restart the window once the sensor is back
        m_leakRefValid = false;
        return;
    }
    uint32_t now = millis();
    if (!m_leakStartValid)
    {
        m_leakStart_pc   = m_tank3->pc();
        m_leakStartValid = true;
    }
    if (!m_leakRefValid)
    {
        m_leakRef_pc   = m_tank3->pc();
        m_leakRef_ms   = now;
        m_leakRefValid = true;
        return;
    }
    if ((now - m_leakRef_ms) < LEAK_CHECK_WINDOW_MS)
        return;

    float rise_pc = m_tank3->pc() - m_leakRef_pc;
    m_leakRef_pc  = m_tank3->pc();
    m_leakRef_ms  = now;

    // Once Tank 3 overflows its level stops rising, so climbing to the overflow
    // since the close also counts as still leaking. A flat level there can't
    // show whether a reclose worked, so this errs towards more cycles: a cycle
    // on a sealed valve only lets in a few litres.
    bool rising   = rise_pc >= LEAK_RISE_MIN_PC;
    bool overflow = m_tank3->pc() >= LEAK_OVERFLOW_PC && m_leakStart_pc < LEAK_OVERFLOW_PC;
    bool latched  = m_fault == FillFault::VALVE_STUCK;

    if (!rising && !overflow)
    {
        if ((now - m_leakWatchStart_ms) >= LEAK_WATCH_MS)
        {
            if (m_leakAttempts > 0)
                Serial.printf("Fill: Leak check passed after %u reclose attempt(s).\n", m_leakAttempts);
            m_leakWatch = false;
        }
        return;
    }

    Serial.printf("Fill: Level rose %.2f%% in %lu min (now %.1f%%, %.1f%% at close) with the valve closed — valve may be stuck open.\n",
                  rise_pc, LEAK_CHECK_WINDOW_MS / 60000UL, m_tank3->pc(), m_leakStart_pc);

    if (m_leakAttempts < LEAK_RECLOSE_ATTEMPTS)
    {
        m_leakAttempts++;
        startReclose(m_state == FillState::COOLDOWN ? FillState::IDLE : m_state);
    }
    else if (!latched || (now - m_leakLastAttempt_ms) >= LEAK_RETRY_MS)
    {
        // Quick attempts used up: latch the fault (blocks auto-fill until cleared)
        // and keep retrying at a slower pace while it is still rising or overflowing.
        if (!latched)
            Serial.printf("Fill: Still leaking after %u reclose attempts — valve_stuck_open latched.\n",
                          m_leakAttempts);
        m_fault = FillFault::VALVE_STUCK;
        if (m_leakAttempts < 0xFF) m_leakAttempts++;
        startReclose(FillState::FAULT_TIMEOUT);
    }
}

// State transitions ///////////////////////////////////////////////////////////////

void FillController::transitionTo(FillState newState)
{
    FillState oldState = m_state;
    m_state            = newState;

    // Persist the fault latch on entry to FAULT_TIMEOUT; clear it on any exit
    // (Clear Fault button, or an explicit OPEN override from the fault state).
    if (newState != FillState::FAULT_TIMEOUT)
        m_fault = FillFault::NONE;
    if (m_config.latchedFault != (uint8_t)m_fault)
    {
        m_config.latchedFault = (uint8_t)m_fault;
        saveConfig();
    }

    if (newState != FillState::FILLING)
        m_fillRate_pcpm = 0;

    // On-exit: if leaving FILLING, log duration
    if (oldState == FillState::FILLING && newState != FillState::FILLING)
    {
        uint32_t elapsed = (millis() - m_fillStart_ms) / 1000UL;
        Serial.printf("Fill: Was filling for %u s.\n", elapsed);
    }

    // On-entry actions
    switch (newState)
    {
    case FillState::DISABLED_STATE:
        allOff();
        m_keepAwake  = false;
        m_fillReason = FillReason::NONE;
        Serial.println("Fill: State -> DISABLED.");
        break;

    case FillState::IDLE:
        allOff();
        m_keepAwake       = false;
        m_fillReason      = FillReason::NONE;
        m_sensorBadStreak = 0;
        Serial.println("Fill: State -> IDLE.");
        break;

    case FillState::VALVE_OPENING:
        m_fillStart_ms = millis();
        m_leakWatch    = false;  // a real fill — the next close re-arms the leak check
        m_reclosing    = false;
        m_leakAttempts = 0;
        startValveOpen();
        Serial.printf("Fill: State -> VALVE_OPENING (reason: %s).\n", fillReasonStr());
        break;

    case FillState::FILLING:
        // Valve is now fully open — water is flowing
        m_keepAwake       = true;
        m_sensorBadStreak = 0;
        m_lastDiagLog_ms  = 0;  // force immediate diagnostic log
        m_progressRef_pc  = m_tank3->pc();
        m_progressRef_ms  = millis();
        Serial.printf("Fill: State -> FILLING (valve %u%% open).\n", valvePosPc());
        break;

    case FillState::VALVE_CLOSING:
        // Handled by startValveClose() — should not be called directly via transitionTo
        break;

    case FillState::COOLDOWN:
        allOff();
        m_cooldownStart_ms = millis();
        m_keepAwake        = true;  // Stay awake during cooldown
        m_fillReason       = FillReason::NONE;
        Serial.println("Fill: State -> COOLDOWN.");
        break;

    case FillState::FAULT_SENSOR:
        allOff();
        m_keepAwake              = false;
        m_sensorRecoveryStart_ms = 0;
        Serial.printf("Fill: State -> FAULT_SENSOR (%s).\n", faultStr());
        break;

    case FillState::FAULT_TIMEOUT:
        allOff();
        m_keepAwake = false;
        Serial.println("Fill: State -> FAULT_TIMEOUT.");
        break;
    }
}

// EEPROM persistence //////////////////////////////////////////////////////////////

void FillController::loadConfig()
{
    EEPROM.get(FILL_CONFIG_ADDR, m_config);

    if (m_config.magic != FILL_CONFIG_MAGIC || m_config.version != FILL_CONFIG_VERSION)
    {
        Serial.printf("Fill: EEPROM config invalid (magic=%#04x, ver=%u) — RESET to defaults.\n",
                      m_config.magic, m_config.version);
        setDefaults();
        saveConfig();
    }
    else
    {
        Serial.printf("Fill: Config loaded from EEPROM (target=%.1f%%, low=%.1f%%, mode=%u).\n",
                      m_config.targetLevel_pc, m_config.lowThreshold_pc, m_config.valveMode);
        if (m_config.targetLevel_pc > FILL_TARGET_MAX_PC)
        {
            Serial.printf("Fill: Target %.1f%% would trip the overflow cap — clamped to %.1f%%.\n",
                          m_config.targetLevel_pc, FILL_TARGET_MAX_PC);
            m_config.targetLevel_pc = FILL_TARGET_MAX_PC;
            saveConfig();
        }
    }
}

void FillController::saveConfig()
{
    m_config.magic   = FILL_CONFIG_MAGIC;
    m_config.version = FILL_CONFIG_VERSION;
    EEPROM.put(FILL_CONFIG_ADDR, m_config);
    EEPROM.commit();
}

void FillController::setDefaults()
{
    m_config.magic           = FILL_CONFIG_MAGIC;
    m_config.version         = FILL_CONFIG_VERSION;
    m_config.fillEnabled     = false;
    m_config.targetLevel_pc  = 80.0f;
    m_config.lowThreshold_pc = 60.0f;
    m_config.maxDuration_min = 240;
    m_config.valveMode       = (uint8_t)ValveMode::CLOSED;
    m_config.latchedFault    = (uint8_t)FillFault::NONE;
    m_config.valveOpening_pc = 100;
    memset(m_config._reserved, 0, sizeof(m_config._reserved));
}
