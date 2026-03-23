#include "fill_controller.h"

FillController::FillController(uint8_t powerPin, uint8_t dirPin, Tank* tank3)
    : m_powerPin(powerPin),
      m_dirPin(dirPin),
      m_tank3(tank3),
      m_state(FillState::DISABLED),
      m_fillReason(FillReason::NONE),
      m_keepAwake(false),
      m_fillStart_ms(0),
      m_cooldownStart_ms(0),
      m_valvePhase(ValvePhase::DONE),
      m_valvePhaseStart_ms(0),
      m_pendingState(FillState::IDLE),
      m_sensorRecoveryCount(0),
      m_openStart_ms(0)
{
}

void FillController::begin()
{
    // Boot-safe relay init should already be done in setup() before this call,
    // but reinforce here as defense-in-depth
    allOff();

    EEPROM.begin(FILL_EEPROM_SIZE);
    loadConfig();

    switch ((ValveMode)m_config.valveMode)
    {
    case ValveMode::AUTO:
        m_state = FillState::IDLE;
        break;
    case ValveMode::OPEN:
        m_fillReason = FillReason::HA_OVERRIDE;
        m_openStart_ms = millis();
        m_state = FillState::IDLE; // will transition to VALVE_OPENING on first update
        break;
    case ValveMode::CLOSED:
    default:
        m_state = FillState::DISABLED;
        break;
    }

    Serial.println("FillController initialised.");
    Serial.printf("  Mode          : %s\n",
        m_config.valveMode == 1 ? "auto" : m_config.valveMode == 2 ? "open" : "closed");
    Serial.printf("  Target        : %.1f %%\n", m_config.targetLevel_pc);
    Serial.printf("  Low threshold : %.1f %%\n", m_config.lowThreshold_pc);
    Serial.printf("  Max duration  : %u min\n", m_config.maxDuration_min);
    Serial.printf("  State         : %s\n", stateStr());
}

void FillController::update()
{
    switch (m_state)
    {
    case FillState::DISABLED:
        // Nothing to do — waiting for enable
        break;

    case FillState::IDLE:
        // Check sensor health first
        if (!m_tank3->sensorOk())
        {
            transitionTo(FillState::FAULT_SENSOR);
            break;
        }
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
        // Auto-recover after consecutive valid sensor reads
        if (m_tank3->sensorOk())
        {
            m_sensorRecoveryCount++;
            if (m_sensorRecoveryCount >= SENSOR_RECOVERY_COUNT)
            {
                Serial.println("Fill: Sensor auto-recovered after consecutive valid reads.");
                m_sensorRecoveryCount = 0;
                transitionTo(FillState::IDLE);
            }
        }
        else
        {
            m_sensorRecoveryCount = 0;
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
            m_valvePhase = ValvePhase::POWERING;
            m_valvePhaseStart_ms = millis();
            Serial.println("Valve: Power ON (opening).");
        }
        break;

    case ValvePhase::POWERING:
        if (elapsed >= VALVE_TRAVEL_MS)
        {
            // Valve fully open — cut power, transition to FILLING
            powerOff();
            m_valvePhase = ValvePhase::DONE;
            Serial.println("Valve: Open complete, power OFF.");
            transitionTo(FillState::FILLING);
        }
        break;

    case ValvePhase::DONE:
        // Should not reach here, but just in case
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
        uint8_t mi = m_config.valveMode > 2 ? 0 : m_config.valveMode;
        Serial.printf("Fill: Active config — target=%.1f%%, low=%.1f%%, mode=%s, tank3=%.1f%%\n",
            m_config.targetLevel_pc, m_config.lowThreshold_pc, modeNames[mi], m_tank3->pc());
        m_lastDiagLog_ms = millis();
    }

    // Check sensor health — fault takes priority
    if (!m_tank3->sensorOk())
    {
        Serial.println("Fill: Sensor fault during fill.");
        startValveClose(FillState::FAULT_SENSOR);
        return;
    }

    // Safety overflow — always applies, even in OPEN mode
    if (m_tank3->pc() >= 95.0f)
    {
        Serial.println("Fill: Safety overflow limit reached (95%).");
        startValveClose(FillState::FAULT_TIMEOUT);
        return;
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
            Serial.printf("Fill: Timeout after %lu min.\n", elapsed_ms / 60000UL);
            startValveClose(FillState::FAULT_TIMEOUT);
            return;
        }
    }

    if (elapsed_ms >= hardMax_ms)
    {
        Serial.printf("Fill: Hard timeout after %lu min.\n", elapsed_ms / 60000UL);
        startValveClose(FillState::FAULT_TIMEOUT);
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
            m_valvePhase = ValvePhase::POWERING;
            m_valvePhaseStart_ms = millis();
            Serial.println("Valve: Power ON (closing).");
        }
        break;

    case ValvePhase::POWERING:
        if (elapsed >= VALVE_TRAVEL_MS)
        {
            // Valve fully closed — cut power, transition to pending state
            powerOff();
            m_valvePhase = ValvePhase::DONE;
            Serial.println("Valve: Close complete, power OFF.");
            transitionTo(m_pendingState);
        }
        break;

    case ValvePhase::DONE:
        // Should not reach here, but just in case
        transitionTo(m_pendingState);
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
    if (pc > 100.0f) pc = 100.0f;
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

void FillController::setValveMode(uint8_t mode)
{
    if (mode > 2) return;
    m_config.valveMode = mode;
    m_config.fillEnabled = (mode == (uint8_t)ValveMode::AUTO);
    saveConfig();

    const char* modeNames[] = {"closed", "auto", "open"};
    Serial.printf("Fill: valve mode = %s\n", modeNames[mode]);

    switch ((ValveMode)mode)
    {
    case ValveMode::CLOSED:
        if (m_state == FillState::FILLING || m_state == FillState::VALVE_OPENING)
            startValveClose(FillState::DISABLED);
        else if (m_state != FillState::VALVE_CLOSING)
            transitionTo(FillState::DISABLED);
        else
            m_pendingState = FillState::DISABLED;
        break;

    case ValveMode::AUTO:
        if (m_state == FillState::DISABLED)
            transitionTo(FillState::IDLE);
        // If currently in manual OPEN override, close and go to IDLE
        if (m_state == FillState::FILLING && m_fillReason == FillReason::HA_OVERRIDE)
            startValveClose(FillState::IDLE);
        break;

    case ValveMode::OPEN:
        m_fillReason = FillReason::HA_OVERRIDE;
        m_openStart_ms = millis();
        if (m_state == FillState::IDLE || m_state == FillState::DISABLED ||
            m_state == FillState::COOLDOWN || m_state == FillState::FAULT_SENSOR ||
            m_state == FillState::FAULT_TIMEOUT)
            transitionTo(FillState::VALVE_OPENING);
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
    else if (m_state == FillState::DISABLED)
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
    case FillState::DISABLED:      return "disabled";
    case FillState::IDLE:          return "idle";
    case FillState::VALVE_OPENING: return "valve_opening";
    case FillState::FILLING:       return "filling";
    case FillState::VALVE_CLOSING: return "valve_closing";
    case FillState::COOLDOWN:      return "cooldown";
    case FillState::FAULT_SENSOR:  return "fault_sensor";
    case FillState::FAULT_TIMEOUT: return "fault_timeout";
    default:                       return "unknown";
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
    case FillState::FAULT_TIMEOUT: return "fill_timeout";
    default:                       return "none";
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
    default:                         return "unknown";
    }
}

const char* FillController::valveStateStr() const
{
    switch (m_state)
    {
    case FillState::VALVE_OPENING:
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
    case FillState::FILLING: return "open";
    case FillState::IDLE:
    case FillState::DISABLED:
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

// Valve hardware control (immediate GPIO writes) /////////////////////////////////

void FillController::powerOn()
{
    digitalWrite(m_powerPin, LOW);  // Active-LOW: LOW = relay energised = 12V applied
}

void FillController::powerOff()
{
    digitalWrite(m_powerPin, HIGH); // Active-LOW: HIGH = relay de-energised = no power
}

void FillController::dirOpen()
{
    digitalWrite(m_dirPin, LOW);    // Active-LOW: LOW = relay energised = route to 'Open' wire
}

void FillController::dirClose()
{
    digitalWrite(m_dirPin, HIGH);   // Active-LOW: HIGH = relay de-energised = route to 'Close' wire
}

void FillController::allOff()
{
    powerOff();
    dirClose(); // default direction is close (de-energised)
}

// Valve sequence initiators //////////////////////////////////////////////////////

void FillController::startValveOpen()
{
    // Step 1: Set direction to Open, then wait for contacts to settle
    dirOpen();
    m_valvePhase = ValvePhase::SET_DIRECTION;
    m_valvePhaseStart_ms = millis();
    m_state = FillState::VALVE_OPENING;
    m_keepAwake = true;
    Serial.println("Valve: Direction -> OPEN, waiting for settle.");
}

void FillController::startValveClose(FillState afterClose)
{
    // Cut power immediately as safety measure before switching direction
    powerOff();
    m_pendingState = afterClose;

    // Step 1: Set direction to Close, then wait for contacts to settle
    dirClose();
    m_valvePhase = ValvePhase::SET_DIRECTION;
    m_valvePhaseStart_ms = millis();
    m_state = FillState::VALVE_CLOSING;
    m_keepAwake = true;
    Serial.printf("Valve: Direction -> CLOSE, pending state: %s.\n",
        afterClose == FillState::IDLE ? "idle" :
        afterClose == FillState::COOLDOWN ? "cooldown" :
        afterClose == FillState::DISABLED ? "disabled" :
        afterClose == FillState::FAULT_SENSOR ? "fault_sensor" :
        afterClose == FillState::FAULT_TIMEOUT ? "fault_timeout" : "unknown");
}

// State transitions ///////////////////////////////////////////////////////////////

void FillController::transitionTo(FillState newState)
{
    FillState oldState = m_state;
    m_state = newState;

    // On-exit: if leaving FILLING, log duration
    if (oldState == FillState::FILLING && newState != FillState::FILLING)
    {
        uint32_t elapsed = (millis() - m_fillStart_ms) / 1000UL;
        Serial.printf("Fill: Was filling for %u s.\n", elapsed);
    }

    // On-entry actions
    switch (newState)
    {
    case FillState::DISABLED:
        allOff();
        m_keepAwake = false;
        m_fillReason = FillReason::NONE;
        Serial.println("Fill: State -> DISABLED.");
        break;

    case FillState::IDLE:
        allOff();
        m_keepAwake = false;
        m_fillReason = FillReason::NONE;
        Serial.println("Fill: State -> IDLE.");
        break;

    case FillState::VALVE_OPENING:
        m_fillStart_ms = millis();
        startValveOpen();
        Serial.printf("Fill: State -> VALVE_OPENING (reason: %s).\n", fillReasonStr());
        break;

    case FillState::FILLING:
        // Valve is now fully open — water is flowing
        m_keepAwake = true;
        m_lastDiagLog_ms = 0; // force immediate diagnostic log
        Serial.println("Fill: State -> FILLING (valve open).");
        break;

    case FillState::VALVE_CLOSING:
        // Handled by startValveClose() — should not be called directly via transitionTo
        break;

    case FillState::COOLDOWN:
        allOff();
        m_cooldownStart_ms = millis();
        m_keepAwake = true; // Stay awake during cooldown
        m_fillReason = FillReason::NONE;
        Serial.println("Fill: State -> COOLDOWN.");
        break;

    case FillState::FAULT_SENSOR:
        allOff();
        m_keepAwake = false;
        m_sensorRecoveryCount = 0;
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
    }
}

void FillController::saveConfig()
{
    m_config.magic = FILL_CONFIG_MAGIC;
    m_config.version = FILL_CONFIG_VERSION;
    EEPROM.put(FILL_CONFIG_ADDR, m_config);
    EEPROM.commit();
}

void FillController::setDefaults()
{
    m_config.magic = FILL_CONFIG_MAGIC;
    m_config.version = FILL_CONFIG_VERSION;
    m_config.fillEnabled = false;
    m_config.targetLevel_pc = 80.0f;
    m_config.lowThreshold_pc = 60.0f;
    m_config.maxDuration_min = 30;
    m_config.valveMode = (uint8_t)ValveMode::CLOSED;
    memset(m_config._reserved, 0, sizeof(m_config._reserved));
}
