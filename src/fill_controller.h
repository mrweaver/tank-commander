#ifndef _H_FILL_CONTROLLER_
#define _H_FILL_CONTROLLER_

#include <Arduino.h>
#include <EEPROM.h>
#include "tank.h"

#define FILL_CONFIG_MAGIC           0xFC03
#define FILL_CONFIG_VERSION         3
#define FILL_CONFIG_ADDR            0
#define FILL_EEPROM_SIZE            128
#define FILL_COOLDOWN_MS            30000  // 30 second cooldown after fill stops
#define SENSOR_RECOVERY_MS          30000  // sensor must read valid for 30s continuously to auto-clear fault
#define FILL_HARD_MAX_MIN           480    // absolute backstop, not changeable via MQTT (8 h — a full
                                           // 0->92% fill takes ~3.6 h at full flow, longer when trickling)
#define FILL_OPENING_MIN_PC         10     // smallest partial opening accepted (ball is still sealed below ~10-15%)
#define FILL_SENSOR_FAULT_THRESHOLD 3      // consecutive bad reads before transitioning to FAULT_SENSOR
#define FILL_OVERFLOW_CAP_PC        95.0f  // safety overflow cap — always closes the valve, even in OPEN mode
#define FILL_TARGET_MARGIN_PC       3.0f   // max target = cap - margin, so a normal fill never trips the cap
#define FILL_TARGET_MAX_PC          (FILL_OVERFLOW_CAP_PC - FILL_TARGET_MARGIN_PC)

// Fill progress (stall) detection: while the valve is open the level must rise
// by at least FILL_PROGRESS_MIN_PC every FILL_PROGRESS_WINDOW_MS. Observed mains
// fill rate is ~0.43 %/min (~2.2 % per window), so 0.5 % tolerates a moderate
// simultaneous draw but catches a dry supply, stuck valve, or overflow/spill.
#define FILL_PROGRESS_WINDOW_MS     (5UL * 60000UL)
#define FILL_PROGRESS_MIN_PC        0.5f

// Motorised ball valve timing
#define VALVE_SETTLE_MS 50    // relay contact settle time before applying power
#define VALVE_TRAVEL_MS      8500  // end-stop move time (8s physical + 500ms margin) — re-references position
#define VALVE_FULL_TRAVEL_MS 8000  // physical closed->open travel; basis for partial-opening move times
#define VALVE_MIN_MOVE_MS    150   // partial moves shorter than this are skipped

/// Fill controller states. The valve states (VALVE_OPENING, VALVE_CLOSING)
/// run a non-blocking sub-sequence to actuate the 3-wire motorised ball valve.
enum class FillState : uint8_t
{
    DISABLED_STATE,
    IDLE,
    VALVE_OPENING,  // non-blocking valve open sequence in progress
    FILLING,        // valve fully open, water flowing
    VALVE_CLOSING,  // non-blocking valve close sequence in progress
    COOLDOWN,
    FAULT_SENSOR,
    FAULT_TIMEOUT
};

enum class FillReason : uint8_t
{
    NONE,
    AUTO_LOW,
    MANUAL_COMMAND,
    HA_OVERRIDE
};

/// Classification of latched fill faults (FAULT_TIMEOUT state). Persisted to
/// EEPROM so a reboot cannot silently clear a fault and start a fresh fill.
enum class FillFault : uint8_t
{
    NONE         = 0,
    TIMEOUT      = 1,  // fill ran longer than max duration / hard max
    NO_PROGRESS  = 2,  // valve open but level not rising
    OVERFLOW_CAP = 3   // level reached FILL_OVERFLOW_CAP_PC
};

/// Valve mode selector — determines how the fill valve behaves
enum class ValveMode : uint8_t
{
    CLOSED = 0,  // Force valve closed (safe default)
    AUTO   = 1,  // Automatic fill via state machine (thresholds)
    OPEN   = 2   // Force valve open (manual override, subject to hard timeout)
};

/// Sub-phases for non-blocking valve actuation (used during VALVE_OPENING / VALVE_CLOSING)
enum class ValvePhase : uint8_t
{
    SET_DIRECTION,  // direction relay set, waiting VALVE_SETTLE_MS
    POWERING,       // power relay energised, waiting VALVE_TRAVEL_MS
    DONE            // sequence complete
};

struct FillConfig
{
    uint16_t magic;            // 0xFC03 — validates stored data
    uint8_t version;           // struct version for future migration
    bool fillEnabled;          // derived from valveMode (kept for compat)
    float targetLevel_pc;      // fill target percentage (default 80)
    float lowThreshold_pc;     // auto-fill trigger percentage (default 60)
    uint16_t maxDuration_min;  // max fill duration per session in minutes (default 30)
    uint8_t valveMode;         // 0=Closed, 1=Auto, 2=Open (see ValveMode enum)
    uint8_t latchedFault;      // FillFault — survives reboot until cleared (was _reserved[0], always 0)
    uint8_t valveOpening_pc;   // fill opening as % of travel time, 10-100 (was _reserved[1]; 0 => 100)
    uint8_t _reserved[5];      // padding for future fields
};

class FillController
{
public:
    FillController(uint8_t powerPin, uint8_t dirPin, Tank* tank3);

    void begin();   // Init GPIO (boot-safe), load config from EEPROM
    void update();  // Called every loop iteration — runs state machine

    // MQTT command handlers
    void setEnabled(bool en);
    void setTarget(float pc);
    void setLowThreshold(float pc);
    void setMaxDuration(uint16_t minutes);
    void setValveMode(uint8_t mode);
    void setValveOpening(uint8_t pc);  // trickle fill — applied live if a fill is in progress
    void commandStart();
    void commandStop();
    void commandClearFault();

    // State accessors
    FillState state() const { return m_state; }
    const char* stateStr() const;
    const char* faultStr() const;
    const char* fillReasonStr() const;
    const char* valveStateStr() const;
    bool isFilling() const { return m_state == FillState::FILLING || m_state == FillState::VALVE_OPENING; }
    bool isValveBusy() const { return m_state == FillState::VALVE_OPENING || m_state == FillState::VALVE_CLOSING; }
    bool keepAwake() const { return m_keepAwake; }
    const FillConfig& config() const { return m_config; }
    uint32_t fillElapsedMin() const;
    float fillRatePcPerMin() const { return m_fillRate_pcpm; }  // last measured rise rate (0 when not filling)
    uint8_t valveOpeningPc() const;                             // configured fill opening
    uint8_t valvePosPc() const;                                 // estimated current valve position

    // Config persistence
    void saveConfig();
    void loadConfig();

private:
    uint8_t m_powerPin;  // controls 12V supply to valve motor
    uint8_t m_dirPin;    // controls direction (HIGH=close, LOW=open)
    Tank* m_tank3;
    FillState m_state;
    FillConfig m_config;
    FillReason m_fillReason;
    bool m_keepAwake;

    uint32_t m_fillStart_ms;
    uint32_t m_cooldownStart_ms;

    // Valve actuation sub-state
    ValvePhase m_valvePhase;
    uint32_t m_valvePhaseStart_ms;
    FillState m_pendingState;           // state to transition to after VALVE_CLOSING completes
    uint32_t m_openStart_ms;            // timestamp when OPEN mode started (for hard timeout)
    uint32_t m_lastDiagLog_ms;          // last diagnostic log during fill
    uint32_t m_sensorRecoveryStart_ms;  // millis() when sensor first read valid after FAULT_SENSOR
    uint8_t m_sensorBadStreak;          // consecutive bad sensor reads in IDLE/FILLING (gates FAULT_SENSOR)
    FillFault m_fault;                  // classification while in FAULT_TIMEOUT
    float m_progressRef_pc;             // level at start of current progress window
    uint32_t m_progressRef_ms;          // millis() at start of current progress window
    float m_fillRate_pcpm;              // rise rate over the last completed window [%/min]

    // Time-based valve position estimate. 0 = closed, VALVE_FULL_TRAVEL_MS = open.
    // Re-referenced at the end stops: every full close sets 0, every full open
    // sets VALVE_FULL_TRAVEL_MS. Partial moves run the motor for the difference.
    uint16_t m_valvePos_ms;     // estimated position when the motor is not running
    uint16_t m_moveStartPos_ms;  // position at the start of the current move
    uint16_t m_moveTarget_ms;    // position at the end of the current move
    uint16_t m_moveDuration_ms;  // motor run time for the current move
    bool m_moveDirOpen;          // direction of the current move
    uint16_t currentValvePos() const;
    uint16_t openTarget_ms() const;

    // Valve hardware control (immediate GPIO writes)
    void powerOn();
    void powerOff();
    void dirOpen();
    void dirClose();
    void allOff();  // safety: both relays de-energised

    // Valve sequence initiators
    void startValveOpen();
    void startValveClose(FillState afterClose);
    void faultClose(FillFault fault);  // close valve, then latch FAULT_TIMEOUT with this classification

    // State machine
    void updateValveOpening();
    void updateFilling();
    void updateValveClosing();
    void transitionTo(FillState newState);
    void setDefaults();
};

#endif  // _H_FILL_CONTROLLER_
