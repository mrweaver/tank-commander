#ifndef _H_FILL_CONTROLLER_
#define _H_FILL_CONTROLLER_

#include <Arduino.h>
#include <EEPROM.h>
#include "tank.h"

#define FILL_CONFIG_MAGIC    0xFC02
#define FILL_CONFIG_VERSION  2
#define FILL_CONFIG_ADDR     0
#define FILL_EEPROM_SIZE     128
#define FILL_COOLDOWN_MS     30000   // 30 second cooldown after fill stops
#define FILL_HARD_MAX_MIN    60      // absolute backstop, not changeable via MQTT

// Motorised ball valve timing
#define VALVE_SETTLE_MS      50      // relay contact settle time before applying power
#define VALVE_TRAVEL_MS      8500    // full valve travel time (8s physical + 500ms margin)

/// Fill controller states. The valve states (VALVE_OPENING, VALVE_CLOSING)
/// run a non-blocking sub-sequence to actuate the 3-wire motorised ball valve.
enum class FillState : uint8_t {
    DISABLED,
    IDLE,
    VALVE_OPENING,   // non-blocking valve open sequence in progress
    FILLING,         // valve fully open, water flowing
    VALVE_CLOSING,   // non-blocking valve close sequence in progress
    COOLDOWN,
    FAULT_SENSOR,
    FAULT_TIMEOUT
};

enum class FillReason : uint8_t {
    NONE,
    AUTO_LOW,
    MANUAL_COMMAND,
    HA_OVERRIDE
};

/// Sub-phases for non-blocking valve actuation (used during VALVE_OPENING / VALVE_CLOSING)
enum class ValvePhase : uint8_t {
    SET_DIRECTION,   // direction relay set, waiting VALVE_SETTLE_MS
    POWERING,        // power relay energised, waiting VALVE_TRAVEL_MS
    DONE             // sequence complete
};

struct FillConfig {
    uint16_t magic;             // 0xFC02 — validates stored data
    uint8_t  version;           // struct version for future migration
    bool     fillEnabled;       // master enable/disable
    float    targetLevel_pc;    // fill target percentage (default 80)
    float    lowThreshold_pc;   // auto-fill trigger percentage (default 60)
    uint16_t maxDuration_min;   // max fill duration per session in minutes (default 30)
    uint8_t  _reserved[8];      // padding for future fields
};

class FillController {
public:
    FillController(uint8_t powerPin, uint8_t dirPin, Tank* tank3);

    void begin();       // Init GPIO (boot-safe), load config from EEPROM
    void update();      // Called every loop iteration — runs state machine

    // MQTT command handlers
    void setEnabled(bool en);
    void setTarget(float pc);
    void setLowThreshold(float pc);
    void setMaxDuration(uint16_t minutes);
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

    // Config persistence
    void saveConfig();
    void loadConfig();

private:
    uint8_t m_powerPin;     // D6 — controls 12V supply to valve motor
    uint8_t m_dirPin;       // D7 — controls direction (HIGH=close, LOW=open)
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
    FillState m_pendingState;   // state to transition to after VALVE_CLOSING completes

    // Valve hardware control (immediate GPIO writes)
    void powerOn();
    void powerOff();
    void dirOpen();
    void dirClose();
    void allOff();  // safety: both relays de-energised

    // Valve sequence initiators
    void startValveOpen();
    void startValveClose(FillState afterClose);

    // State machine
    void updateValveOpening();
    void updateFilling();
    void updateValveClosing();
    void transitionTo(FillState newState);
    void setDefaults();
};

#endif // _H_FILL_CONTROLLER_
