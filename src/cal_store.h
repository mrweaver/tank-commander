#ifndef _H_CAL_STORE_
#define _H_CAL_STORE_

#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>

// Tank calibrations (head_mm = mA * m + b), saved in EEPROM (NVS on the ESP32)
// after the FillConfig block so they survive reboots without HA re-sending them.
// Only tanks that have been calibrated over MQTT are restored; the rest keep the
// compiled-in defaults. Shares the EEPROM area FillController::begin() opens.
#define CAL_CONFIG_MAGIC   0xCA01
#define CAL_CONFIG_VERSION 1
#define CAL_CONFIG_ADDR    64  // FillConfig sits at 0 and is well under 64 bytes
#define CAL_TANKS          3

struct CalConfig
{
    uint16_t magic;
    uint8_t version;
    uint8_t setMask;  // bit n = tank n+1 has a saved calibration
    float m[CAL_TANKS];
    float b[CAL_TANKS];
};

inline bool calValid(float m, float b)
{
    return isfinite(m) && isfinite(b) && m > 0.0f;
}

/// Read the saved calibrations. Returns false (and a cleared config) if none are saved.
inline bool calLoad(CalConfig& cfg)
{
    EEPROM.get(CAL_CONFIG_ADDR, cfg);
    if (cfg.magic != CAL_CONFIG_MAGIC || cfg.version != CAL_CONFIG_VERSION)
    {
        memset(&cfg, 0, sizeof(cfg));
        return false;
    }
    for (uint8_t i = 0; i < CAL_TANKS; i++)
        if (!calValid(cfg.m[i], cfg.b[i]))
            cfg.setMask &= ~(1 << i);  // ignore a corrupt entry
    return true;
}

/// Save one tank's calibration. Skips the write when nothing changed, so a
/// retained calibration re-delivered on every MQTT reconnect doesn't wear the flash.
inline bool calSave(uint8_t idx, float m, float b)
{
    if (idx >= CAL_TANKS || !calValid(m, b))
        return false;
    CalConfig cfg;
    calLoad(cfg);
    if ((cfg.setMask & (1 << idx)) && cfg.m[idx] == m && cfg.b[idx] == b)
        return true;
    cfg.magic   = CAL_CONFIG_MAGIC;
    cfg.version = CAL_CONFIG_VERSION;
    cfg.m[idx]  = m;
    cfg.b[idx]  = b;
    cfg.setMask |= (1 << idx);
    EEPROM.put(CAL_CONFIG_ADDR, cfg);
    return EEPROM.commit();
}

#endif  // _H_CAL_STORE_
