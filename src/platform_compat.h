#ifndef _H_PLATFORM_COMPAT_
#define _H_PLATFORM_COMPAT_

#include <Arduino.h>

#if defined(ESP8266)
#include <ESP8266WiFi.h>

inline void tcSetHostname(const char* hostname)
{
    WiFi.hostname(hostname);
}

inline void tcStartDeepSleep(uint64_t sleepTime_us)
{
    ESP.deepSleep(sleepTime_us, WAKE_NO_RFCAL);
}

inline const char* tcResetReasonStr()
{
    return "unknown";  // ESP8266 target retired — not implemented
}

#ifndef TC_BOARD_MODEL
#define TC_BOARD_MODEL "D1 Mini Lite"
#endif

#elif defined(ESP32)
#include <WiFi.h>
#include <esp_sleep.h>
#include <esp_system.h>

inline void tcSetHostname(const char* hostname)
{
    WiFi.setHostname(hostname);
}

inline void tcStartDeepSleep(uint64_t sleepTime_us)
{
    esp_sleep_enable_timer_wakeup(sleepTime_us);
    esp_deep_sleep_start();
}

inline const char* tcResetReasonStr()
{
    switch (esp_reset_reason())
    {
    case ESP_RST_POWERON:   return "power_on";
    case ESP_RST_EXT:       return "external_pin";
    case ESP_RST_SW:        return "software";  // includes OTA restart
    case ESP_RST_PANIC:     return "panic";
    case ESP_RST_INT_WDT:   return "interrupt_wdt";
    case ESP_RST_TASK_WDT:  return "task_wdt";
    case ESP_RST_WDT:       return "other_wdt";
    case ESP_RST_DEEPSLEEP: return "deep_sleep";
    case ESP_RST_BROWNOUT:  return "brownout";
    case ESP_RST_SDIO:      return "sdio";
    default:                return "unknown";
    }
}

#ifndef TC_BOARD_MODEL
#if defined(ARDUINO_MH_ET_LIVE_ESP32MINIKIT)
#define TC_BOARD_MODEL "MH ET LIVE ESP32MiniKit"
#else
#define TC_BOARD_MODEL "ESP32"
#endif
#endif

#else
#error Unsupported platform
#endif

#endif  // _H_PLATFORM_COMPAT_