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

#ifndef TC_BOARD_MODEL
#define TC_BOARD_MODEL "D1 Mini Lite"
#endif

#elif defined(ESP32)
#include <WiFi.h>
#include <esp_sleep.h>

inline void tcSetHostname(const char* hostname)
{
    WiFi.setHostname(hostname);
}

inline void tcStartDeepSleep(uint64_t sleepTime_us)
{
    esp_sleep_enable_timer_wakeup(sleepTime_us);
    esp_deep_sleep_start();
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