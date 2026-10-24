///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// Tank Commander
///
/// Engineer: Michael Weaver
/// Version History:
/// 20-08-05 - v0.1 - Initial Commit
/// 25-02-20 - v0.3 - Add relay-controlled fill solenoid for Tank 3
/// 26-02-20 - v0.4 - Replace solenoid with 3-wire motorised ball valve (2-channel relay)
/// 26-02-20 - v0.5 - Remove MQTT auth, disable OTA defaults, add project documentation
/// 26-02-20 - v0.6 - Improve Home Assistant integration (bouncing switch, enums, keep awake, interval)
/// 26-02-21 - v0.7 - Sensor reliability fixes, WiFi resilience, valve mode selector (Closed/Auto/Open)
/// 26-03-24 - v0.7.1 - Fill diagnostic logging, 95% safety overflow cap, stale MQTT cleanup, calibration fix
/// 26-04-26 - v0.7.2 - Fix hang: remove blocking wifiConnect() from MQTT retry path; always advance tsTransmit_ms; time-based FAULT_SENSOR recovery; add read_uptime_s to tank payloads
/// 26-04-26 - v0.7.3 - Sensor health reporting for all tanks: connected/sensor_ok/sensor_fault in payloads; HA discovery for health+connected+read_uptime per tank; sensorOk() guards disconnected INA219; fill controller FAULT_SENSOR on disconnected tank; fill state buffer 256->384; MQTT buffer 768->1024
/// 26-05-07 - v0.7.4 - Active INA219 connectivity probe via R/W integrity test on register 0x05 (verifies ACK + register memory + bus paths); always attempt read() so disconnected sensors can recover; replace read_uptime_s with last_read_age_s; fill controller requires 3 consecutive bad reads before FAULT_SENSOR; fix latent POWERDOWN-after-construct bug
/// 26-05-07 - v0.7.5 - Diagnostic surface: I2C bus scanner sensor + on-demand rescan button; per-tank mA/bus_V/sensor_fault/init_ok/fail_streak entities; fill fault/elapsed/reason entities; keepawake_hw binary sensor; last_read_age_s renders "never" instead of 4.3 billion seconds when no successful read this wake; library isAlive() method delegates probe to Adafruit_INA219
/// 26-05-14 - v0.8.0 - First multi-platform controller release: ESP32 platform support via compatibility layer/build targets, richer tank and fill diagnostics, immediate MQTT command acknowledgements, retained availability for HA entities, controller-first always-online default build, README/docs aligned with the new control model
/// 26-10-03 - v0.8.1 - Command latency: MQTT reconnect from loop() every 5s (was only at tx interval); fixed 15s keep-alive (interval changes no longer desync broker keep-alive); 2s socket timeout; retained-message wait only in deep-sleep builds and now decodes messages; Valve Mode "open" during valve close now reverses immediately instead of being lost
/// 26-10-03 - v0.9.0 - Fill safety: valve driven closed on every boot; FAULT_TIMEOUT latched in EEPROM across reboots; fill progress (stall) detection -> no_progress fault; fault classification (fill_timeout / no_progress / overflow_cap); target clamped below 95% overflow cap; OPEN override not resumed after reboot; fill state published on every state change; fill rate + reset reason diagnostics; ESP8266 build targets retired
/// 26-10-03 - v0.10.0 - Trickle fill: configurable valve opening (10-100% of travel time) via time-based position estimate re-referenced at end stops, applied live mid-fill; hard max fill 60 -> 480 min, default max duration 240 min
/// 26-10-24 - v0.11.0 - Post-close leak check: after every close, a 1.5% rise in 5 min, or climbing past 97% (overflowing), within 30 min means the valve did not seal; cycle it fully open and closed up to 5 times, then latch valve_stuck_open fault and retry every 15 min while still leaking; new fill_reason valve_reclose
/// 26-10-24 - v0.12.0 - Leak check ignores sun/cloud swings in the Tank 3 reading (~3% peak to peak on sunny middays): reads Tank 3 every 10 s while watching and compares 5 min averages with the first reading after the close; leaking = 3% above it (replaces the 1.5%/5 min and 97% rules)
/// 26-10-24 - v0.13.0 - Leak check rise, averaging window and watch time adjustable over MQTT (tanks/fill/config/leak_rise|leak_window|leak_watch) and from HA number entities; saved in the fill config (padding bytes, 0 = defaults 3% / 5 min / 30 min)
///
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// SYSTEM DESCRIPTION
///
/// Three rainwater tanks are cascaded: rainwater enters Tank 1, overflows to Tank 2,
/// then to Tank 3. All outlets are connected with ball valves and can be individually
/// selected. A pump delivers water through a filter to the house for drinking water,
/// or bypasses the filter for yard use. Future plan: a second pump on Tank 3 to
/// isolate house/yard water operations.
///
/// Tank 3 has a 3-wire 12V motorised ball valve for filling with mains water,
/// controlled by a 2-channel opto-isolated relay module (active-LOW):
///   ESP8266 env: D6 (GPIO12) = IN1, D7 (GPIO13) = IN2
///   Other envs: relay pins are supplied by board-specific PlatformIO build flags
/// Valve travel time is ~8 seconds. Non-blocking actuation via millis() timers.
/// Fill operations are governed by a local safety state machine that operates
/// independently of Home Assistant connectivity.
///
///////////////////////////////////////////////////////////////////////////////////////////////////

#define VER_MAJOR 0
#define VER_MINOR 13
#define VER_PATCH 0
#define VER_DATE  "2026-10-24"

#define USE_BAT   0  // use battery monitor
#define USE_BOOST 0  // use boost converter management

#include <Arduino.h>

// Sensor
#include <RunningAverage.h>

// MQTT -> with JSON
#include <ArduinoJson.h>
#include "platform_compat.h"
#include <PubSubClient.h>

// Tanks
#include "tank.h"
#include <vector>  // for tank array

// Fill Controller
#include "fill_controller.h"

// Credentials (gitignored)
#include "secrets.h"

// OTA Updates
#include <ArduinoOTA.h>

// TIMING /////////////////////////////////////////////////////////////////////////////////
// Timing
#define TIME_MS_IN_Y          31557600000         // milliseconds in year
#define TIME_MS_IN_W          604800000           // milliseconds in week
#define TIME_MS_IN_D          86400000            // milliseconds in day
#define TIME_MS_IN_H          3600000             // milliseconds in hour
#define TIME_MS_IN_M          60000               // milliseconds in minute
#define TIME_MS_IN_S          1000                // milliseconds in second
#define FILL_READ_INTERVAL_MS 10000               // [ms] fast sensor reads during fill
uint32_t lastFillRead_ms = 0;                     // timestamp of last fast fill read
uint64_t tTxInterval_ms  = 60000;                 // [ms]
uint64_t tsTransmit_ms   = tTxInterval_ms;        // [ms] timestamp of previous transmission
uint64_t sleepTime_us    = tTxInterval_ms * 1e3;  // [us] Measurement and transmission interval
uint16_t bootTime_ms     = 250;                   // [ms] boot up time for combined boost converter and sensors
#ifndef TC_ALLOW_DEEP_SLEEP
#define TC_ALLOW_DEEP_SLEEP 0
#endif
void setTxInterval(uint64_t s);
String msToString(uint64_t ms);

// WIFI ///////////////////////////////////////////////////////////////////////
const char wlan_ssid[]     = WIFI_SSID;        // wifi SSID
const char wlan_pass[]     = WIFI_PASS;        // WiFi pass
const char wlan_hostname[] = "TankCommander";  // WiFi device hostname
WiFiClient wifiClient;                         // init wifi class
bool wifiConnect();
void OTAinit();
uint32_t lastWifiReconnectAttempt_ms = 0;
extern volatile bool otaInProgress;
extern bool otaReady;

// JSON ///////////////////////////////////////////////////////////////////////
const size_t capacityOut = JSON_OBJECT_SIZE(12);
StaticJsonDocument<capacityOut> docOut;        // use DynamicJsonDocument if doc is >1KB
const char* jsonTank_mm     = "mm";            // [mm] tank head
const char* jsonTank_pc     = "pc";            // [%] tank percent full
const char* jsonTank_L      = "L";             // [L] tank volume
const char* jsonTank_bus    = "bus_V";         // [V] tank bus voltage
const char* jsonBat_V       = "bat_V";         // [V]  Battery voltage
const char* jsonIntrvl      = "interval";      // [hh:mm:ss]  Transmission interval
const char* jsonKeepAwakeSW = "keepawake_sw";  // Software forced "keep awake"
const char* jsonKeepAwakeHW = "keepawake_hw";  // Hardware forced "keep awake"
char jsonBuffer[384];                          // JSON buffer for output to MQTT

// MQTT ///////////////////////////////////////////////////////////////////////
const char* mqttTopicPub          = "tanks";                            // MQTT publish topic
const char* mqttTopicSubInterval  = "tanks/commands/interval";          // MQTT subscription topic
const char* mqttTopicSubKeepAwake = "tanks/commands/keepawake";         // MQTT subscription topic
const char* mqttTopicSubCal1      = "tanks/commands/cal1";              // MQTT subscription topic
const char* mqttTopicSubCal2      = "tanks/commands/cal2";              // MQTT subscription topic
const char* mqttTopicSubCal3      = "tanks/commands/cal3";              // MQTT subscription topic
const char* mqttTopicAvailability = "tanks/status";                     // MQTT availability topic
bool retainedInterval             = false;                              // MQTT was interval retained successfully?
bool retainedKeepAwake            = false;                              // MQTT was keep awake retained successfully?
char homeServer_ip[]              = MQTT_SERVER;                        // MQTT broker address
uint16_t homeServer_port          = MQTT_PORT;                          // MQTT port
const uint8_t mqttMaxRetry        = 5;                                  // MQTT maximum retry
const uint16_t mqttKeepAlive_s    = 15;                                 // [s] fixed — independent of tx interval
const uint16_t mqttSocketTimeout_s = 2;                                 // [s] bounds blocking in connect()
const uint32_t mqttReconnectInterval_ms = 5000;                         // [ms] reconnect attempt cadence
uint32_t lastMqttReconnectAttempt_ms    = 0;
void mqttCallback(char* topic, uint8_t* payload, unsigned int length);  // MQTT callback
char* mqttTopic;
uint8_t* mqttPayload;
unsigned int mqttBytesAvailable = 0;
PubSubClient mqttClient(homeServer_ip, homeServer_port, mqttCallback, wifiClient);  // init MQTT class
bool mqttConnect();
bool mqttPublishStates();
bool mqttPublishDeviceState();
bool mqttPublishAvailability(bool online);
void mqttDecode();
bool mqttPublishFillState();
void mqttPublishHADiscovery();
String i2cScan();
void publishI2CScan();

// MQTT Diagnostic Topics
const char* mqttTopicDiagI2C       = "tanks/diag/i2c_scan";
const char* mqttTopicDiagRescanCmd = "tanks/diag/i2c_scan/rescan";

// MQTT Fill Topics
const char* mqttTopicFillState      = "tanks/fill/state";
const char* mqttTopicFillCommand    = "tanks/fill/command";
const char* mqttTopicFillOverride   = "tanks/fill/command/override";
const char* mqttTopicFillCfgEnabled = "tanks/fill/config/enabled";
const char* mqttTopicFillCfgTarget  = "tanks/fill/config/target";
const char* mqttTopicFillCfgLow     = "tanks/fill/config/low_threshold";
const char* mqttTopicFillCfgMaxDur  = "tanks/fill/config/max_duration";
const char* mqttTopicFillCfgMode    = "tanks/fill/config/mode";
const char* mqttTopicFillCfgOpening = "tanks/fill/config/opening";
const char* mqttTopicFillCfgLeakRise   = "tanks/fill/config/leak_rise";
const char* mqttTopicFillCfgLeakWindow = "tanks/fill/config/leak_window";
const char* mqttTopicFillCfgLeakWatch  = "tanks/fill/config/leak_watch";

// SENSOR AND CALIBRATION /////////////////////////////////////////////////////

#define BATT_BIT_TO_V      (4.4 / 1023)  // (100k) /( 120k + 220k + 100k) => 1/4.4 ; 1023 bit ADC.
#define SENSOR_MAX_kPa     20
#define SENSOR_MIN_kPa     0
#define SENSOR_MAX_mA      20
#define SENSOR_MIN_mA      4
#define SENSOR_mA_TO_kPa_M ((SENSOR_MAX_kPa - SENSOR_MIN_kPa) / (SENSOR_MAX_mA - SENSOR_MIN_mA))  // (20)kPa / (20-4)mA
#define SENSOR_mA_TO_kPa_B (-SENSOR_mA_TO_kPa_M * SENSOR_MIN_mA)                                  // offset
#define DENSITY_H2O        (1000)                                                                 // [kg/m^3]
#define ACC_GRAVITY        (9.80665)                                                              // [m/s^-2]
#define MM_PER_KPA         (1000 * 1000 / (DENSITY_H2O * ACC_GRAVITY))                            // [mm/kPa]

float bat_V = 0;  // Battery voltage
// void sensorInit();
void readSensors();

// TANKS //////////////////////////////////////////////////////////////////////

// Addresses
#define TANK1_ADDR (0x40)  // 1000000 (A0 = GND, A1 = GND, no bridges)
#define TANK2_ADDR (0x41)  // 1000001 (A0 = Vs+, A1 = GND: bridge A0)
#define TANK3_ADDR (0x44)  // 1000100 (A0 = GND, A1 = Vs+: bridge A1)

// Diameter
#define DF_TANK_D_MM (1820)
#define TANK1_D_MM   (DF_TANK_D_MM)
#define TANK2_D_MM   (DF_TANK_D_MM)
#define TANK3_D_MM   (DF_TANK_D_MM)

// Height
// #define DF_TANK_H_MM (2190)
#define DF_TANK_H_MM (1920)  // TODO: Measure each tank: sensor to outlet
#define TANK1_H_MM   (DF_TANK_H_MM)
#define TANK2_H_MM   (DF_TANK_H_MM)
#define TANK3_H_MM   (DF_TANK_H_MM)

std::vector<Tank> tanks;
void tankInit();
FillController* fillController = nullptr;

// POWER SAVING //////////////////////////////////////////////////////////////////
void sleep();
void sleepInit();
bool keepAwake_sw = false;

// OTHER FUNCTIONS //////////////////////////////////////////////////////////////////

void setup();
void loop();
