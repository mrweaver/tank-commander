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
///   D6 (GPIO12) = IN1 (Power Relay) — controls 12V supply to valve motor
///   D7 (GPIO13) = IN2 (Direction Relay) — de-energised=Close, energised=Open
/// Valve travel time is ~8 seconds. Non-blocking actuation via millis() timers.
/// Fill operations are governed by a local safety state machine that operates
/// independently of Home Assistant connectivity.
///
///////////////////////////////////////////////////////////////////////////////////////////////////

#define VER_MAJOR 0
#define VER_MINOR 6
#define VER_PATCH 0
#define VER_DATE "2026-02-20"

#define USE_BAT 0   // use battery monitor
#define USE_BOOST 0 // use boost converter management

#include <Arduino.h>

// Sensor
#include <RunningAverage.h>

// MQTT -> with JSON
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Tanks
#include "tank.h"
#include <vector> // for tank array

// Fill Controller
#include "fill_controller.h"

// Credentials (gitignored)
#include "secrets.h"

// OTA Updates
#include <ArduinoOTA.h>

// TIMING /////////////////////////////////////////////////////////////////////////////////
// Timing
#define TIME_MS_IN_Y 31557600000              // milliseconds in year
#define TIME_MS_IN_W 604800000                // milliseconds in week
#define TIME_MS_IN_D 86400000                 // milliseconds in day
#define TIME_MS_IN_H 3600000                  // milliseconds in hour
#define TIME_MS_IN_M 60000                    // milliseconds in minute
#define TIME_MS_IN_S 1000                     // milliseconds in second
uint64_t tTxInterval_ms = 60000;              // [ms]
uint64_t tsTransmit_ms = tTxInterval_ms;      // [ms] timestamp of previous transmission
uint64_t sleepTime_us = tTxInterval_ms * 1e3; // [us] Measurement and transmission interval
uint16_t bootTime_ms = 250;                   // [ms] boot up time for combined boost converter and sensors
void setTxInterval(uint64_t s);
String msToString(uint64_t ms);

// WIFI ///////////////////////////////////////////////////////////////////////
const char wlan_ssid[] = WIFI_SSID;              // wifi SSID
const char wlan_pass[] = WIFI_PASS;              // WiFi pass
const char wlan_hostname[] = "TankCommander";    // WiFi device hostname
WiFiClient wifiClient;                          // init wifi class
bool wifiConnect();
void OTAinit();
extern volatile bool otaInProgress;

// JSON ///////////////////////////////////////////////////////////////////////
const size_t capacityOut = JSON_OBJECT_SIZE(12);
StaticJsonDocument<capacityOut> docOut;       // use DynamicJsonDocument if doc is >1KB
const char *jsonTank_mm = "mm";               // [mm] tank head
const char *jsonTank_pc = "pc";               // [%] tank percent full
const char *jsonTank_L = "L";                 // [L] tank volume
const char *jsonTank_bus = "bus_V";           // [V] tank bus voltage
const char *jsonBat_V = "bat_V";              // [V]  Battery voltage
const char *jsonIntrvl = "interval";          // [hh:mm:ss]  Transmission interval
const char *jsonKeepAwakeSW = "keepawake_sw"; // Software forced "keep awake"
const char *jsonKeepAwakeHW = "keepawake_hw"; // Hardware forced "keep awake"
char jsonBuffer[384];                         // JSON buffer for output to MQTT

// MQTT ///////////////////////////////////////////////////////////////////////
const char *mqttTopicPub = "tanks";                                                    // MQTT publish topic
const char *mqttTopicSubInterval = "tanks/commands/interval";                          // MQTT subscription topic
const char *mqttTopicSubKeepAwake = "tanks/commands/keepawake";                        // MQTT subscription topic
const char *mqttTopicSubCal1 = "tanks/commands/cal1";                                  // MQTT subscription topic
const char *mqttTopicSubCal2 = "tanks/commands/cal2";                                  // MQTT subscription topic
const char *mqttTopicSubCal3 = "tanks/commands/cal3";                                  // MQTT subscription topic
bool retainedInterval = false;                                                         // MQTT was interval retained successfully?
bool retainedKeepAwake = false;                                                        // MQTT was keep awake retained successfully?
char homeServer_ip[] = MQTT_SERVER;                                                    // MQTT broker address
uint16_t homeServer_port = MQTT_PORT;                                                  // MQTT port
const uint8_t mqttMaxRetry = 5;                                                        // MQTT maximum retry
uint16_t mqttKeepAlive_s = (tTxInterval_ms / 1000);                                    // [s]
ICACHE_RAM_ATTR void mqttCallback(char *topic, uint8_t *payload, unsigned int length); // MQTT callback
char *mqttTopic;
uint8_t *mqttPayload;
unsigned int mqttBytesAvailable = 0;
PubSubClient mqttClient(homeServer_ip, homeServer_port, mqttCallback, wifiClient); // init MQTT class
bool mqttConnect();
bool mqttPublishStates();
void mqttDecode();
void mqttPublishFillState();
void mqttPublishHADiscovery();

// MQTT Fill Topics
const char* mqttTopicFillState       = "tanks/fill/state";
const char* mqttTopicFillCommand     = "tanks/fill/command";
const char* mqttTopicFillOverride    = "tanks/fill/command/override";
const char* mqttTopicFillCfgEnabled  = "tanks/fill/config/enabled";
const char* mqttTopicFillCfgTarget   = "tanks/fill/config/target";
const char* mqttTopicFillCfgLow      = "tanks/fill/config/low_threshold";
const char* mqttTopicFillCfgMaxDur   = "tanks/fill/config/max_duration";

// SENSOR AND CALIBRATION /////////////////////////////////////////////////////

#define BATT_BIT_TO_V (4.4 / 1023) // (100k) /( 120k + 220k + 100k) => 1/4.4 ; 1023 bit ADC.
#define SENSOR_MAX_kPa 20
#define SENSOR_MIN_kPa 0
#define SENSOR_MAX_mA 20
#define SENSOR_MIN_mA 4
#define SENSOR_mA_TO_kPa_M ((SENSOR_MAX_kPa - SENSOR_MIN_kPa) / (SENSOR_MAX_mA - SENSOR_MIN_mA)) // (20)kPa / (20-4)mA
#define SENSOR_mA_TO_kPa_B (-SENSOR_mA_TO_kPa_M * SENSOR_MIN_mA)                                 // offset
#define DENSITY_H2O (1000)                                                                       // [kg/m^3]
#define ACC_GRAVITY (9.80665)                                                                    // [m/s^-2]
#define MM_PER_KPA (1000 * 1000 / (DENSITY_H2O * ACC_GRAVITY))                                   // [mm/kPa]

float bat_V = 0; // Battery voltage
// void sensorInit();
void readSensors();

// TANKS //////////////////////////////////////////////////////////////////////

// Addresses
#define TANK1_ADDR (0x40) // 1000000 (A0 = GND, A1 = GND, no bridges)
#define TANK2_ADDR (0x41) // 1000001 (A0 = Vs+, A1 = GND: bridge A0)
#define TANK3_ADDR (0x44) // 1000100 (A0 = GND, A1 = Vs+: bridge A1)

// Diameter
#define DF_TANK_D_MM (1820)
#define TANK1_D_MM (DF_TANK_D_MM)
#define TANK2_D_MM (DF_TANK_D_MM)
#define TANK3_D_MM (DF_TANK_D_MM)

// Height
// #define DF_TANK_H_MM (2190)
#define DF_TANK_H_MM (1920) // TODO: Measure each tank: sensor to outlet
#define TANK1_H_MM (DF_TANK_H_MM)
#define TANK2_H_MM (DF_TANK_H_MM)
#define TANK3_H_MM (DF_TANK_H_MM)

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
