# Tank Commander v0.6

ESP8266-based IoT monitoring and control system for a 3-tank rainwater harvesting setup. Monitors water levels via 4–20 mA pressure transmitters, automates Tank 3 filling through a motorized ball valve, and integrates with Home Assistant via MQTT Discovery.

## Hardware

### Components

| Component | Qty | Purpose |
|-----------|-----|---------|
| WeMos D1 Mini Lite (ESP8266) | 1 | Microcontroller |
| Adafruit INA219 breakout | 3 | 4–20 mA current measurement |
| 4–20 mA liquid level transmitter (0–20 kPa) | 3 | Pressure head sensing |
| 3-wire 12V motorized ball valve | 1 | Tank 3 fill control |
| 2-channel opto-isolated relay module | 1 | Valve power and direction |
| 12V power supply | 1 | Valve motor supply |

### Wiring / Pin Assignment

| Pin | GPIO | Function | Notes |
|-----|------|----------|-------|
| D1 | 5 | I2C SCL | Shared bus for 3× INA219 |
| D2 | 4 | I2C SDA | Shared bus for 3× INA219 |
| D5 | 14 | Keep Awake | Pull LOW to prevent deep sleep |
| D6 | 12 | Valve Power Relay | Active-LOW — energises 12V supply |
| D7 | 13 | Valve Direction Relay | Active-LOW — LOW = open, HIGH = close |
| A0 | ADC0 | Battery Monitor | Currently disabled (`USE_BAT=0`) |

### INA219 I2C Addresses

| Sensor | Address | Jumper Config |
|--------|---------|---------------|
| Tank 1 | 0x40 | A0 = GND, A1 = GND |
| Tank 2 | 0x41 | A0 = Vcc, A1 = GND |
| Tank 3 | 0x44 | A0 = GND, A1 = Vcc |

### Tank Dimensions (Default)

All three tanks: **1820 mm diameter × 1920 mm height** (cylindrical).

## Software Architecture

```
src/
├── main.h                # Global defines, WiFi/MQTT config, pin assignments
├── main.cpp              # Application entry: setup, loop, MQTT handlers, HA Discovery
├── tank.h / tank.cpp     # Tank class — INA219 driver, level calculation, sensor health
├── fill_controller.h/.cpp # FillController — state machine, valve control, EEPROM config
└── secrets.h             # Credentials (gitignored — see Setup section)
```

### Core Flow

1. **Boot** — Relays driven HIGH (safe) before `pinMode`; serial, I2C, and tanks initialised.
2. **Connect** — WiFi → mDNS (`TankCommander`) → OTA handler → MQTT broker.
3. **HA Discovery** — Publishes auto-discovery configs for all entities.
4. **Loop** — OTA handle → MQTT loop → decode commands → read sensors (every *interval* seconds) → publish state → run fill controller → sleep check.
5. **Sleep** — Deep sleep (`WAKE_NO_RFCAL`) when no fill activity and keep-awake is off. Relays de-energised and sensors put in power-save mode before sleeping.

### Fill Controller State Machine

```
DISABLED ──► IDLE ──► VALVE_OPENING ──► FILLING ──► VALVE_CLOSING ──► COOLDOWN ──► IDLE
                          │                              ▲
                          └── FAULT_SENSOR / FAULT_TIMEOUT
```

**Valve actuation is non-blocking** (millis-based): 50 ms relay settle → 8.5 s valve travel.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `fillEnabled` | `false` | Master enable/disable |
| `targetLevel_pc` | 80% | Stop filling at this level |
| `lowThreshold_pc` | 60% | Auto-fill trigger |
| `maxDuration_min` | 30 min | Abort fill safety timeout |
| Hard max | 60 min | Absolute non-configurable limit |
| Cooldown | 30 s | Prevents rapid valve cycling |

Configuration is persisted to EEPROM (magic `0xFC02`, version 2).

### Sensor Health Monitoring

The `Tank` class continuously checks each 4–20 mA loop:

| Condition | Threshold | Result |
|-----------|-----------|--------|
| Wire break | < 3.5 mA | `FAULT_SENSOR` |
| Short circuit | > 21.0 mA | `FAULT_SENSOR` |
| Stuck reading | ±0.01 mA for 5 consecutive reads | `FAULT_SENSOR` |

### Calibration

Each sensor converts milliamps to pressure head using a linear equation:

```
head_mm = mA × m + b
```

**Default calibration:** `m = 128.5657757`, `b = -516.7934059`

Calibration can be updated at runtime via MQTT (see below). Changes are **not persisted** across reboots.

## Setup

### 1. Create `src/secrets.h`

This file is gitignored. Create it manually:

```cpp
#pragma once

#define WIFI_SSID     "YourSSID"
#define WIFI_PASS     "YourPassword"

#define MQTT_SERVER   "your-broker.example.com"
#define MQTT_PORT     1883

#define OTA_PASS      "YourOTAPassword"
```

### 2. Install PlatformIO

Install [PlatformIO IDE](https://platformio.org/install/ide) (VS Code extension) or the CLI.

### 3. Build & Upload

**Serial upload (first time):**

```bash
pio run -t upload --upload-port COMx    # Windows
pio run -t upload --upload-port /dev/ttyUSBx  # Linux
```

**OTA upload (subsequent):**

```bash
# Set the OTA password as an environment variable first
export OTA_PASS="YourOTAPassword"
pio run -t upload
```

OTA is preconfigured in `platformio.ini` to target `10.0.0.68`. Change `upload_port` if your device has a different IP.

### 4. Serial Monitor

```bash
pio device monitor -b 115200
```

## MQTT Interface

### Broker Connection

- **Host:** Defined in `secrets.h` (`MQTT_SERVER`)
- **Port:** 1883
- **Client ID:** `TankCommander`
- **Keep-alive:** Matches the sensor read interval (default 60 s)

### Published Topics

| Topic | Payload | Retained | Description |
|-------|---------|----------|-------------|
| `tanks` | JSON `{interval, keep_awake}` | No | Device status |
| `tanks/tank1` | JSON `{mm, pc, L, bus_V}` | No | Tank 1 readings |
| `tanks/tank2` | JSON `{mm, pc, L, bus_V}` | No | Tank 2 readings |
| `tanks/tank3` | JSON `{mm, pc, L, bus_V}` | No | Tank 3 readings |
| `tanks/fill/state` | JSON (see below) | Yes | Fill controller state |

**Fill state payload:**
```json
{
  "state": "idle",
  "filling": false,
  "enabled": false,
  "target_pc": 80.0,
  "low_threshold_pc": 60.0,
  "max_duration_min": 30,
  "tank3_pc": 75.5,
  "tank3_L": 12340.0,
  "sensor_ok": true,
  "fault": "none",
  "fill_elapsed_min": 0,
  "fill_reason": "none",
  "valve": "closed"
}
```

### Subscribed Topics (Commands)

| Topic | Payload | Description |
|-------|---------|-------------|
| `tanks/commands/interval` | Seconds (integer) | Sensor read interval |
| `tanks/commands/keepawake` | `True` / `False` | Software keep-awake |
| `tanks/commands/cal1` | `"m,b"` | Tank 1 calibration |
| `tanks/commands/cal2` | `"m,b"` | Tank 2 calibration |
| `tanks/commands/cal3` | `"m,b"` | Tank 3 calibration |
| `tanks/fill/config/enabled` | `true` / `false` | Enable/disable fill system |
| `tanks/fill/config/target` | `10`–`100` | Fill target percentage |
| `tanks/fill/config/low_threshold` | `5`–`95` | Auto-fill trigger percentage |
| `tanks/fill/config/max_duration` | `1`–`60` | Max fill duration (minutes) |
| `tanks/fill/command` | `start` / `stop` | Manual fill start/stop |
| `tanks/fill/command/override` | `clear_fault` | Clear fault state |

### Calibration Example

```bash
mosquitto_pub -h your-broker.example.com -t "tanks/commands/cal1" -m "128.5657757,-516.7934059"
```

## Home Assistant Integration

Tank Commander publishes MQTT Discovery messages automatically on boot. No manual HA configuration is needed — entities appear under the **Tank Commander** device.

### Auto-Discovered Entities

| Entity | Type | Description |
|--------|------|-------------|
| Fill Enable | Switch | Master fill on/off |
| Fill Command | Switch | Manual start/stop |
| Fill Target Level | Number | Target % (10–100) |
| Fill Low Threshold | Number | Auto-fill trigger % (5–95) |
| Fill Max Duration | Number | Timeout in minutes (1–60) |
| Fill State | Sensor | Current state machine state |
| Sensor Health | Binary Sensor | Tank 3 sensor OK/fault |
| Valve State | Sensor | Valve position/activity |
| Clear Fault | Button | Reset fault state |
| Tank 1/2/3 Level (mm) | Sensor | Water height in mm |
| Tank 1/2/3 Level (%) | Sensor | Water height percentage |
| Tank 1/2/3 Volume (L) | Sensor | Calculated volume |

### Example Automation

```yaml
automation:
  - alias: "Alert when Tank 3 sensor fails"
    trigger:
      - platform: state
        entity_id: binary_sensor.tankcommander_sensor_health
        to: "on"
    action:
      - service: notify.mobile_app
        data:
          title: "Tank Commander"
          message: "Tank 3 sensor fault detected!"
```

## Dependencies

Managed by PlatformIO (`platformio.ini`):

| Library | Purpose |
|---------|---------|
| [Adafruit INA219](https://github.com/mrweaver/Adafruit_INA219) (fork) | 4–20 mA current measurement |
| Adafruit BusIO | I2C abstraction |
| ArduinoJson ~6.16.0 | JSON serialisation |
| PubSubClient | MQTT client |
| RunningAverage | Sensor smoothing |
| ArduinoOTA (built-in) | Over-the-air updates |

## Safety Notes

- **Relays are driven HIGH (de-energised) at the very start of `setup()`** before `pinMode` is called — the valve cannot actuate during boot.
- **Both relays are de-energised before deep sleep** as a defence-in-depth measure.
- **Hard maximum fill duration of 60 minutes** cannot be overridden via MQTT.
- **Sensor fault detection** automatically halts any active fill.
- **Non-blocking valve control** ensures the main loop continues running (OTA, MQTT, watchdog) while the valve is moving.

## License

Private project — not licensed for redistribution.
