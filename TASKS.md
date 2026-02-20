# Tank Commander — Task Tracker

## Pending

- [ ] Verify calibration — send `"m,b"` via MQTT to `tanks/commands/cal1` and confirm readings match expected values
- [ ] Fix regex in Home Assistant — may already be resolved by HA MQTT Discovery; needs verification

## Uncertain / Needs Retest

- [ ] Add "current cal" display and clear input_texts when sending — likely obsolete with HA Discovery; confirm
- [ ] Fix oscillating interval time issue — may be resolved; retest

## Completed

- [x] Create project README with instructions and documentation *(2026-02-20)*
- [x] Auto-adjust tank max height from recorded sensor maximum *(2026-02-20)*
- [x] MQTT calibration parsing — `m,b` format via `cal1`/`cal2`/`cal3` topics *(2026-02-20)*
- [x] Extract secrets to gitignored `secrets.h` *(2026-02-20)*
- [x] Check OTA — fully implemented with sleep protection *(2026-02-20)*
- [x] MQTT input — calibration *(2026-02-20)*
- [x] Use `sscanf` to parse calibration `"m,b"` string — replaces TinyExpr/RegEx approach *(2026-02-20)*
- [x] MQTT input — stay awake (retained message) *(2020-08-16)*
- [x] MQTT input — interval time *(2020-08-13)*
- [x] Get MQTT to read retained messages from broker *(2020-08-13)*
- [x] OTA with sleep *(2020-08-12)*
- [x] Power-save INA219 on sleep *(2020-08-12)*
- [x] Additional INA219 devices *(2020-08-12)*
- [x] Resistor change *(2020-08-12)*
- [x] Improve power saving *(2020-08-12)*
- [x] Modify Adafruit_INA219 *(2020-08-12)*
- [x] Create Tank class *(2020-08-12)*
- [x] MQTT core *(2020-08-12)*
- [x] Height calibration *(2020-08-08)*
