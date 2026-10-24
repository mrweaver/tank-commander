#include "main.h"
#include <Wire.h>

static_assert(FILL_TARGET_MAX_PC == 92.0f, "update fill_target discovery max to match FILL_TARGET_MAX_PC");
static_assert(FILL_HARD_MAX_MIN == 480, "update fill_max_duration discovery max to match FILL_HARD_MAX_MIN");
static_assert(FILL_CONFIG_ADDR + sizeof(FillConfig) <= CAL_CONFIG_ADDR &&
                  CAL_CONFIG_ADDR + sizeof(CalConfig) <= FILL_EEPROM_SIZE,
              "FillConfig and CalConfig must not overlap and must fit in the EEPROM area");
static_assert(LEAK_WINDOW_MIN_MIN == 1 && LEAK_WINDOW_MAX_MIN == 15 && LEAK_WATCH_MIN_MIN == 10 && LEAK_WATCH_MAX_MIN == 120,
              "update leak_window / leak_watch discovery min/max to match");

void setup(void)
{
    Wire.begin(P_SDA, P_SCL);

    // Boot-safe relay init — MUST be first, before anything else.
    // Both pins written HIGH (de-energised) BEFORE setting pinMode to OUTPUT,
    // preventing the relays from briefly firing during MCU boot.
    digitalWrite(P_VALVE_PWR, HIGH);  // Power relay OFF
    digitalWrite(P_VALVE_DIR, HIGH);  // Direction relay to Close (de-energised)
    pinMode(P_VALVE_PWR, OUTPUT);
    pinMode(P_VALVE_DIR, OUTPUT);

    Serial.begin(115200);
    // Serial.setTimeout(2000);

    // delay(100);

    Serial.println();
    Serial.println();
    Serial.println("|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||");
    Serial.println("||                                                       ||");
    Serial.println("||                     TANK COMMANDER                    ||");
    Serial.printf("||                         v%u.%u.%u                        ||\n", VER_MAJOR, VER_MINOR, VER_PATCH);
    Serial.printf("||                       %s                      ||\n", VER_DATE);
    Serial.println("||                                                       ||");
    Serial.println("|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||");
    Serial.println();
    Serial.println();

    tankInit();

    // Init fill controller for Tank 3 (index 2)
    // P_VALVE_PWR / P_VALVE_DIR are board-specific relay pins from build flags.
    fillController = new FillController(P_VALVE_PWR, P_VALVE_DIR, &tanks[2]);
    fillController->begin();  // also opens the EEPROM area

    // Restore saved tank calibrations (HA's retained cal topics still apply on top)
    CalConfig cal;
    if (calLoad(cal))
    {
        for (uint8_t i = 0; i < CAL_TANKS && i < tanks.size(); i++)
        {
            if (!(cal.setMask & (1 << i)))
                continue;
            tanks[i].setCali_mA_to_mm(cal.m[i], cal.b[i]);
            Serial.printf("Tank %u calibration restored: m=%0.4f, b=%0.4f\n", i + 1, cal.m[i], cal.b[i]);
        }
    }

    if (wifiConnect())
    {
        OTAinit();
        mqttConnect();
        mqttPublishHADiscovery();
        publishI2CScan();  // boot-time bus scan published to tanks/diag/i2c_scan
    }
    sleepInit();  // does not trigger sleep - prepares buttons etc.
}

void loop(void)
{
    // WiFi reconnection (non-blocking, attempt every 30s)
    if (WiFi.status() != WL_CONNECTED)
    {
        otaReady = false;
        if ((millis() - lastWifiReconnectAttempt_ms) > 30000)
        {
            lastWifiReconnectAttempt_ms = millis();
            Serial.println("WiFi disconnected, attempting reconnect...");
            WiFi.reconnect();
        }
    }
    else if (!otaReady)
    {
        OTAinit();
    }

    // MQTT reconnection — retried every few seconds so commands are not lost
    // for a whole tx interval after a broker restart or WiFi blip.
    if (WiFi.status() == WL_CONNECTED && !mqttClient.connected() && !otaInProgress &&
        (millis() - lastMqttReconnectAttempt_ms) > mqttReconnectInterval_ms)
    {
        lastMqttReconnectAttempt_ms = millis();
        Serial.println("MQTT disconnected — attempting reconnect.");
        mqttConnect();
    }

    // These polling functions only work when sleep switch is deactivated.
    ArduinoOTA.handle();
    mqttClient.loop();

    // mqtt Receive
    if (mqttBytesAvailable)  // triggered through ISR
        mqttDecode();        // if enough time has elapsed

    // Serial.printf("millis(): %u, tsTransmit: %llu, interval: %llu\n", millis(), tsTransmit_ms, tTxInterval_ms);
    if ((millis() - tsTransmit_ms) > tTxInterval_ms)
    {
        tsTransmit_ms = millis();  // always advance — prevents tight retry loop on MQTT failure
        readSensors();
        mqttPublishStates();  // broadcast new states to MQTT broker
    }

    // Fast sensor reads during active fill and the post-close leak check
    // (every 10s instead of 60s; the leak check averages them).
    // Always attempt read() — read() handles the I2C probe and recovery
    // internally; gating on connected() would prevent recovery.
    if ((fillController->isFilling() || fillController->leakWatching()) &&
        (millis() - lastFillRead_ms) > FILL_READ_INTERVAL_MS)
    {
        tanks[2].read();
        lastFillRead_ms = millis();
    }

    fillController->update();

    // Publish fill state as soon as the controller changes state on its own
    // (valve phases, fill start/stop, faults) — not only at the tx interval.
    // stateStr()/valveStateStr() return string literals, so pointer compare is exact.
    static const char* lastFillState = nullptr;
    static const char* lastValveState = nullptr;
    if (mqttClient.connected() &&
        (fillController->stateStr() != lastFillState || fillController->valveStateStr() != lastValveState))
    {
        if (mqttPublishFillState())
        {
            lastFillState  = fillController->stateStr();
            lastValveState = fillController->valveStateStr();
        }
    }

    if (TC_ALLOW_DEEP_SLEEP && !keepAwake_sw && !digitalRead(P_KEEPAWAKE) && !fillController->keepAwake() && !otaInProgress)
        sleep();
}

// Sensor /////////////////////////////////////////////////////////////////////////////

void readSensors()
{
    Serial.println();
    Serial.println("-------------------------");

#if USE_BOOST
    // Turn on Boost converter for sensors
    Serial.println("Boost converter : On");
    digitalWrite(P_BOOST, true);
    delay(bootTime_ms);  // wait for powerup
#endif

    // float shuntvoltage = 0;
    // float busvoltage = 0;
    // float current_mA = 0;
    // float loadvoltage = 0;
    // float power_mW = 0;
    // float head_mm = 0;

    // shuntvoltage = ina219.getShuntVoltage_mV();
    // busvoltage = ina219.getBusVoltage_V();
    // power_mW = ina219.getPower_mW();
    // loadvoltage = busvoltage + (shuntvoltage / 1000);
    for (size_t t = 0; t < tanks.size(); t++)
    {
        // Always attempt read(); read() probes the bus and tracks failures
        // internally. Skipping based on connected() would prevent recovery
        // from a transient I2C miss.
        Serial.printf("Tank %u:\n", t + 1);
        tanks[t].read();

        if (!tanks[t].connected())
        {
            Serial.printf("\t(no I2C ACK — skipping log)\n");
            continue;
        }

        Serial.printf("\tBus Voltage   : %0.3f V\n", tanks[t].busV());
        Serial.printf("\tCurrent       : %0.3f mA\n", tanks[t].mA());
        Serial.printf("\tPressure Head : %0.3f mm\n", tanks[t].mm());
        Serial.printf("\tVolume        : %0.3f %%\n", tanks[t].pc());
    }

#if USE_BAT
    bat_V = analogRead(P_BATV) * BATT_BIT_TO_V;
    Serial.printf("Battery       : %0.3f V\n", bat_V);
#endif

#if USE_BOOST
    // Shut down boost converter
    Serial.println("Boost converter : Off");
    digitalWrite(P_BOOST, false);
#endif
}

// I2C Scanner ///////////////////////////////////////////////////////////////////////
// Walks I2C addresses 0x03..0x77 and returns a comma-separated list of ACKing
// addresses (e.g. "0x40,0x41,0x44") or "none" if the bus is silent. Useful for
// confirming whether the INA219 chips are actually present on the bus.
String i2cScan()
{
    String result;
    int count = 0;
    for (uint8_t addr = 0x03; addr < 0x78; addr++)
    {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0)
        {
            if (count++ > 0) result += ",";
            char buf[8];
            snprintf(buf, sizeof(buf), "0x%02X", addr);
            result += buf;
        }
    }
    return count == 0 ? String("none") : result;
}

void publishI2CScan()
{
    String scan = i2cScan();
    Serial.printf("I2C bus scan: %s\n", scan.c_str());
    if (mqttClient.connected())
        mqttClient.publish(mqttTopicDiagI2C, scan.c_str(), true);  // retained
}

// Tanks //////////////////////////////////////////////////////////////////////////////
void tankInit()
{
#if USE_BOOST
    pinMode(P_BOOST, OUTPUT);
#endif

    // Create tank instances
    tanks.push_back(Tank(TANK1_ADDR, TANK1_D_MM, TANK1_H_MM));  // Tank 1
    tanks.push_back(Tank(TANK2_ADDR, TANK2_D_MM, TANK2_H_MM));  // Tank 2
    tanks.push_back(Tank(TANK3_ADDR, TANK3_D_MM, TANK3_H_MM));  // Tank 3
}

// WiFi ////////////////////////////////////////////////////////////////////////////////
bool wifiConnect()
{
    Serial.print("Connecting to ");
    Serial.print(wlan_ssid);
    tcSetHostname(wlan_hostname);
    WiFi.begin(wlan_ssid, wlan_pass);

    uint32_t wifiStart = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
        if ((millis() - wifiStart) > 30000)
        {
            Serial.println();
            Serial.println(F("WiFi connection timeout. Continuing offline."));
            return false;
        }
        delay(100);
        Serial.print(F("."));
    }
    Serial.println();
    Serial.println(F("WiFi connected."));
    Serial.print(F("IP address: "));
    Serial.println(WiFi.localIP());

    return true;
}

volatile bool otaInProgress = false;
bool otaReady               = false;

void OTAinit()
{
    ArduinoOTA.setHostname(wlan_hostname);
    ArduinoOTA.setPassword(OTA_PASS);

    ArduinoOTA.onStart([]()
                       {
        otaInProgress = true;
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
        Serial.println("OTA Start: " + type); });

    ArduinoOTA.onEnd([]()
                     {
        otaInProgress = false;
        Serial.println("\nOTA End — rebooting."); });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                          { Serial.printf("OTA Progress: %u%%\r", (progress * 100) / total); });

    ArduinoOTA.onError([](ota_error_t error)
                       {
        otaInProgress = false;
        Serial.printf("OTA Error[%u]: ", error);
        if      (error == OTA_AUTH_ERROR)    Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)   Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
#if defined(OTA_END_ERROR)
        else if (error == OTA_END_ERROR)     Serial.println("End Failed");
#endif
        else                                 Serial.println("Unknown"); });

    ArduinoOTA.begin();
    otaReady = true;
    Serial.printf("OTA ready — hostname: %s, IP: %s\n", wlan_hostname, WiFi.localIP().toString().c_str());
}

// MQTT ////////////////////////////////////////////////////////////////////////////////

bool mqttConnect()
{
    if (!WiFi.isConnected())
        return false;  // Non-blocking: WiFi reconnection is handled in loop()

    Serial.println("Connecting to MQTT broker...");
    mqttClient.setBufferSize(1024);  // Needed for HA Discovery payloads
    mqttClient.setKeepAlive(mqttKeepAlive_s);          // must be set before connect() — sent in CONNECT
    mqttClient.setSocketTimeout(mqttSocketTimeout_s);  // keep reconnect from starving the valve state machine
    if (mqttClient.connect("TankCommander", mqttTopicAvailability, 1, true, "offline"))
    {
        Serial.println("Connected.");
        if (mqttClient.subscribe(mqttTopicSubInterval))
        {
            Serial.print("Subscribed to ");
            Serial.println(mqttTopicSubInterval);
        }
        if (mqttClient.subscribe(mqttTopicSubKeepAwake))
        {
            Serial.print("Subscribed to ");
            Serial.println(mqttTopicSubKeepAwake);
        }
        if (mqttClient.subscribe(mqttTopicSubCal1))
        {
            Serial.print("Subscribed to ");
            Serial.println(mqttTopicSubCal1);
        }
        if (mqttClient.subscribe(mqttTopicSubCal2))
        {
            Serial.print("Subscribed to ");
            Serial.println(mqttTopicSubCal2);
        }
        if (mqttClient.subscribe(mqttTopicSubCal3))
        {
            Serial.print("Subscribed to ");
            Serial.println(mqttTopicSubCal3);
        }

        // Fill controller topics
        mqttClient.subscribe(mqttTopicFillCfgEnabled);
        mqttClient.subscribe(mqttTopicFillCfgTarget);
        mqttClient.subscribe(mqttTopicFillCfgLow);
        mqttClient.subscribe(mqttTopicFillCfgMaxDur);
        mqttClient.subscribe(mqttTopicFillCfgMode);
        mqttClient.subscribe(mqttTopicFillCfgOpening);
        mqttClient.subscribe(mqttTopicFillCfgLeakRise);
        mqttClient.subscribe(mqttTopicFillCfgLeakWindow);
        mqttClient.subscribe(mqttTopicFillCfgLeakWatch);
        mqttClient.subscribe(mqttTopicFillCommand);
        mqttClient.subscribe(mqttTopicFillOverride);
        mqttClient.subscribe(mqttTopicDiagRescanCmd);
        Serial.println("Subscribed to fill topics.");
        mqttPublishAvailability(true);
    }
    else
    {
        Serial.printf("Failed (state %d).\n", mqttClient.state());
        return false;
    }

#if TC_ALLOW_DEEP_SLEEP
    // Deep-sleep builds only: give the broker time to deliver retained commands
    // (interval, keep awake) before loop() decides whether to sleep. Always-on
    // builds skip this — messages are handled by loop() as they arrive.
    Serial.print("Waiting for retained messages");
    uint32_t tsDot = millis();
    uint32_t tsEnd = millis();
    while ((millis() - tsEnd) < 3000)
    {
        mqttClient.loop();
        if (mqttBytesAvailable)
            mqttDecode();  // decode each message — the callback holds only one at a time
        fillController->update();
        if (retainedInterval && retainedKeepAwake)
            break;

        if ((millis() - tsDot) > 500)
        {
            Serial.print(".");
            tsDot = millis();
        }
    }
    Serial.println();
#endif

    if (mqttClient.connected())
    {
        mqttPublishDeviceState();
        mqttPublishFillState();
    }

    return mqttClient.connected();
}

bool mqttPublishStates()
{
    if (!mqttClient.connected())
    {
        Serial.println("MQTT not connected — skipping publish (reconnect handled in loop()).");
        return false;
    }

    bool ok = true;
    for (size_t t = 0; t < tanks.size(); t++)
    {
        docOut[jsonTank_mm]    = tanks[t].mm();
        docOut[jsonTank_pc]    = tanks[t].pc();
        docOut[jsonTank_L]     = tanks[t].L();
        docOut[jsonTank_bus]   = tanks[t].busV();
        docOut["mA"]           = tanks[t].mA();  // raw current; useful when probe failed
        docOut["sensor_ok"]    = tanks[t].sensorOk();
        docOut["sensor_fault"] = tanks[t].sensorFaultStr();
        docOut["connected"]    = tanks[t].connected();
        docOut["init_ok"]      = tanks[t].initOk();
        docOut["fail_streak"]  = tanks[t].failStreak();
        // last_read_age_s: seconds since the last successful I2C read this wake.
        // 0 means "just read"; large values or 0xFFFFFFFF mean "never read OK
        // since boot/wake". Resets on deep sleep wake (no RTC RAM persistence).
        uint32_t lastOk           = tanks[t].lastReadOk_ms();
        docOut["last_read_age_s"] = lastOk
                                        ? (uint32_t)((millis() - lastOk) / 1000UL)
                                        : 0xFFFFFFFFu;

        size_t n = serializeJson(docOut, jsonBuffer);

        String sTank = mqttTopicPub;
        sTank += "/tank";
        sTank += (t + 1);
        // sprintf(sTank, "tank%u/", t+1);
        ok &= mqttClient.publish(sTank.c_str(), jsonBuffer, n);
        docOut.clear();
    }
    ok &= mqttPublishDeviceState();

    // Publish fill controller state
    ok &= mqttPublishFillState();

    if (ok)
        Serial.println("MQTT sent.");

    return ok;
}

void mqttCallback(char* topic, uint8_t* payload, unsigned int length)
{
    mqttTopic          = topic;
    mqttPayload        = payload;
    mqttBytesAvailable = length;
}

void mqttDecode()
{
    // handle message arrived
    Serial.println("MQTT Message Received.");
    String payload;
    String topic(mqttTopic);
    bool publishDeviceAck = false;
    bool publishFillAck   = false;

    for (size_t i = 0; i < mqttBytesAvailable; i++)
    {
        // Serial.print((char)mqttPayload[i]);
        payload += (char)mqttPayload[i];
    }

    if (topic == mqttTopicSubInterval)
    {
        setTxInterval(payload.toInt());
        Serial.printf("Received - Interval Time: %u s\n", (uint32_t)(tTxInterval_ms / 1000));
        // Keep-alive is deliberately NOT tied to the interval: PubSubClient only
        // sends it in CONNECT, so changing it mid-session desyncs from the broker.
        retainedInterval = true;
        publishDeviceAck = true;
    }

    if (topic == mqttTopicSubKeepAwake)
    {
        keepAwake_sw = (payload == "True");
        Serial.printf("Received - Keep Awake : %u\n", keepAwake_sw);
        retainedKeepAwake = true;
        publishDeviceAck  = true;
    }

    // Calibration commands — payload format: "m,b" (e.g. "128.57,-516.79")
    if (topic == mqttTopicSubCal1 || topic == mqttTopicSubCal2 || topic == mqttTopicSubCal3)
    {
        float m = 0, b = 0;
        if (sscanf(payload.c_str(), "%f,%f", &m, &b) == 2)
        {
            uint8_t idx = (topic == mqttTopicSubCal1) ? 0 : (topic == mqttTopicSubCal2) ? 1
                                                                                        : 2;
            if (calValid(m, b))
            {
                tanks[idx].setCali_mA_to_mm(m, b);
                if (!calSave(idx, m, b))
                    Serial.println("Calibration save failed.");
                Serial.printf("Received - Tank %u calibration: m=%0.4f, b=%0.4f\n", idx + 1, m, b);
            }
            else
            {
                Serial.printf("Tank %u calibration rejected: m=%0.4f, b=%0.4f\n", idx + 1, m, b);
            }
        }
        else
        {
            Serial.println("Calibration parse error — expected format: \"m,b\"");
        }
    }

    // Fill controller config
    if (topic == mqttTopicFillCfgMode)
    {
        uint8_t mode = 0;  // default CLOSED
        if (payload == "auto")
            mode = 1;
        else if (payload == "open")
            mode = 2;
        fillController->setValveMode(mode);
        publishFillAck = true;
    }
    else if (topic == mqttTopicFillCfgEnabled)
    {
        fillController->setEnabled(payload == "true");
        publishFillAck = true;
    }
    else if (topic == mqttTopicFillCfgTarget)
    {
        fillController->setTarget(payload.toFloat());
        publishFillAck = true;
    }
    else if (topic == mqttTopicFillCfgLow)
    {
        fillController->setLowThreshold(payload.toFloat());
        publishFillAck = true;
    }
    else if (topic == mqttTopicFillCfgMaxDur)
    {
        fillController->setMaxDuration(payload.toInt());
        publishFillAck = true;
    }
    else if (topic == mqttTopicFillCfgOpening)
    {
        fillController->setValveOpening((uint8_t)constrain(payload.toInt(), 0, 100));
        publishFillAck = true;
    }
    else if (topic == mqttTopicFillCfgLeakRise)
    {
        fillController->setLeakRise(payload.toFloat());
        publishFillAck = true;
    }
    else if (topic == mqttTopicFillCfgLeakWindow)
    {
        fillController->setLeakWindow((uint8_t)constrain(payload.toInt(), 0, 255));
        publishFillAck = true;
    }
    else if (topic == mqttTopicFillCfgLeakWatch)
    {
        fillController->setLeakWatch((uint8_t)constrain(payload.toInt(), 0, 255));
        publishFillAck = true;
    }

    // Fill controller commands
    if (topic == mqttTopicFillCommand)
    {
        if (payload == "start")
            fillController->commandStart();
        else if (payload == "stop")
            fillController->commandStop();
        publishFillAck = true;
    }
    if (topic == mqttTopicFillOverride)
    {
        if (payload == "clear_fault")
            fillController->commandClearFault();
        publishFillAck = true;
    }

    // Diagnostic: on-demand I2C bus rescan triggered by HA button
    if (topic == mqttTopicDiagRescanCmd)
    {
        Serial.println("Received - I2C rescan command.");
        publishI2CScan();
    }

    Serial.println();
    if (mqttClient.connected())
    {
        if (publishDeviceAck)
            mqttPublishDeviceState();
        if (publishFillAck)
            mqttPublishFillState();
    }
    mqttBytesAvailable = 0;
    // mqttClient.loop();
}

// POWER SAVING //////////////////////////////////////////////////////////////////

void sleep()
{
#if TC_ALLOW_DEEP_SLEEP
    // Defense-in-depth: ensure both relays are OFF before deep sleep
    digitalWrite(P_VALVE_PWR, HIGH);  // Power relay OFF
    digitalWrite(P_VALVE_DIR, HIGH);  // Direction relay to Close

    for (size_t t = 0; t < tanks.size(); t++)
        tanks[t].powerSave(true);

    if (mqttClient.connected())
        mqttPublishAvailability(false);

    mqttClient.disconnect();
    WiFi.disconnect();
    Serial.println("Entering deep sleep mode.");
    tcStartDeepSleep(sleepTime_us);
    delay(100);  // Note: required for deepSleep (above) to work
#else
    Serial.println("Sleep requested, but deep sleep is disabled in this build.");
#endif
}

void sleepInit()
{
    pinMode(P_KEEPAWAKE, INPUT_PULLUP);
}

// GENERAL ////////////////////////////////////////////////////////////////////////
void setTxInterval(uint64_t s)
{
    // TODO: Store this value in RTC RAM - or look at retained MQTT
    tTxInterval_ms = (s > 0) ? (s * 1000UL) : 1000;  // [ms] Prevent continous transmission
    tsTransmit_ms  = millis() + tTxInterval_ms;      // [ms] read and trqnsmit immediately
    sleepTime_us   = tTxInterval_ms * 1e3;           // [us] sleep time
}

// FILL STATE PUBLISHING ///////////////////////////////////////////////////////////

bool mqttPublishFillState()
{
    StaticJsonDocument<768> doc;
    doc["state"]            = fillController->stateStr();
    doc["filling"]          = fillController->isFilling();
    doc["enabled"]          = fillController->config().fillEnabled;
    doc["target_pc"]        = fillController->config().targetLevel_pc;
    doc["low_threshold_pc"] = fillController->config().lowThreshold_pc;
    doc["max_duration_min"] = fillController->config().maxDuration_min;
    doc["tank3_pc"]         = tanks[2].pc();
    doc["tank3_L"]          = tanks[2].L();
    doc["sensor_ok"]        = tanks[2].sensorOk();
    doc["fault"]            = fillController->faultStr();
    doc["fill_elapsed_min"] = fillController->fillElapsedMin();
    doc["fill_reason"]      = fillController->fillReasonStr();
    doc["fill_rate_pc_min"] = fillController->fillRatePcPerMin();
    const char* modeNames[] = {"closed", "auto", "open"};
    uint8_t modeIdx         = fillController->config().valveMode;
    doc["mode"]             = modeNames[modeIdx > 2 ? 0 : modeIdx];
    doc["valve"]            = fillController->valveStateStr();
    doc["opening_pc"]       = fillController->valveOpeningPc();
    doc["valve_pos_pc"]     = fillController->valvePosPc();
    doc["leak_rise_pc"]     = fillController->leakRisePc();
    doc["leak_window_min"]  = fillController->leakWindowMin();
    doc["leak_watch_min"]   = fillController->leakWatchMin();

    char buf[768];
    size_t n = serializeJson(doc, buf);
    return mqttClient.publish(mqttTopicFillState, (const uint8_t*)buf, n, true);  // retained
}

bool mqttPublishDeviceState()
{
#if USE_BAT
    docOut[jsonBat_V] = bat_V;
#endif
    docOut[jsonIntrvl]      = msToString(tTxInterval_ms);
    docOut["interval_s"]    = (uint32_t)(tTxInterval_ms / 1000);
    docOut[jsonKeepAwakeSW] = keepAwake_sw;
    docOut[jsonKeepAwakeHW] = (bool)digitalRead(P_KEEPAWAKE);
    docOut["rssi"]          = WiFi.RSSI();
    docOut["ip"]            = WiFi.localIP().toString();
    docOut["heap"]          = ESP.getFreeHeap();
    docOut["uptime_s"]      = (uint32_t)(millis() / 1000);
    docOut["reset_reason"]  = tcResetReasonStr();  // string literal — no doc pool copy

    size_t n = serializeJson(docOut, jsonBuffer);
    bool ok  = mqttClient.publish(mqttTopicPub, jsonBuffer, n);
    docOut.clear();
    return ok;
}

bool mqttPublishAvailability(bool online)
{
    return mqttClient.publish(mqttTopicAvailability, online ? "online" : "offline", true);
}

// HA MQTT DISCOVERY ///////////////////////////////////////////////////////////////

static void publishDiscoveryEntity(const char* component, const char* objectId,
                                   const char* name, const char* extraJson)
{
    // Build topic: homeassistant/<component>/tankcommander/<objectId>/config
    String topic = "homeassistant/";
    topic += component;
    topic += "/tankcommander/";
    topic += objectId;
    topic += "/config";

    // Build payload with device block
    StaticJsonDocument<768> doc;
    doc["name"] = name;
    String uid  = "tankcommander_";
    uid += objectId;
    doc["unique_id"] = uid;

    // Device block
    JsonObject dev = doc.createNestedObject("device");
    JsonArray ids  = dev.createNestedArray("identifiers");
    ids.add("tankcommander");
    dev["name"]         = "Tank Commander";
    dev["model"]        = TC_BOARD_MODEL;
    dev["manufacturer"] = "TankCommander";
    char swVer[12];
    snprintf(swVer, sizeof(swVer), "%u.%u.%u", VER_MAJOR, VER_MINOR, VER_PATCH);
    dev["sw_version"]            = swVer;
    doc["availability_topic"]    = mqttTopicAvailability;
    doc["payload_available"]     = "online";
    doc["payload_not_available"] = "offline";

    // Parse extra JSON fields into the doc
    StaticJsonDocument<384> extra;
    deserializeJson(extra, extraJson);
    for (JsonPair kv : extra.as<JsonObject>())
        doc[kv.key()] = kv.value();

    char buf[768];
    size_t n = serializeJson(doc, buf);
    mqttClient.publish(topic.c_str(), (const uint8_t*)buf, n, true);  // retained
}

void mqttPublishHADiscovery()
{
    Serial.println("Publishing HA MQTT Discovery configs...");

    // Remove deprecated entities and stale retained messages from previous versions
    mqttClient.publish("homeassistant/switch/tankcommander/fill_enable/config", "", true);
    mqttClient.publish("homeassistant/switch/tankcommander/fill_command/config", "", true);
    mqttClient.publish("homeassistant/switch/tankcommander/keep_awake/config", "", true);
    mqttClient.publish("tanks/fill/config/enabled", "", true);                                       // clear stale retained command
    mqttClient.publish("homeassistant/binary_sensor/tankcommander/sensor_health/config", "", true);  // replaced by per-tank entities
    // v0.7.4: per-tank read_uptime entities renamed to last_read_age — clear the old configs
    mqttClient.publish("homeassistant/sensor/tankcommander/tank1_read_uptime/config", "", true);
    mqttClient.publish("homeassistant/sensor/tankcommander/tank2_read_uptime/config", "", true);
    mqttClient.publish("homeassistant/sensor/tankcommander/tank3_read_uptime/config", "", true);

    // Valve Mode select (replaces Fill Enable + Fill Command switches)
    publishDiscoveryEntity("select", "valve_mode", "Valve Mode",
                           "{\"command_topic\":\"tanks/fill/config/mode\","
                           "\"state_topic\":\"tanks/fill/state\","
                           "\"value_template\":\"{{ value_json.mode }}\","
                           "\"options\":[\"closed\",\"auto\",\"open\"],"
                           "\"icon\":\"mdi:valve\"}");

    // Target Level number
    publishDiscoveryEntity("number", "fill_target", "Fill Target Level",
                           "{\"command_topic\":\"tanks/fill/config/target\","
                           "\"state_topic\":\"tanks/fill/state\","
                           "\"value_template\":\"{{ value_json.target_pc }}\","
                           "\"min\":10,\"max\":92,\"step\":1,"  // max = FILL_TARGET_MAX_PC
                           "\"unit_of_measurement\":\"%\"}");

    // Low Threshold number
    publishDiscoveryEntity("number", "fill_low_threshold", "Fill Low Threshold",
                           "{\"command_topic\":\"tanks/fill/config/low_threshold\","
                           "\"state_topic\":\"tanks/fill/state\","
                           "\"value_template\":\"{{ value_json.low_threshold_pc }}\","
                           "\"min\":5,\"max\":95,\"step\":5,"
                           "\"unit_of_measurement\":\"%\"}");

    // Max Duration number
    publishDiscoveryEntity("number", "fill_max_duration", "Fill Max Duration",
                           "{\"command_topic\":\"tanks/fill/config/max_duration\","
                           "\"state_topic\":\"tanks/fill/state\","
                           "\"value_template\":\"{{ value_json.max_duration_min }}\","
                           "\"min\":10,\"max\":480,\"step\":10,"  // max = FILL_HARD_MAX_MIN
                           "\"unit_of_measurement\":\"min\"}");

    // Fill State sensor
    publishDiscoveryEntity("sensor", "fill_state", "Fill State",
                           "{\"state_topic\":\"tanks/fill/state\","
                           "\"value_template\":\"{{ value_json.state }}\","
                           "\"device_class\":\"enum\","
                           "\"options\":[\"disabled\",\"idle\",\"valve_opening\",\"filling\",\"valve_closing\",\"cooldown\",\"fault_sensor\",\"fault_timeout\",\"unknown\"],"
                           "\"icon\":\"mdi:water-pump\"}");

    // Valve State sensor
    publishDiscoveryEntity("sensor", "valve_state", "Valve State",
                           "{\"state_topic\":\"tanks/fill/state\","
                           "\"value_template\":\"{{ value_json.valve }}\","
                           "\"device_class\":\"enum\","
                           "\"options\":[\"opening_settle\",\"opening_travel\",\"opening\",\"closing_settle\",\"closing_travel\",\"closing\",\"adjusting\",\"open\",\"partial\",\"closed\",\"unknown\"],"
                           "\"icon\":\"mdi:valve\"}");

    // Fill valve opening — trickle fill to limit the pressure drop on the house supply.
    // Percent of valve travel TIME, not flow: ball valves are non-linear.
    publishDiscoveryEntity("number", "fill_opening", "Fill Valve Opening",
                           "{\"command_topic\":\"tanks/fill/config/opening\","
                           "\"state_topic\":\"tanks/fill/state\","
                           "\"value_template\":\"{{ value_json.opening_pc }}\","
                           "\"min\":10,\"max\":100,\"step\":5,"  // min = FILL_OPENING_MIN_PC
                           "\"unit_of_measurement\":\"%\","
                           "\"icon\":\"mdi:valve-open\"}");

    // Post-close leak check settings (stuck-open valve detection)
    publishDiscoveryEntity("number", "leak_rise", "Leak Check Rise",
                           "{\"command_topic\":\"tanks/fill/config/leak_rise\","
                           "\"state_topic\":\"tanks/fill/state\","
                           "\"value_template\":\"{{ value_json.leak_rise_pc }}\","
                           "\"min\":1,\"max\":10,\"step\":0.1,"  // LEAK_RISE_MIN_PC..LEAK_RISE_MAX_PC
                           "\"unit_of_measurement\":\"%\","
                           "\"entity_category\":\"config\","
                           "\"icon\":\"mdi:arrow-up-bold-box-outline\"}");
    publishDiscoveryEntity("number", "leak_window", "Leak Check Window",
                           "{\"command_topic\":\"tanks/fill/config/leak_window\","
                           "\"state_topic\":\"tanks/fill/state\","
                           "\"value_template\":\"{{ value_json.leak_window_min }}\","
                           "\"min\":1,\"max\":15,\"step\":1,"  // LEAK_WINDOW_MIN_MIN..LEAK_WINDOW_MAX_MIN
                           "\"unit_of_measurement\":\"min\","
                           "\"entity_category\":\"config\","
                           "\"icon\":\"mdi:timer-sand\"}");
    publishDiscoveryEntity("number", "leak_watch", "Leak Check Watch Time",
                           "{\"command_topic\":\"tanks/fill/config/leak_watch\","
                           "\"state_topic\":\"tanks/fill/state\","
                           "\"value_template\":\"{{ value_json.leak_watch_min }}\","
                           "\"min\":10,\"max\":120,\"step\":5,"  // LEAK_WATCH_MIN_MIN..LEAK_WATCH_MAX_MIN
                           "\"unit_of_measurement\":\"min\","
                           "\"entity_category\":\"config\","
                           "\"icon\":\"mdi:timer-outline\"}");

    // Estimated valve position (time-based)
    publishDiscoveryEntity("sensor", "valve_position", "Valve Position",
                           "{\"state_topic\":\"tanks/fill/state\","
                           "\"value_template\":\"{{ value_json.valve_pos_pc }}\","
                           "\"unit_of_measurement\":\"%\","
                           "\"entity_category\":\"diagnostic\","
                           "\"icon\":\"mdi:valve\"}");

    // Clear Fault button
    publishDiscoveryEntity("button", "clear_fault", "Clear Fill Fault",
                           "{\"command_topic\":\"tanks/fill/command/override\","
                           "\"payload_press\":\"clear_fault\","
                           "\"icon\":\"mdi:alert-remove\"}");

    // Fill fault classification (sensor_* / fill_timeout / no_progress / overflow_cap / valve_stuck_open / none)
    publishDiscoveryEntity("sensor", "fill_fault", "Fill Fault",
                           "{\"state_topic\":\"tanks/fill/state\","
                           "\"value_template\":\"{{ value_json.fault }}\","
                           "\"device_class\":\"enum\","
                           "\"options\":[\"none\",\"sensor_wire_break\",\"sensor_short\",\"sensor_unknown\",\"fill_timeout\",\"no_progress\",\"overflow_cap\",\"valve_stuck_open\"],"
                           "\"entity_category\":\"diagnostic\","
                           "\"icon\":\"mdi:alert-circle-outline\"}");

    // Fill elapsed time (minutes since current fill started; 0 when not filling)
    publishDiscoveryEntity("sensor", "fill_elapsed", "Fill Elapsed",
                           "{\"state_topic\":\"tanks/fill/state\","
                           "\"value_template\":\"{{ value_json.fill_elapsed_min }}\","
                           "\"unit_of_measurement\":\"min\","
                           "\"entity_category\":\"diagnostic\","
                           "\"icon\":\"mdi:timer-sand\"}");

    // Fill reason (auto_low / manual_command / ha_override / valve_reclose / none)
    publishDiscoveryEntity("sensor", "fill_reason", "Fill Reason",
                           "{\"state_topic\":\"tanks/fill/state\","
                           "\"value_template\":\"{{ value_json.fill_reason }}\","
                           "\"device_class\":\"enum\","
                           "\"options\":[\"none\",\"auto_low\",\"manual_command\",\"ha_override\",\"valve_reclose\"],"
                           "\"entity_category\":\"diagnostic\","
                           "\"icon\":\"mdi:information-outline\"}");

    // Fill rate — level rise per minute over the last progress window (0 when not filling)
    publishDiscoveryEntity("sensor", "fill_rate", "Fill Rate",
                           "{\"state_topic\":\"tanks/fill/state\","
                           "\"value_template\":\"{{ value_json.fill_rate_pc_min }}\","
                           "\"unit_of_measurement\":\"%/min\","
                           "\"state_class\":\"measurement\","
                           "\"entity_category\":\"diagnostic\","
                           "\"icon\":\"mdi:trending-up\"}");

    // Keep Awake switch is only meaningful when deep sleep is enabled.
#if TC_ALLOW_DEEP_SLEEP
    publishDiscoveryEntity("switch", "keep_awake", "Keep Awake",
                           "{\"command_topic\":\"tanks/commands/keepawake\","
                           "\"state_topic\":\"tanks\","
                           "\"value_template\":\"{{ 'True' if value_json.keepawake_sw else 'False' }}\","
                           "\"payload_on\":\"True\",\"payload_off\":\"False\","
                           "\"state_on\":\"True\",\"state_off\":\"False\","
                           "\"icon\":\"mdi:sleep-off\"}");
#endif

    // Keep Awake (Hardware) — physical jumper state on P_KEEPAWAKE pin
    publishDiscoveryEntity("binary_sensor", "keep_awake_hw", "Keep Awake (HW)",
                           "{\"state_topic\":\"tanks\","
                           "\"value_template\":\"{{ 'ON' if value_json.keepawake_hw else 'OFF' }}\","
                           "\"entity_category\":\"diagnostic\","
                           "\"icon\":\"mdi:jump-rope\"}");

    // Transmission Interval number
    publishDiscoveryEntity("number", "tx_interval", "Transmission Interval",
                           "{\"command_topic\":\"tanks/commands/interval\","
                           "\"state_topic\":\"tanks\","
                           "\"value_template\":\"{{ value_json.interval_s }}\","
                           "\"min\":10,\"max\":86400,\"step\":10,"
                           "\"unit_of_measurement\":\"s\","
                           "\"icon\":\"mdi:timer-outline\"}");

    // Tank sensors (existing data, now discoverable via HA Discovery)
    const char* tankNames[]  = {"Tank 1", "Tank 2", "Tank 3"};
    const char* tankTopics[] = {"tanks/tank1", "tanks/tank2", "tanks/tank3"};
    const char* tankIds[]    = {"tank1", "tank2", "tank3"};

    for (int i = 0; i < 3; i++)
    {
        // Level mm
        String id    = String(tankIds[i]) + "_mm";
        String name  = String(tankNames[i]) + " Level";
        String extra = "{\"state_topic\":\"" + String(tankTopics[i]) +
                       "\","
                       "\"value_template\":\"{{ value_json.mm | round(0) }}\","
                       "\"unit_of_measurement\":\"mm\","
                       "\"icon\":\"mdi:water\"}";
        publishDiscoveryEntity("sensor", id.c_str(), name.c_str(), extra.c_str());

        // Percent
        id    = String(tankIds[i]) + "_pc";
        name  = String(tankNames[i]) + " Percent";
        extra = "{\"state_topic\":\"" + String(tankTopics[i]) +
                "\","
                "\"value_template\":\"{{ value_json.pc | round(1) }}\","
                "\"unit_of_measurement\":\"%\","
                "\"icon\":\"mdi:water-percent\"}";
        publishDiscoveryEntity("sensor", id.c_str(), name.c_str(), extra.c_str());

        // Volume L
        id    = String(tankIds[i]) + "_L";
        name  = String(tankNames[i]) + " Volume";
        extra = "{\"state_topic\":\"" + String(tankTopics[i]) +
                "\","
                "\"value_template\":\"{{ value_json.L | round(0) }}\","
                "\"unit_of_measurement\":\"L\","
                "\"icon\":\"mdi:cup-water\"}";
        publishDiscoveryEntity("sensor", id.c_str(), name.c_str(), extra.c_str());

        // Sensor health
        id    = String(tankIds[i]) + "_health";
        name  = String(tankNames[i]) + " Sensor Health";
        extra = "{\"state_topic\":\"" + String(tankTopics[i]) +
                "\","
                "\"value_template\":\"{{ 'ON' if not value_json.sensor_ok else 'OFF' }}\","
                "\"device_class\":\"problem\","
                "\"entity_category\":\"diagnostic\"}";
        publishDiscoveryEntity("binary_sensor", id.c_str(), name.c_str(), extra.c_str());

        // INA219 connected
        id    = String(tankIds[i]) + "_connected";
        name  = String(tankNames[i]) + " Sensor Connected";
        extra = "{\"state_topic\":\"" + String(tankTopics[i]) +
                "\","
                "\"value_template\":\"{{ 'ON' if value_json.connected else 'OFF' }}\","
                "\"device_class\":\"connectivity\","
                "\"entity_category\":\"diagnostic\"}";
        publishDiscoveryEntity("binary_sensor", id.c_str(), name.c_str(), extra.c_str());

        // Age of last successful read — render "never" when the chip hasn't
        // produced a good reading this wake (sentinel 4294967295). Without
        // this guard, HA shows the raw uint32_max as a meaningless "4.2B s".
        id    = String(tankIds[i]) + "_last_read_age";
        name  = String(tankNames[i]) + " Last Read Age";
        extra = "{\"state_topic\":\"" + String(tankTopics[i]) +
                "\","
                "\"value_template\":\"{% if value_json.last_read_age_s == 4294967295 %}never{% else %}{{ value_json.last_read_age_s }}{% endif %}\","
                "\"entity_category\":\"diagnostic\","
                "\"icon\":\"mdi:clock-check-outline\"}";
        publishDiscoveryEntity("sensor", id.c_str(), name.c_str(), extra.c_str());

        // Raw current (mA) — diagnostic visibility into what the INA219 is
        // actually reading; useful for distinguishing wire-break from real
        // low-water states.
        id    = String(tankIds[i]) + "_mA";
        name  = String(tankNames[i]) + " Sensor Current";
        extra = "{\"state_topic\":\"" + String(tankTopics[i]) +
                "\","
                "\"value_template\":\"{{ value_json.mA | round(3) }}\","
                "\"unit_of_measurement\":\"mA\","
                "\"entity_category\":\"diagnostic\","
                "\"icon\":\"mdi:current-dc\"}";
        publishDiscoveryEntity("sensor", id.c_str(), name.c_str(), extra.c_str());

        // Bus voltage — diagnostic; INA219 should report ~3.3V when healthy.
        id    = String(tankIds[i]) + "_bus_V";
        name  = String(tankNames[i]) + " Bus Voltage";
        extra = "{\"state_topic\":\"" + String(tankTopics[i]) +
                "\","
                "\"value_template\":\"{{ value_json.bus_V | round(3) }}\","
                "\"unit_of_measurement\":\"V\","
                "\"device_class\":\"voltage\","
                "\"entity_category\":\"diagnostic\","
                "\"icon\":\"mdi:flash\"}";
        publishDiscoveryEntity("sensor", id.c_str(), name.c_str(), extra.c_str());

        // Sensor fault enum — exposes the specific fault classification
        // (wire_break / short_circuit / none).
        id    = String(tankIds[i]) + "_sensor_fault";
        name  = String(tankNames[i]) + " Sensor Fault";
        extra = "{\"state_topic\":\"" + String(tankTopics[i]) +
                "\","
                "\"value_template\":\"{{ value_json.sensor_fault }}\","
                "\"device_class\":\"enum\","
                "\"options\":[\"none\",\"wire_break\",\"short_circuit\"],"
                "\"entity_category\":\"diagnostic\","
                "\"icon\":\"mdi:alert-circle-outline\"}";
        publishDiscoveryEntity("sensor", id.c_str(), name.c_str(), extra.c_str());

        // Library init success — true if Adafruit_INA219::begin() succeeded.
        id    = String(tankIds[i]) + "_init_ok";
        name  = String(tankNames[i]) + " Library Init OK";
        extra = "{\"state_topic\":\"" + String(tankTopics[i]) +
                "\","
                "\"value_template\":\"{{ 'ON' if value_json.init_ok else 'OFF' }}\","
                "\"entity_category\":\"diagnostic\","
                "\"icon\":\"mdi:check-circle-outline\"}";
        publishDiscoveryEntity("binary_sensor", id.c_str(), name.c_str(), extra.c_str());

        // Consecutive bad-read streak — reaches FAIL_STREAK_DISCONNECT (3)
        // before connected() flips false. Lets you watch transient I2C
        // failures without waiting for full lock-out.
        id    = String(tankIds[i]) + "_fail_streak";
        name  = String(tankNames[i]) + " I2C Fail Streak";
        extra = "{\"state_topic\":\"" + String(tankTopics[i]) +
                "\","
                "\"value_template\":\"{{ value_json.fail_streak }}\","
                "\"entity_category\":\"diagnostic\","
                "\"icon\":\"mdi:counter\"}";
        publishDiscoveryEntity("sensor", id.c_str(), name.c_str(), extra.c_str());
    }

    // Diagnostic sensors
    publishDiscoveryEntity("sensor", "wifi_rssi", "WiFi Signal",
                           "{\"state_topic\":\"tanks\","
                           "\"value_template\":\"{{ value_json.rssi }}\","
                           "\"device_class\":\"signal_strength\","
                           "\"unit_of_measurement\":\"dBm\","
                           "\"entity_category\":\"diagnostic\","
                           "\"icon\":\"mdi:wifi\"}");

    publishDiscoveryEntity("sensor", "ip_address", "IP Address",
                           "{\"state_topic\":\"tanks\","
                           "\"value_template\":\"{{ value_json.ip }}\","
                           "\"entity_category\":\"diagnostic\","
                           "\"icon\":\"mdi:ip-network\"}");

    publishDiscoveryEntity("sensor", "free_heap", "Free Heap",
                           "{\"state_topic\":\"tanks\","
                           "\"value_template\":\"{{ value_json.heap }}\","
                           "\"unit_of_measurement\":\"B\","
                           "\"entity_category\":\"diagnostic\","
                           "\"icon\":\"mdi:memory\"}");

    publishDiscoveryEntity("sensor", "uptime", "Uptime",
                           "{\"state_topic\":\"tanks\","
                           "\"value_template\":\"{{ value_json.uptime_s }}\","
                           "\"device_class\":\"duration\","
                           "\"unit_of_measurement\":\"s\","
                           "\"entity_category\":\"diagnostic\","
                           "\"icon\":\"mdi:clock-outline\"}");

    publishDiscoveryEntity("sensor", "reset_reason", "Last Reset Reason",
                           "{\"state_topic\":\"tanks\","
                           "\"value_template\":\"{{ value_json.reset_reason }}\","
                           "\"entity_category\":\"diagnostic\","
                           "\"icon\":\"mdi:restart-alert\"}");

    // I2C bus scan diagnostic — populated at boot and on-demand via rescan button
    publishDiscoveryEntity("sensor", "i2c_scan", "I2C Scan",
                           "{\"state_topic\":\"tanks/diag/i2c_scan\","
                           "\"entity_category\":\"diagnostic\","
                           "\"icon\":\"mdi:lan-connect\"}");

    publishDiscoveryEntity("button", "i2c_rescan", "I2C Rescan",
                           "{\"command_topic\":\"tanks/diag/i2c_scan/rescan\","
                           "\"payload_press\":\"scan\","
                           "\"entity_category\":\"diagnostic\","
                           "\"icon\":\"mdi:refresh\"}");

    Serial.println("HA Discovery published.");
}

// GENERAL ////////////////////////////////////////////////////////////////////////

String msToString(uint64_t ms)
{
    uint years, days, hours, mins, secs, msRem = ms;
    char buff[12];

    years = msRem / TIME_MS_IN_Y;
    msRem -= years * TIME_MS_IN_Y;
    days = msRem / TIME_MS_IN_D;
    msRem -= days * TIME_MS_IN_D;
    hours = msRem / TIME_MS_IN_H;
    msRem -= hours * TIME_MS_IN_H;
    mins = msRem / TIME_MS_IN_M;
    msRem -= mins * TIME_MS_IN_M;
    secs = msRem / TIME_MS_IN_S;
    msRem -= secs * TIME_MS_IN_S;

    //   snprintf(buff, sizeof(buff), "%02u:%02u:%02u:%02u", years, days, hours, mins); // "dd:hh:mm:ss"

    if (days > 0)
        snprintf(buff, sizeof(buff), "%02u:%02u:%02u:%02u", days, hours, mins, secs);  // "dd:hh:mm:ss"
    else
        snprintf(buff, sizeof(buff), "%02u:%02u:%02u", hours, mins, secs);  // "hh:mm:ss"

    return String(buff);
}