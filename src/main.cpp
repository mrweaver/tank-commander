#include "main.h"

void setup(void)
{
    // Boot-safe relay init — MUST be first, before anything else.
    // Both pins written HIGH (de-energised) BEFORE setting pinMode to OUTPUT,
    // preventing the relays from briefly firing during ESP8266 boot.
    digitalWrite(P_VALVE_PWR, HIGH);     // Power relay OFF
    digitalWrite(P_VALVE_DIR, HIGH); // Direction relay to Close (de-energised)
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
    // P_VALVE_PWR = power relay (D6), P_VALVE_DIR = direction relay (D7)
    fillController = new FillController(P_VALVE_PWR, P_VALVE_DIR, &tanks[2]);
    fillController->begin();

    wifiConnect();
    OTAinit();
    mqttConnect();
    mqttPublishHADiscovery();
    sleepInit(); // does not trigger sleep - prepares buttons etc.
}

void loop(void)
{
    // These polling functions only work when sleep switch is deactivated.
    ArduinoOTA.handle();
    mqttClient.loop();

    // mqtt Receive
    if (mqttBytesAvailable) // triggered through ISR
        mqttDecode();       // if enough time has elapsed

    // Serial.printf("millis(): %u, tsTransmit: %llu, interval: %llu\n", millis(), tsTransmit_ms, tTxInterval_ms);
    if ((millis() - tsTransmit_ms) > tTxInterval_ms)
    {
        readSensors();
        if (mqttPublishStates())      // broadcast new states to MQTT broker
            tsTransmit_ms = millis(); // if successful, timestamp
    }

    fillController->update();

    if (!keepAwake_sw && !digitalRead(P_KEEPAWAKE) && !fillController->keepAwake() && !otaInProgress)
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
    delay(bootTime_ms); // wait for powerup
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
        if (!tanks[t].connected())
            break;

        Serial.printf("Tank %u:\n", t + 1);
        tanks[t].read();

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

// Tanks //////////////////////////////////////////////////////////////////////////////
void tankInit()
{
#if USE_BOOST
    pinMode(P_BOOST, OUTPUT);
#endif

    // Create tank instances
    tanks.push_back(Tank(TANK1_ADDR, TANK1_D_MM, TANK1_H_MM)); // Tank 1
    tanks.push_back(Tank(TANK2_ADDR, TANK2_D_MM, TANK2_H_MM)); // Tank 2
    tanks.push_back(Tank(TANK3_ADDR, TANK3_D_MM, TANK3_H_MM)); // Tank 3
}

// WiFi ////////////////////////////////////////////////////////////////////////////////
bool wifiConnect()
{
    Serial.print("Connecting to ");
    Serial.print(wlan_ssid);
    WiFi.hostname(wlan_hostname);
    WiFi.begin(wlan_ssid, wlan_pass);
    // WiFi.setSleepMode();
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(100);
        Serial.print(F(".")); // locked here forever if no wifi
    }
    Serial.println();
    Serial.println(F("WiFi connected."));
    Serial.print(F("IP address: "));
    Serial.println(WiFi.localIP());

    return WiFi.isConnected();
}

volatile bool otaInProgress = false;

void OTAinit()
{
    ArduinoOTA.setHostname(wlan_hostname);
    ArduinoOTA.setPassword(OTA_PASS);

    ArduinoOTA.onStart([]() {
        otaInProgress = true;
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
        Serial.println("OTA Start: " + type);
    });

    ArduinoOTA.onEnd([]() {
        otaInProgress = false;
        Serial.println("\nOTA End — rebooting.");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("OTA Progress: %u%%\r", (progress * 100) / total);
    });

    ArduinoOTA.onError([](ota_error_t error) {
        otaInProgress = false;
        Serial.printf("OTA Error[%u]: ", error);
        if      (error == OTA_AUTH_ERROR)    Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)   Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)     Serial.println("End Failed");
    });

    ArduinoOTA.begin();
    Serial.printf("OTA ready — hostname: %s, IP: %s\n", wlan_hostname, WiFi.localIP().toString().c_str());
}

// MQTT ////////////////////////////////////////////////////////////////////////////////

bool mqttConnect()
{
    if (!WiFi.isConnected())
        if (!wifiConnect())
            return false;

    Serial.println("Connecting to MQTT broker...");
    mqttClient.setBufferSize(768); // Needed for HA Discovery payloads
    if (mqttClient.connect("TankCommander"))
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
        mqttClient.subscribe(mqttTopicFillCommand);
        mqttClient.subscribe(mqttTopicFillOverride);
        Serial.println("Subscribed to fill topics.");
    }
    else
    {
        Serial.println("Failed.");
    }

    Serial.print("Waiting for retained messages");

    // allow  time for retained messages.
    uint8_t cnt = 0;
    bool success = false;
    // use millis to create a continous polling wiht timer.
    uint64_t tsDot = millis();
    uint64_t tsEnd = millis();
    while ((millis() - tsEnd) < 3000) // todo create min and max time
    {
        mqttClient.loop();
        if (success = (retainedInterval & retainedKeepAwake))
            break;

        if ((millis() - tsDot) > 500)
        {
            Serial.print(".");
            tsDot = millis();
        }
    }
    Serial.println();
    return mqttClient.connected();
}

bool mqttPublishStates()
{
    uint8_t retry = 0;
    while (!mqttClient.connected() && (retry < mqttMaxRetry))
    {
        retry++;
        Serial.println("No existing MQTT connection.");
        Serial.printf("Retry attempt: %u\n", retry);
        mqttConnect();
        delay(1000);
    }
    if (retry >= mqttMaxRetry)
    {
        Serial.println("Retries failed. Aborting operation.");
        return false;
    }

    bool ok = true;
    for (size_t t = 0; t < tanks.size(); t++)
    {
        if (!tanks[t].connected())
            break;

        docOut[jsonTank_mm] = tanks[t].mm();
        docOut[jsonTank_pc] = tanks[t].pc();
        docOut[jsonTank_L] = tanks[t].L();
        docOut[jsonTank_bus] = tanks[t].busV();

        size_t n = serializeJson(docOut, jsonBuffer);

        String sTank = mqttTopicPub;
        sTank += "/tank";
        sTank += (t + 1);
        // sprintf(sTank, "tank%u/", t+1);
        ok &= mqttClient.publish(sTank.c_str(), jsonBuffer, n);
        docOut.clear();
    }
#if USE_BAT
    docOut[jsonBat_V] = bat_V;
#endif
    docOut[jsonIntrvl] = msToString(tTxInterval_ms);
    docOut["interval_s"] = (uint32_t)(tTxInterval_ms / 1000);
    docOut[jsonKeepAwakeSW] = keepAwake_sw;
    docOut[jsonKeepAwakeHW] = (bool)digitalRead(P_KEEPAWAKE);
    docOut["rssi"] = WiFi.RSSI();
    docOut["ip"] = WiFi.localIP().toString();
    docOut["heap"] = ESP.getFreeHeap();
    docOut["uptime_s"] = (uint32_t)(millis() / 1000);

    size_t n = serializeJson(docOut, jsonBuffer);
    ok &= mqttClient.publish(mqttTopicPub, jsonBuffer, n);
    docOut.clear();

    // Publish fill controller state
    mqttPublishFillState();

    if (ok)
        Serial.println("MQTT sent.");

    return ok;
}

ICACHE_RAM_ATTR void mqttCallback(char *topic, uint8_t *payload, unsigned int length)
{
    noInterrupts();
    mqttTopic = topic;
    mqttPayload = payload;
    mqttBytesAvailable = length;
    interrupts();
}

void mqttDecode()
{
    // handle message arrived
    Serial.println("MQTT Message Received.");
    String payload;
    String topic(mqttTopic);

    for (size_t i = 0; i < mqttBytesAvailable; i++)
    {
        // Serial.print((char)mqttPayload[i]);
        payload += (char)mqttPayload[i];
    }

    if (topic == mqttTopicSubInterval)
    {
        setTxInterval(payload.toInt());
        Serial.printf("Received - Interval Time: %u s\n", (uint32_t)(tTxInterval_ms / 1000));
        mqttClient.setKeepAlive(tTxInterval_ms / 1000);
        retainedInterval = true;
    }

    if (topic == mqttTopicSubKeepAwake)
    {
        keepAwake_sw = (payload == "True");
        Serial.printf("Received - Keep Awake : %u\n", keepAwake_sw);
        retainedKeepAwake = true;
    }

    // Calibration commands — payload format: "m,b" (e.g. "128.57,-516.79")
    if (topic == mqttTopicSubCal1 || topic == mqttTopicSubCal2 || topic == mqttTopicSubCal3)
    {
        float m = 0, b = 0;
        if (sscanf(payload.c_str(), "%f,%f", &m, &b) == 2)
        {
            uint8_t idx = (topic == mqttTopicSubCal1) ? 0 :
                          (topic == mqttTopicSubCal2) ? 1 : 2;
            tanks[idx].setCali_mA_to_mm(m, b);
            Serial.printf("Received - Tank %u calibration: m=%0.4f, b=%0.4f\n", idx + 1, m, b);
        }
        else
        {
            Serial.println("Calibration parse error — expected format: \"m,b\"");
        }
    }

    // Fill controller config
    if (topic == mqttTopicFillCfgEnabled)
        fillController->setEnabled(payload == "true");
    else if (topic == mqttTopicFillCfgTarget)
        fillController->setTarget(payload.toFloat());
    else if (topic == mqttTopicFillCfgLow)
        fillController->setLowThreshold(payload.toFloat());
    else if (topic == mqttTopicFillCfgMaxDur)
        fillController->setMaxDuration(payload.toInt());

    // Fill controller commands
    if (topic == mqttTopicFillCommand)
    {
        if (payload == "start")
            fillController->commandStart();
        else if (payload == "stop")
            fillController->commandStop();
    }
    if (topic == mqttTopicFillOverride)
    {
        if (payload == "clear_fault")
            fillController->commandClearFault();
    }

    Serial.println();
    tsTransmit_ms = millis() + tTxInterval_ms - 1000; // force readiness
    mqttBytesAvailable = 0;
    // mqttClient.loop();
}

// POWER SAVING //////////////////////////////////////////////////////////////////

void sleep()
{
    // Defense-in-depth: ensure both relays are OFF before deep sleep
    digitalWrite(P_VALVE_PWR, HIGH);     // Power relay OFF
    digitalWrite(P_VALVE_DIR, HIGH); // Direction relay to Close

    for (size_t t = 0; t < tanks.size(); t++)
        tanks[t].powerSave(true);

    mqttClient.disconnect();
    WiFi.disconnect();
    Serial.println("Entering deep sleep mode.");
    ESP.deepSleep(sleepTime_us, WAKE_NO_RFCAL);
    delay(100); // Note: required for deepSleep (above) to work
}

void sleepInit()
{
    pinMode(P_KEEPAWAKE, INPUT_PULLUP);
}

// GENERAL ////////////////////////////////////////////////////////////////////////
void setTxInterval(uint64_t s)
{
    // TODO: Store this value in RTC RAM - or look at retained MQTT
    tTxInterval_ms = (s > 0) ? (s * 1000UL) : 1000; // [ms] Prevent continous transmission
    tsTransmit_ms = millis() + tTxInterval_ms;      // [ms] read and trqnsmit immediately
    sleepTime_us = tTxInterval_ms * 1e3;            // [us] sleep time
}

// FILL STATE PUBLISHING ///////////////////////////////////////////////////////////

void mqttPublishFillState()
{
    StaticJsonDocument<256> doc;
    doc["state"] = fillController->stateStr();
    doc["filling"] = fillController->isFilling();
    doc["enabled"] = fillController->config().fillEnabled;
    doc["target_pc"] = fillController->config().targetLevel_pc;
    doc["low_threshold_pc"] = fillController->config().lowThreshold_pc;
    doc["max_duration_min"] = fillController->config().maxDuration_min;
    doc["tank3_pc"] = tanks[2].pc();
    doc["tank3_L"] = tanks[2].L();
    doc["sensor_ok"] = tanks[2].sensorOk();
    doc["fault"] = fillController->faultStr();
    doc["fill_elapsed_min"] = fillController->fillElapsedMin();
    doc["fill_reason"] = fillController->fillReasonStr();
    doc["valve"] = fillController->valveStateStr();

    char buf[256];
    size_t n = serializeJson(doc, buf);
    mqttClient.publish(mqttTopicFillState, (const uint8_t*)buf, n, true); // retained
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
    String uid = "tankcommander_";
    uid += objectId;
    doc["unique_id"] = uid;

    // Device block
    JsonObject dev = doc.createNestedObject("device");
    JsonArray ids = dev.createNestedArray("identifiers");
    ids.add("tankcommander");
    dev["name"] = "Tank Commander";
    dev["model"] = "D1 Mini Lite";
    dev["manufacturer"] = "TankCommander";
    char swVer[12];
    snprintf(swVer, sizeof(swVer), "%u.%u.%u", VER_MAJOR, VER_MINOR, VER_PATCH);
    dev["sw_version"] = swVer;

    // Parse extra JSON fields into the doc
    StaticJsonDocument<384> extra;
    deserializeJson(extra, extraJson);
    for (JsonPair kv : extra.as<JsonObject>())
        doc[kv.key()] = kv.value();

    char buf[768];
    size_t n = serializeJson(doc, buf);
    mqttClient.publish(topic.c_str(), (const uint8_t*)buf, n, true); // retained
}

void mqttPublishHADiscovery()
{
    Serial.println("Publishing HA MQTT Discovery configs...");

    // Fill Enable switch
    publishDiscoveryEntity("switch", "fill_enable", "Fill Enable",
        "{\"command_topic\":\"tanks/fill/config/enabled\","
        "\"state_topic\":\"tanks/fill/state\","
        "\"value_template\":\"{{ value_json.enabled }}\","
        "\"payload_on\":\"true\",\"payload_off\":\"false\","
        "\"state_on\":\"True\",\"state_off\":\"False\"}");

    // Fill Command switch (unavailable when fill is disabled)
    publishDiscoveryEntity("switch", "fill_command", "Fill Command",
        "{\"command_topic\":\"tanks/fill/command\","
        "\"state_topic\":\"tanks/fill/state\","
        "\"value_template\":\"{{ value_json.filling }}\","
        "\"payload_on\":\"start\",\"payload_off\":\"stop\","
        "\"state_on\":\"True\",\"state_off\":\"False\","
        "\"availability_topic\":\"tanks/fill/state\","
        "\"availability_template\":\"{{ 'online' if value_json.state != 'disabled' else 'offline' }}\"}");

    // Target Level number
    publishDiscoveryEntity("number", "fill_target", "Fill Target Level",
        "{\"command_topic\":\"tanks/fill/config/target\","
        "\"state_topic\":\"tanks/fill/state\","
        "\"value_template\":\"{{ value_json.target_pc }}\","
        "\"min\":10,\"max\":100,\"step\":5,"
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
        "\"min\":5,\"max\":120,\"step\":5,"
        "\"unit_of_measurement\":\"min\"}");

    // Fill State sensor
    publishDiscoveryEntity("sensor", "fill_state", "Fill State",
        "{\"state_topic\":\"tanks/fill/state\","
        "\"value_template\":\"{{ value_json.state }}\","
        "\"device_class\":\"enum\","
        "\"options\":[\"disabled\",\"idle\",\"valve_opening\",\"filling\",\"valve_closing\",\"cooldown\",\"fault_sensor\",\"fault_timeout\",\"unknown\"],"
        "\"icon\":\"mdi:water-pump\"}");

    // Sensor Health binary sensor
    publishDiscoveryEntity("binary_sensor", "sensor_health", "Tank 3 Sensor Health",
        "{\"state_topic\":\"tanks/fill/state\","
        "\"value_template\":\"{{ 'ON' if not value_json.sensor_ok else 'OFF' }}\","
        "\"device_class\":\"problem\"}");

    // Valve State sensor
    publishDiscoveryEntity("sensor", "valve_state", "Valve State",
        "{\"state_topic\":\"tanks/fill/state\","
        "\"value_template\":\"{{ value_json.valve }}\","
        "\"device_class\":\"enum\","
        "\"options\":[\"opening_settle\",\"opening_travel\",\"opening\",\"closing_settle\",\"closing_travel\",\"closing\",\"open\",\"closed\",\"unknown\"],"
        "\"icon\":\"mdi:valve\"}");

    // Clear Fault button
    publishDiscoveryEntity("button", "clear_fault", "Clear Fill Fault",
        "{\"command_topic\":\"tanks/fill/command/override\","
        "\"payload_press\":\"clear_fault\","
        "\"icon\":\"mdi:alert-remove\"}");

    // Keep Awake switch
    publishDiscoveryEntity("switch", "keep_awake", "Keep Awake",
        "{\"command_topic\":\"tanks/commands/keepawake\","
        "\"state_topic\":\"tanks\","
        "\"value_template\":\"{{ 'True' if value_json.keepawake_sw else 'False' }}\","
        "\"payload_on\":\"True\",\"payload_off\":\"False\","
        "\"state_on\":\"True\",\"state_off\":\"False\","
        "\"icon\":\"mdi:sleep-off\"}");

    // Transmission Interval number
    publishDiscoveryEntity("number", "tx_interval", "Transmission Interval",
        "{\"command_topic\":\"tanks/commands/interval\","
        "\"state_topic\":\"tanks\","
        "\"value_template\":\"{{ value_json.interval_s }}\","
        "\"min\":10,\"max\":86400,\"step\":10,"
        "\"unit_of_measurement\":\"s\","
        "\"icon\":\"mdi:timer-outline\"}");

    // Tank sensors (existing data, now discoverable via HA Discovery)
    const char* tankNames[] = {"Tank 1", "Tank 2", "Tank 3"};
    const char* tankTopics[] = {"tanks/tank1", "tanks/tank2", "tanks/tank3"};
    const char* tankIds[] = {"tank1", "tank2", "tank3"};

    for (int i = 0; i < 3; i++)
    {
        // Level mm
        String id = String(tankIds[i]) + "_mm";
        String name = String(tankNames[i]) + " Level";
        String extra = "{\"state_topic\":\"" + String(tankTopics[i]) + "\","
            "\"value_template\":\"{{ value_json.mm | round(0) }}\","
            "\"unit_of_measurement\":\"mm\","
            "\"icon\":\"mdi:water\"}";
        publishDiscoveryEntity("sensor", id.c_str(), name.c_str(), extra.c_str());

        // Percent
        id = String(tankIds[i]) + "_pc";
        name = String(tankNames[i]) + " Percent";
        extra = "{\"state_topic\":\"" + String(tankTopics[i]) + "\","
            "\"value_template\":\"{{ value_json.pc | round(1) }}\","
            "\"unit_of_measurement\":\"%\","
            "\"icon\":\"mdi:water-percent\"}";
        publishDiscoveryEntity("sensor", id.c_str(), name.c_str(), extra.c_str());

        // Volume L
        id = String(tankIds[i]) + "_L";
        name = String(tankNames[i]) + " Volume";
        extra = "{\"state_topic\":\"" + String(tankTopics[i]) + "\","
            "\"value_template\":\"{{ value_json.L | round(0) }}\","
            "\"unit_of_measurement\":\"L\","
            "\"icon\":\"mdi:cup-water\"}";
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
        snprintf(buff, sizeof(buff), "%02u:%02u:%02u:%02u", days, hours, mins, secs); // "dd:hh:mm:ss"
    else
        snprintf(buff, sizeof(buff), "%02u:%02u:%02u", hours, mins, secs); // "hh:mm:ss"

    return String(buff);
}