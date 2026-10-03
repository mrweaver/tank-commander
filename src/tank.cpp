
#include "tank.h"
#include <Wire.h>

Tank::Tank(uint8_t ina_addr, float dia_mm, float maxHead_mm)
    : m_addr(ina_addr), m_dia_mm(dia_mm), m_maxH_mm(maxHead_mm)
{
    m_ina219 = new Adafruit_INA219(ina_addr);
    Serial.printf("Connecting to INA219 on address: %#02x - ", ina_addr);

    // Adafruit_INA219::begin() always returns true with default args (no
    // address detection), so a "true" here means I2C bus is up — not that the
    // chip is alive. We treat this as init OK and rely on the active probe in
    // read() to track real connectivity.
    m_initOk = m_ina219->begin();
    if (!m_initOk)
    {
        Serial.println("begin() failed — will retry in read().");
        return;
    }

    useAverage(true);
    m_ina219->setCalibration_16V_40mA();  // uses 1ohm shunt resistor
    // Explicitly set mode before final writeConfig — setCalibration_16V_40mA
    // ends with writeConfig() but ina219_mode is still 0 (POWERDOWN) at that
    // point. Without this, the chip is left powered down until first read().
    m_ina219->setMode(INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED);
    m_ina219->writeConfig();

    // Diagnostic-only probe — does NOT affect m_failStreak or connected().
    // Just prints boot-time chip status for visibility; the real probe runs
    // inside read() after the chip has had time to settle.
    if (m_ina219->isAlive())
        Serial.println("library OK, chip ACK confirmed.");
    else
        Serial.println("library OK, chip NOT YET ACKing (will retry in read()).");
}

bool Tank::probeI2C()
{
    // Keep the hot-path probe non-destructive: an address ACK is sufficient
    // to decide whether a read attempt is worth making.
    Wire.beginTransmission(m_addr);
    return Wire.endTransmission() == 0;
}

void Tank::read(uint8_t num)
{
    // Retry library begin() if it failed at construction. Cheap; default
    // arg path is just Wire.begin() + return true, but it also re-creates
    // the i2c_dev in case heap was tight at boot.
    if (!m_initOk)
    {
        m_initOk = m_ina219->begin();
        if (m_initOk)
        {
            useAverage(true);
            m_ina219->setCalibration_16V_40mA();
            m_ina219->setMode(INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED);
            m_ina219->writeConfig();
            Serial.printf("INA219 @%#02x: late-init succeeded.\n", m_addr);
        }
    }

    // Active connectivity probe — if the chip doesn't ACK, don't bother
    // reading and don't update cached values; let the streak counter trip
    // connected() to false after FAIL_STREAK_DISCONNECT consecutive misses.
    if (!m_initOk || !probeI2C())
    {
        if (m_failStreak < 0xFF) m_failStreak++;
        Serial.printf("\tINA219 @%#02x probe FAILED (streak=%u)\n", m_addr, m_failStreak);
        return;
    }
    Serial.printf("\tINA219 @%#02x probe OK\n", m_addr);

    powerSave(false);
    configReadout();
    m_ina219->triggerRead(true);

    m_mA            = m_ina219->getCurrent_mA();
    m_failStreak    = 0;
    m_lastReadOk_ms = millis();
    if (m_lastReadOk_ms == 0) m_lastReadOk_ms = 1;  // 0 reserved for "never"

    evaluateHealth();
    if (m_mA < sensorMin_mA)
    {
        m_mA = sensorMin_mA;
    }
    m_mm = m_mA * m_mA_to_mm_m_theory + m_mA_to_mm_b_theory;
    m_pc = (m_mm / m_maxH_mm) * 100;
    m_L  = (PI * pow(m_dia_mm / 2, 2) * m_mm * 1e-6);
}

void Tank::configReadout(bool useAve, bool useTrig)
{
    useAverage(useAve);
    if (useTrig)
        m_ina219->setMode(INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED);
    else
        m_ina219->setMode(INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS);
    m_ina219->setCalibration_16V_40mA();
}

float Tank::mA()
{
    return m_mA;
}

float Tank::busV()
{
    // return m_useAve ? m_busV->getAverage() : m_busV->getValue(m_busV->getCount() - 1);
    return m_ina219->getBusVoltage_V();
}

void Tank::evaluateHealth()
{
    // Wire break: current below 4mA minimum (with margin)
    if (m_mA < 3.5f)
    {
        m_sensorFault = SensorFault::WIRE_BREAK;
        m_sensorOk    = false;
        return;
    }

    // Short circuit: current above 20mA maximum (with margin)
    if (m_mA > 21.0f)
    {
        m_sensorFault = SensorFault::SHORT_CIRCUIT;
        m_sensorOk    = false;
        return;
    }

    m_sensorFault = SensorFault::NONE;
    m_sensorOk    = true;
}