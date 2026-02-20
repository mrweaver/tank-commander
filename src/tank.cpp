
#include "tank.h"

Tank::Tank(uint8_t ina_addr, float dia_mm, float maxHead_mm) : m_dia_mm(dia_mm), m_maxH_mm(maxHead_mm)
{
    m_ina219 = new Adafruit_INA219(ina_addr);
    Serial.printf("Connecting to INA219 on address: %#02x - ", ina_addr);
    // m_mA = new RunningAverage(10);
    // m_busV = new RunningAverage(10);

    // Sensor init ///////////
    // Initialize the INA219.
    // By default the initialization will use the largest range (32V, 2A). However
    // you can call a setCalibration function to change this range (see comments).
    if (!m_ina219->begin())
    {
        Serial.println("Failed!");
        m_connected = false;
        return;
    }
    // To use a slightly lower 32V, 1A range (higher precision on amps):
    // ina219.setCalibration_32V_1A();
    // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
    m_connected = true;
    useAverage(true);
    m_ina219->setCalibration_16V_40mA(); // uses 1ohm shunt resistor
    m_ina219->writeConfig();
    Serial.println("INA219 initialised.");
}

void Tank::read(uint8_t num)
{
    powerSave(false);
    configReadout();
    m_ina219->triggerRead(true);

    m_mA = m_ina219->getCurrent_mA();
    evaluateHealth();
    if (m_mA < sensorMin_mA)
    {
        m_mA = sensorMin_mA;
    }
    m_mm = m_mA * m_mA_to_mm_m_theory + m_mA_to_mm_b_theory;
    // Auto-adjust max height: if reading exceeds stored max, raise it
    if (m_mm > m_maxH_mm)
        m_maxH_mm = m_mm;
    m_pc = (m_mm / m_maxH_mm) * 100;
    m_L = (PI * pow(m_dia_mm / 2, 2) * m_mm * 1e-6);

    // for (size_t i = 0; i < num; i++)
    // {
    // m_mA->addValue(m_ina219->getCurrent_mA());
    // m_busV->addValue(m_ina219->getBusVoltage_V());
    // }
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
        m_sensorOk = false;
        return;
    }

    // Short circuit: current above 20mA maximum (with margin)
    if (m_mA > 21.0f)
    {
        m_sensorFault = SensorFault::SHORT_CIRCUIT;
        m_sensorOk = false;
        return;
    }

    // Stuck reading: same value for N consecutive reads
    m_lastmA[m_readIdx] = m_mA;
    m_readIdx = (m_readIdx + 1) % SENSOR_STUCK_COUNT;

    bool stuck = true;
    for (uint8_t i = 1; i < SENSOR_STUCK_COUNT; i++)
    {
        if (fabs(m_lastmA[i] - m_lastmA[0]) > SENSOR_STUCK_TOL)
        {
            stuck = false;
            break;
        }
    }
    // Only flag stuck if the buffer is full (all slots written at least once)
    // We use a simple heuristic: if m_lastmA[0] is non-zero, buffer is likely filled
    if (stuck && m_lastmA[0] > 0.0f)
    {
        m_sensorFault = SensorFault::STUCK_READING;
        m_sensorOk = false;
        return;
    }

    m_sensorFault = SensorFault::NONE;
    m_sensorOk = true;
}