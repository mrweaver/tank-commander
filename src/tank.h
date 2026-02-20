#ifndef _H_TANKS_
#define _H_TANKS_

#include "Adafruit_INA219.h" // TODO: copy this to lib when complete.

#define SENSOR_STUCK_COUNT 5    // consecutive identical readings to trigger stuck fault
#define SENSOR_STUCK_TOL   0.01 // mA tolerance for stuck detection

enum class SensorFault : uint8_t {
    NONE = 0,
    WIRE_BREAK,     // mA < 3.5 (below 4mA minimum)
    SHORT_CIRCUIT,  // mA > 21.0 (above 20mA maximum)
    STUCK_READING   // same value for SENSOR_STUCK_COUNT consecutive reads
};

class Tank
{
public:
    Tank(uint8_t ina_addr, float dia_mm, float maxHead_mm);
    // ~Tank();

    // Setters
    void read(uint8_t num = 1); // take "num" readings
    // void setmA(float mA) { m_mA->addValue(mA); }
    void setDiameter(float dia_mm) { m_dia_mm = dia_mm; }
    void setMaxHead(float head_mm) { m_maxH_mm = head_mm; }
    void useAverage(bool useAve)
    {
        m_useAve = useAve;
        if (useAve)
        {
            // 128 samples
            m_ina219->setResShunt(INA219_CONFIG_SADCRES_12BIT_128S_69MS);
            m_ina219->setResBus(INA219_CONFIG_BADCRES_12BIT_128S_69MS);
        }
        else
        {
            // 1 sample
            m_ina219->setResShunt(INA219_CONFIG_SADCRES_12BIT_1S_532US);
            m_ina219->setResBus(INA219_CONFIG_BADCRES_12BIT);
        }
    }
    void setCali_mA_to_mm(float m, float b)
    {
        m_mA_to_mm_m = m;
        m_mA_to_mm_b = b;
    }
    void powerSave(bool on) { m_ina219->powerSave(on); }

    // Getter
    float mA();
    float busV();
    // float kPa() { return mA() * m_mA_to_kPa_m + m_mA_to_kPa_b; }
    float mm() { return m_mm; }
    // float mm() { return m() * 1000; }
    float pc() { return m_pc; }
    float L() { return m_L; }
    bool connected() { return m_connected; }
    bool sensorOk() const { return m_sensorOk; }
    SensorFault sensorFault() const { return m_sensorFault; }

private:
    // float m_mA = 0;
    Adafruit_INA219 *m_ina219;
    // RunningAverage *m_mA;
    // RunningAverage *m_busV;
    float m_dia_mm = 0;
    float m_maxH_mm = 0;
    bool m_connected = false;
    float m_mA = 0;
    float m_mm = 0;
    float m_L = 0;
    float m_pc = 0;

    //Configuration
    bool m_useAve = true;
    uint16_t m_ina219ConfigB = INA219_CONFIG_BADCRES_12BIT_128S_69MS;
    uint16_t m_ina219ConfigS = INA219_CONFIG_SADCRES_12BIT_128S_69MS;
    void configReadout(bool useAve = true, bool useTrig = true);
    void triggerRead();

    // Sensor
    float sensorMax_kPa = 20;
    float sensorMin_kPa = 0;
    float sensorMax_mA = 20;
    float sensorMin_mA = 4;
    // Constants
    const float gravity_ms2 = 9.80665; // [m/s^-2]
    const float h2o_kgm3 = 1000;       // [kg/m^3]
    // Calibrations
    // TODO: store in EEPROM (actually RTC RAM!) and make updatable using MQTT
    float m_mA_to_mm_m = 128.5657757;         // y = mx + b
    float m_mA_to_mm_b = -516.7934059;        // y = mx + b
    float m_mA_to_mm_m_theory = 127.4645266;  // y = mx + b
    float m_mA_to_mm_b_theory = -509.8581065; // y = mx + b
    // float m_mA_to_kPa_m = (sensorMax_kPa - sensorMin_kPa) / (sensorMax_mA - sensorMin_mA);
    // float m_mA_to_kPa_b = (-m_mA_to_kPa_m * sensorMin_mA);

    // Conversions
    float sw = (h2o_kgm3 * gravity_ms2); // [N/m^3] Specific weight

    // Sensor health
    SensorFault m_sensorFault = SensorFault::NONE;
    bool m_sensorOk = true;
    float m_lastmA[SENSOR_STUCK_COUNT] = {0};
    uint8_t m_readIdx = 0;
    void evaluateHealth();
};

#endif // H_TANKS