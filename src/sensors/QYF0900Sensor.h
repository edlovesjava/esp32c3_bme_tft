// src/sensors/QYF0900Sensor.h
#pragma once

#include "core/ISensor.h"

/**
 * QYF0900 3-Axis Analog Accelerometer
 *
 * Analog output accelerometer with separate X, Y, Z voltage outputs.
 * Each axis outputs a voltage proportional to acceleration.
 * Typical: 0g = Vcc/2, sensitivity ~300-330mV/g
 *
 * Pins: VCC, X-OUT, Y-OUT, Z-OUT, GND
 */
class QYF0900Sensor : public ISensor {
public:
    static constexpr const char* SENSOR_ID = "QYF0900";
    static constexpr const char* SENSOR_NAME = "Accel";
    static constexpr uint8_t NUM_VALUES = 3;

    // Default ADC pins for ESP32-C3 (adjust as needed)
    static constexpr int8_t DEFAULT_X_PIN = 2;  // GPIO2 (ADC1_CH2)
    static constexpr int8_t DEFAULT_Y_PIN = 3;  // GPIO3 (ADC1_CH3)
    static constexpr int8_t DEFAULT_Z_PIN = 4;  // GPIO4 (ADC1_CH4)

    // Calibration constants (adjust for your sensor)
    static constexpr float VREF = 3.3f;           // Reference voltage
    static constexpr float ZERO_G_VOLTAGE = 1.65f; // Voltage at 0g (typically Vcc/2)
    static constexpr float SENSITIVITY = 0.330f;   // V/g (330mV/g typical)
    static constexpr uint8_t ADC_BITS = 12;        // ESP32 ADC resolution

    enum ValueIndex : uint8_t {
        ACCEL_X = 0,
        ACCEL_Y = 1,
        ACCEL_Z = 2
    };

    QYF0900Sensor(int8_t xPin = DEFAULT_X_PIN,
                  int8_t yPin = DEFAULT_Y_PIN,
                  int8_t zPin = DEFAULT_Z_PIN);

    // ISensor implementation
    const char* getSensorId() const override { return SENSOR_ID; }
    const char* getSensorName() const override { return SENSOR_NAME; }
    SensorInterface getInterfaceType() const override { return SensorInterface::GPIO; }
    AnalogConfig getAnalogConfig() const override { return _analogConfig; }

    bool begin() override;
    bool isConnected() const override { return _connected; }

    uint8_t getValueCount() const override { return NUM_VALUES; }
    const SensorValueDescriptor* getValueDescriptor(uint8_t index) const override;

    bool read() override;
    SensorReading getValue(uint8_t index) const override;

    // Calibration methods
    void setZeroGVoltage(float voltage) { _zeroGVoltage = voltage; }
    void setSensitivity(float mvPerG) { _sensitivity = mvPerG; }

private:
    AnalogConfig _analogConfig;
    SensorReading _readings[NUM_VALUES];
    bool _connected;

    float _zeroGVoltage;
    float _sensitivity;

    float adcToG(uint16_t adcValue) const;

    static const SensorValueDescriptor DESCRIPTORS[NUM_VALUES];
};
