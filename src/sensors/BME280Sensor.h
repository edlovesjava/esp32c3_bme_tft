// src/sensors/BME280Sensor.h
#pragma once

#include "core/ISensor.h"
#include <Adafruit_BME280.h>

class BME280Sensor : public ISensor {
public:
    static constexpr const char* SENSOR_ID = "BME280";
    static constexpr const char* SENSOR_NAME = "Environmental";
    static constexpr uint8_t I2C_ADDR = 0x76;
    static constexpr uint8_t NUM_VALUES = 3;

    enum ValueIndex : uint8_t {
        TEMPERATURE = 0,
        HUMIDITY = 1,
        PRESSURE = 2
    };

    BME280Sensor();

    const char* getSensorId() const override { return SENSOR_ID; }
    const char* getSensorName() const override { return SENSOR_NAME; }
    SensorInterface getInterfaceType() const override { return SensorInterface::I2C; }
    uint8_t getI2CAddress() const override { return I2C_ADDR; }

    bool begin() override;
    bool isConnected() const override { return _connected; }

    uint8_t getValueCount() const override { return NUM_VALUES; }
    const SensorValueDescriptor* getValueDescriptor(uint8_t index) const override;

    bool read() override;
    SensorReading getValue(uint8_t index) const override;

private:
    Adafruit_BME280 _bme;
    SensorReading _readings[NUM_VALUES];
    bool _connected;

    static const SensorValueDescriptor DESCRIPTORS[NUM_VALUES];
};
