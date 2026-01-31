// src/sensors/MockSensor.h
#pragma once

#include "core/ISensor.h"

class MockSensor : public ISensor {
public:
    static constexpr const char* SENSOR_ID = "MOCK";
    static constexpr const char* SENSOR_NAME = "Test Sensor";
    static constexpr uint8_t NUM_VALUES = 2;

    MockSensor();

    const char* getSensorId() const override { return SENSOR_ID; }
    const char* getSensorName() const override { return SENSOR_NAME; }
    SensorInterface getInterfaceType() const override { return SensorInterface::VIRTUAL; }

    bool begin() override { _connected = true; return true; }
    bool isConnected() const override { return _connected; }

    uint8_t getValueCount() const override { return NUM_VALUES; }
    const SensorValueDescriptor* getValueDescriptor(uint8_t index) const override;

    bool read() override;
    SensorReading getValue(uint8_t index) const override;

private:
    SensorReading _readings[NUM_VALUES];
    bool _connected;
    float _counter;

    static const SensorValueDescriptor DESCRIPTORS[NUM_VALUES];
};
