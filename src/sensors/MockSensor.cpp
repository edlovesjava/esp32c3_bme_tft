// src/sensors/MockSensor.cpp
#include "MockSensor.h"
#include <Arduino.h>

const SensorValueDescriptor MockSensor::DESCRIPTORS[NUM_VALUES] = {
    {"mock_sin", "Wave", "", -1.0f, 1.0f, 0xF81F, true, 2},
    {"mock_cnt", "Count", "", 0.0f, 100.0f, 0xFFE0, false, 0}
};

MockSensor::MockSensor() : _connected(false), _counter(0) {
    for (uint8_t i = 0; i < NUM_VALUES; i++) {
        _readings[i] = SensorReading::invalid(DESCRIPTORS[i].id);
    }
}

const SensorValueDescriptor* MockSensor::getValueDescriptor(uint8_t index) const {
    if (index >= NUM_VALUES) return nullptr;
    return &DESCRIPTORS[index];
}

bool MockSensor::read() {
    if (!_connected) return false;

    uint32_t now = millis();
    _counter += 0.1f;
    if (_counter > 100.0f) _counter = 0;

    _readings[0] = SensorReading::make(DESCRIPTORS[0].id, sin(_counter));
    _readings[0].timestamp = now;

    _readings[1] = SensorReading::make(DESCRIPTORS[1].id, _counter);
    _readings[1].timestamp = now;

    return true;
}

SensorReading MockSensor::getValue(uint8_t index) const {
    if (index >= NUM_VALUES) return SensorReading::invalid("invalid");
    return _readings[index];
}
