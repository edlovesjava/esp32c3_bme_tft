// src/sensors/QYF0900Sensor.cpp
#include "QYF0900Sensor.h"
#include <Arduino.h>

const SensorValueDescriptor QYF0900Sensor::DESCRIPTORS[NUM_VALUES] = {
    {"accel_x", "X", "g", -3.0f, 3.0f, Colors::ACCEL_BLUE, true, 2},
    {"accel_y", "Y", "g", -3.0f, 3.0f, Colors::MOTION_RED, true, 2},
    {"accel_z", "Z", "g", -3.0f, 3.0f, Colors::PRESS_GREEN, true, 2}
};

QYF0900Sensor::QYF0900Sensor(int8_t xPin, int8_t yPin, int8_t zPin)
    : _connected(false), _zeroGVoltage(ZERO_G_VOLTAGE), _sensitivity(SENSITIVITY) {

    _analogConfig.pins[0] = xPin;
    _analogConfig.pins[1] = yPin;
    _analogConfig.pins[2] = zPin;
    _analogConfig.pins[3] = -1;
    _analogConfig.pins[4] = -1;
    _analogConfig.pins[5] = -1;
    _analogConfig.pinCount = 3;
    _analogConfig.resolution = ADC_BITS;
    _analogConfig.vRef = VREF;

    for (uint8_t i = 0; i < NUM_VALUES; i++) {
        _readings[i] = SensorReading::invalid(DESCRIPTORS[i].id);
    }
}

bool QYF0900Sensor::begin() {
    // Configure ADC pins as inputs
    for (uint8_t i = 0; i < _analogConfig.pinCount; i++) {
        if (_analogConfig.pins[i] >= 0) {
            pinMode(_analogConfig.pins[i], INPUT);
        }
    }

    // Set ADC resolution (ESP32)
    analogReadResolution(_analogConfig.resolution);

    // Perform a test read to verify sensor is responding
    // Analog sensors don't have a connection check, so we assume connected
    // if pins are configured
    _connected = (_analogConfig.pins[0] >= 0 &&
                  _analogConfig.pins[1] >= 0 &&
                  _analogConfig.pins[2] >= 0);

    return _connected;
}

const SensorValueDescriptor* QYF0900Sensor::getValueDescriptor(uint8_t index) const {
    if (index >= NUM_VALUES) return nullptr;
    return &DESCRIPTORS[index];
}

float QYF0900Sensor::adcToG(uint16_t adcValue) const {
    // Convert ADC value to voltage
    float maxAdcValue = (1 << _analogConfig.resolution) - 1;
    float voltage = (adcValue / maxAdcValue) * _analogConfig.vRef;

    // Convert voltage to g value
    // g = (voltage - zeroGVoltage) / sensitivity
    return (voltage - _zeroGVoltage) / _sensitivity;
}

bool QYF0900Sensor::read() {
    if (!_connected) {
        for (uint8_t i = 0; i < NUM_VALUES; i++) {
            _readings[i].valid = false;
        }
        return false;
    }

    uint32_t now = millis();

    // Read X axis
    uint16_t xRaw = analogRead(_analogConfig.pins[0]);
    _readings[ACCEL_X] = SensorReading::make(DESCRIPTORS[ACCEL_X].id, adcToG(xRaw));
    _readings[ACCEL_X].timestamp = now;

    // Read Y axis
    uint16_t yRaw = analogRead(_analogConfig.pins[1]);
    _readings[ACCEL_Y] = SensorReading::make(DESCRIPTORS[ACCEL_Y].id, adcToG(yRaw));
    _readings[ACCEL_Y].timestamp = now;

    // Read Z axis
    uint16_t zRaw = analogRead(_analogConfig.pins[2]);
    _readings[ACCEL_Z] = SensorReading::make(DESCRIPTORS[ACCEL_Z].id, adcToG(zRaw));
    _readings[ACCEL_Z].timestamp = now;

    return true;
}

SensorReading QYF0900Sensor::getValue(uint8_t index) const {
    if (index >= NUM_VALUES) {
        return SensorReading::invalid("invalid");
    }
    return _readings[index];
}
