// src/sensors/BME280Sensor.cpp
#include "BME280Sensor.h"
#include <Arduino.h>

const SensorValueDescriptor BME280Sensor::DESCRIPTORS[NUM_VALUES] = {
    {"bme_temp", "Temp", "C", -10.0f, 50.0f, Colors::TEMP_ORANGE, true, 1},
    {"bme_humid", "Humid", "%", 0.0f, 100.0f, Colors::HUMID_CYAN, false, 0},
    {"bme_press", "Press", "hPa", 950.0f, 1050.0f, Colors::PRESS_GREEN, false, 0}
};

BME280Sensor::BME280Sensor() : _connected(false) {
    for (uint8_t i = 0; i < NUM_VALUES; i++) {
        _readings[i] = SensorReading::invalid(DESCRIPTORS[i].id);
    }
}

bool BME280Sensor::begin() {
    _connected = _bme.begin(I2C_ADDR);
    if (_connected) {
        _bme.setSampling(
            Adafruit_BME280::MODE_FORCED,
            Adafruit_BME280::SAMPLING_X1,
            Adafruit_BME280::SAMPLING_X1,
            Adafruit_BME280::SAMPLING_X1,
            Adafruit_BME280::FILTER_OFF
        );
    }
    return _connected;
}

const SensorValueDescriptor* BME280Sensor::getValueDescriptor(uint8_t index) const {
    if (index >= NUM_VALUES) return nullptr;
    return &DESCRIPTORS[index];
}

bool BME280Sensor::read() {
    if (!_connected) {
        for (uint8_t i = 0; i < NUM_VALUES; i++) {
            _readings[i].valid = false;
        }
        return false;
    }

    _bme.takeForcedMeasurement();
    uint32_t now = millis();

    _readings[TEMPERATURE] = SensorReading::make(
        DESCRIPTORS[TEMPERATURE].id, _bme.readTemperature());
    _readings[TEMPERATURE].timestamp = now;

    _readings[HUMIDITY] = SensorReading::make(
        DESCRIPTORS[HUMIDITY].id, _bme.readHumidity());
    _readings[HUMIDITY].timestamp = now;

    _readings[PRESSURE] = SensorReading::make(
        DESCRIPTORS[PRESSURE].id, _bme.readPressure() / 100.0f);
    _readings[PRESSURE].timestamp = now;

    return true;
}

SensorReading BME280Sensor::getValue(uint8_t index) const {
    if (index >= NUM_VALUES) {
        return SensorReading::invalid("invalid");
    }
    return _readings[index];
}
