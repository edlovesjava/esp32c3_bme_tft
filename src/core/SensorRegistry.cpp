// src/core/SensorRegistry.cpp
#include "SensorRegistry.h"

SensorRegistry::SensorRegistry()
    : _sensorCount(0), _totalValues(0) {
    for (uint8_t i = 0; i < MAX_SENSORS; i++) {
        _sensors[i] = nullptr;
    }
}

bool SensorRegistry::registerSensor(ISensor* sensor) {
    if (sensor == nullptr || _sensorCount >= MAX_SENSORS) {
        return false;
    }
    _sensors[_sensorCount++] = sensor;
    return true;
}

bool SensorRegistry::initializeAll() {
    bool allOk = true;
    for (uint8_t i = 0; i < _sensorCount; i++) {
        if (!_sensors[i]->begin()) {
            allOk = false;
        }
    }
    buildValueMap();
    return allOk;
}

void SensorRegistry::pollAll() {
    for (uint8_t i = 0; i < _sensorCount; i++) {
        if (_sensors[i]->isConnected()) {
            _sensors[i]->read();
        }
    }
}

void SensorRegistry::buildValueMap() {
    _totalValues = 0;
    for (uint8_t s = 0; s < _sensorCount && _totalValues < MAX_DISPLAY_VALUES; s++) {
        if (!_sensors[s]->isConnected()) continue;
        uint8_t count = _sensors[s]->getValueCount();
        for (uint8_t v = 0; v < count && _totalValues < MAX_DISPLAY_VALUES; v++) {
            _valueMap[_totalValues].sensorIndex = s;
            _valueMap[_totalValues].valueIndex = v;
            _totalValues++;
        }
    }
}

uint8_t SensorRegistry::getTotalValueCount() const {
    return _totalValues;
}

SensorReading SensorRegistry::getReading(uint8_t globalIndex) const {
    if (globalIndex >= _totalValues) {
        return SensorReading::invalid("invalid");
    }
    const ValueMapping& m = _valueMap[globalIndex];
    return _sensors[m.sensorIndex]->getValue(m.valueIndex);
}

const SensorValueDescriptor* SensorRegistry::getDescriptor(uint8_t globalIndex) const {
    if (globalIndex >= _totalValues) {
        return nullptr;
    }
    const ValueMapping& m = _valueMap[globalIndex];
    return _sensors[m.sensorIndex]->getValueDescriptor(m.valueIndex);
}

uint8_t SensorRegistry::getSensorCount() const {
    return _sensorCount;
}

bool SensorRegistry::isSensorConnected(uint8_t sensorIndex) const {
    if (sensorIndex >= _sensorCount) return false;
    return _sensors[sensorIndex]->isConnected();
}
