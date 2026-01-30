// src/core/SensorRegistry.h
#pragma once

#include "ISensor.h"
#include "Config.h"

class SensorRegistry {
public:
    SensorRegistry();

    bool registerSensor(ISensor* sensor);
    bool initializeAll();
    void pollAll();

    uint8_t getTotalValueCount() const;
    SensorReading getReading(uint8_t globalIndex) const;
    const SensorValueDescriptor* getDescriptor(uint8_t globalIndex) const;

    uint8_t getSensorCount() const;
    bool isSensorConnected(uint8_t sensorIndex) const;

private:
    struct ValueMapping {
        uint8_t sensorIndex;
        uint8_t valueIndex;
    };

    ISensor* _sensors[MAX_SENSORS];
    uint8_t _sensorCount;
    ValueMapping _valueMap[MAX_DISPLAY_VALUES];
    uint8_t _totalValues;

    void buildValueMap();
};
