// src/core/ISensor.h
#pragma once

#include "SensorTypes.h"

/**
 * Abstract interface for all I2C sensors.
 */
class ISensor {
public:
    virtual ~ISensor() = default;

    // Identification
    virtual const char* getSensorId() const = 0;
    virtual const char* getSensorName() const = 0;
    virtual uint8_t getI2CAddress() const = 0;

    // Lifecycle
    virtual bool begin() = 0;
    virtual bool isConnected() const = 0;

    // Value descriptors
    virtual uint8_t getValueCount() const = 0;
    virtual const SensorValueDescriptor* getValueDescriptor(uint8_t index) const = 0;

    // Data acquisition
    virtual bool read() = 0;
    virtual SensorReading getValue(uint8_t index) const = 0;
};
