// src/core/ISensor.h
#pragma once

#include "SensorTypes.h"

/**
 * Abstract interface for all sensors (I2C, UART, SPI, etc).
 */
class ISensor {
public:
    virtual ~ISensor() = default;

    // Identification
    virtual const char* getSensorId() const = 0;
    virtual const char* getSensorName() const = 0;

    // Interface type
    virtual SensorInterface getInterfaceType() const = 0;

    // I2C configuration (return 0 for non-I2C sensors)
    virtual uint8_t getI2CAddress() const { return 0; }

    // UART configuration (override for UART sensors)
    virtual UARTConfig getUARTConfig() const { return UARTConfig::none(); }

    // Analog/GPIO configuration (override for ADC-based sensors)
    virtual AnalogConfig getAnalogConfig() const { return AnalogConfig::none(); }

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
