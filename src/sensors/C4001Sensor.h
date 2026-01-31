// src/sensors/C4001Sensor.h
#pragma once

#include "core/ISensor.h"
#include <HardwareSerial.h>

/**
 * C4001 mmWave Human Presence Sensor (UART interface)
 *
 * Detects human presence, distance, and motion state.
 * Default UART: 115200 baud, 8N1
 */
class C4001Sensor : public ISensor {
public:
    static constexpr const char* SENSOR_ID = "C4001";
    static constexpr const char* SENSOR_NAME = "Presence";
    static constexpr uint8_t NUM_VALUES = 3;

    // UART configuration
    static constexpr uint32_t BAUD_RATE = 115200;
    static constexpr int8_t DEFAULT_RX_PIN = 20;  // Adjust for your board
    static constexpr int8_t DEFAULT_TX_PIN = 21;  // Adjust for your board

    enum ValueIndex : uint8_t {
        PRESENCE = 0,   // 0 = no presence, 1 = presence detected
        DISTANCE = 1,   // Distance to target in cm
        MOTION = 2      // 0 = stationary, 1 = approaching, 2 = receding
    };

    C4001Sensor(HardwareSerial& serial, int8_t rxPin = DEFAULT_RX_PIN,
                int8_t txPin = DEFAULT_TX_PIN);

    // ISensor implementation
    const char* getSensorId() const override { return SENSOR_ID; }
    const char* getSensorName() const override { return SENSOR_NAME; }
    SensorInterface getInterfaceType() const override { return SensorInterface::UART; }
    UARTConfig getUARTConfig() const override { return _uartConfig; }

    bool begin() override;
    bool isConnected() const override { return _connected; }

    uint8_t getValueCount() const override { return NUM_VALUES; }
    const SensorValueDescriptor* getValueDescriptor(uint8_t index) const override;

    bool read() override;
    SensorReading getValue(uint8_t index) const override;

private:
    HardwareSerial& _serial;
    UARTConfig _uartConfig;
    SensorReading _readings[NUM_VALUES];
    bool _connected;

    // Receive buffer
    static constexpr uint8_t RX_BUFFER_SIZE = 64;
    uint8_t _rxBuffer[RX_BUFFER_SIZE];
    uint8_t _rxIndex;

    // Parse incoming data frame
    bool parseFrame(const uint8_t* data, uint8_t len);

    static const SensorValueDescriptor DESCRIPTORS[NUM_VALUES];
};
