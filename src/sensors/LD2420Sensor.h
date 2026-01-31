// src/sensors/LD2420Sensor.h
#pragma once

#include "core/ISensor.h"
#include <HardwareSerial.h>

/**
 * LD2420 24GHz mmWave Radar Sensor (Hi-Link)
 *
 * Human presence and motion detection with distance measurement.
 * UART interface: 115200 baud (default), 8N1
 *
 * Frame format:
 *   Header: 0xFD 0xFC 0xFB 0xFA
 *   Length: 2 bytes (little-endian)
 *   Data:   Variable
 *   Footer: 0x04 0x03 0x02 0x01
 */
class LD2420Sensor : public ISensor {
public:
    static constexpr const char* SENSOR_ID = "LD2420";
    static constexpr const char* SENSOR_NAME = "Radar";
    static constexpr uint8_t NUM_VALUES = 3;

    // UART configuration
    static constexpr uint32_t BAUD_RATE = 115200;
    static constexpr int8_t DEFAULT_RX_PIN = 20;
    static constexpr int8_t DEFAULT_TX_PIN = 21;

    // Frame markers
    static constexpr uint8_t HEADER[] = {0xFD, 0xFC, 0xFB, 0xFA};
    static constexpr uint8_t FOOTER[] = {0x04, 0x03, 0x02, 0x01};

    enum ValueIndex : uint8_t {
        PRESENCE = 0,    // 0 = no one, 1 = presence detected
        MOTION = 1,      // 0 = stationary, 1 = moving
        DISTANCE = 2     // Distance in cm (0-600)
    };

    LD2420Sensor(HardwareSerial& serial, int8_t rxPin = DEFAULT_RX_PIN,
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

    // Frame parsing state machine
    enum class ParseState : uint8_t {
        WAIT_HEADER,
        READ_LENGTH,
        READ_DATA,
        WAIT_FOOTER
    };

    ParseState _parseState;
    uint8_t _headerIndex;
    uint8_t _footerIndex;
    uint16_t _dataLength;
    uint16_t _dataIndex;

    static constexpr uint8_t MAX_DATA_SIZE = 64;
    uint8_t _dataBuffer[MAX_DATA_SIZE];

    void resetParser();
    bool processByte(uint8_t byte);
    bool parseData();

    static const SensorValueDescriptor DESCRIPTORS[NUM_VALUES];
};
