// src/sensors/C4001Sensor.cpp
#include "C4001Sensor.h"
#include <Arduino.h>

const SensorValueDescriptor C4001Sensor::DESCRIPTORS[NUM_VALUES] = {
    {"presence", "Present", "", 0.0f, 1.0f, Colors::PRESENCE_LIME, false, 0},
    {"distance", "Dist", "cm", 0.0f, 500.0f, Colors::DIST_MAGENTA, true, 0},
    {"motion", "Motion", "", 0.0f, 2.0f, Colors::MOTION_RED, false, 0}
};

C4001Sensor::C4001Sensor(HardwareSerial& serial, int8_t rxPin, int8_t txPin)
    : _serial(serial), _connected(false), _rxIndex(0) {
    _uartConfig.baudRate = BAUD_RATE;
    _uartConfig.rxPin = rxPin;
    _uartConfig.txPin = txPin;

    for (uint8_t i = 0; i < NUM_VALUES; i++) {
        _readings[i] = SensorReading::invalid(DESCRIPTORS[i].id);
    }
}

bool C4001Sensor::begin() {
    // Initialize UART with specified pins
    _serial.begin(BAUD_RATE, SERIAL_8N1, _uartConfig.rxPin, _uartConfig.txPin);

    // Wait for sensor to initialize
    delay(100);

    // Clear any pending data
    while (_serial.available()) {
        _serial.read();
    }

    // TODO: Send initialization command if required by C4001
    // Some mmWave sensors need configuration commands

    _connected = true;
    return _connected;
}

const SensorValueDescriptor* C4001Sensor::getValueDescriptor(uint8_t index) const {
    if (index >= NUM_VALUES) return nullptr;
    return &DESCRIPTORS[index];
}

bool C4001Sensor::read() {
    if (!_connected) {
        for (uint8_t i = 0; i < NUM_VALUES; i++) {
            _readings[i].valid = false;
        }
        return false;
    }

    // Read available data into buffer
    while (_serial.available() && _rxIndex < RX_BUFFER_SIZE) {
        uint8_t byte = _serial.read();
        _rxBuffer[_rxIndex++] = byte;

        // Check for frame end (adjust based on C4001 protocol)
        // Many mmWave sensors use 0x55 0x55 or similar frame markers
        if (_rxIndex >= 2 && _rxBuffer[_rxIndex - 1] == 0x55 &&
            _rxBuffer[_rxIndex - 2] == 0x55) {
            // Parse complete frame
            if (parseFrame(_rxBuffer, _rxIndex)) {
                _rxIndex = 0;
                return true;
            }
            _rxIndex = 0;
        }
    }

    // Buffer overflow protection
    if (_rxIndex >= RX_BUFFER_SIZE) {
        _rxIndex = 0;
    }

    return true;  // No complete frame yet, but not an error
}

bool C4001Sensor::parseFrame(const uint8_t* data, uint8_t len) {
    // TODO: Implement actual C4001 protocol parsing
    // This is a placeholder showing the expected structure

    // Example frame structure (varies by manufacturer):
    // [Header 0x55][Header 0xAA][Type][Length][Data...][Checksum][Footer 0x55][Footer 0x55]

    if (len < 8) return false;  // Minimum frame length

    uint32_t now = millis();

    // Extract presence (placeholder - adjust for actual protocol)
    // Typically presence is a single bit or byte
    bool presence = (data[4] & 0x01) != 0;
    _readings[PRESENCE] = SensorReading::make(DESCRIPTORS[PRESENCE].id,
                                               presence ? 1.0f : 0.0f);
    _readings[PRESENCE].timestamp = now;

    // Extract distance (placeholder - adjust for actual protocol)
    // Typically 2 bytes, little-endian or big-endian
    uint16_t distRaw = (data[5] << 8) | data[6];
    float distCm = distRaw / 10.0f;  // Often in mm, convert to cm
    _readings[DISTANCE] = SensorReading::make(DESCRIPTORS[DISTANCE].id, distCm);
    _readings[DISTANCE].timestamp = now;

    // Extract motion state (placeholder - adjust for actual protocol)
    // 0 = stationary, 1 = approaching, 2 = receding
    uint8_t motion = data[7] & 0x03;
    _readings[MOTION] = SensorReading::make(DESCRIPTORS[MOTION].id, (float)motion);
    _readings[MOTION].timestamp = now;

    return true;
}

SensorReading C4001Sensor::getValue(uint8_t index) const {
    if (index >= NUM_VALUES) {
        return SensorReading::invalid("invalid");
    }
    return _readings[index];
}
