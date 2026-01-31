// src/sensors/LD2420Sensor.cpp
#include "LD2420Sensor.h"
#include <Arduino.h>

// Static frame markers
constexpr uint8_t LD2420Sensor::HEADER[];
constexpr uint8_t LD2420Sensor::FOOTER[];

const SensorValueDescriptor LD2420Sensor::DESCRIPTORS[NUM_VALUES] = {
    {"ld_presence", "Present", "", 0.0f, 1.0f, Colors::PRESENCE_LIME, false, 0},
    {"ld_motion", "Motion", "", 0.0f, 1.0f, Colors::MOTION_RED, false, 0},
    {"ld_distance", "Dist", "cm", 0.0f, 600.0f, Colors::DIST_MAGENTA, true, 0}
};

LD2420Sensor::LD2420Sensor(HardwareSerial& serial, int8_t rxPin, int8_t txPin)
    : _serial(serial), _connected(false), _parseState(ParseState::WAIT_HEADER),
      _headerIndex(0), _footerIndex(0), _dataLength(0), _dataIndex(0) {

    _uartConfig.baudRate = BAUD_RATE;
    _uartConfig.rxPin = rxPin;
    _uartConfig.txPin = txPin;

    for (uint8_t i = 0; i < NUM_VALUES; i++) {
        _readings[i] = SensorReading::invalid(DESCRIPTORS[i].id);
    }
}

bool LD2420Sensor::begin() {
    _serial.begin(BAUD_RATE, SERIAL_8N1, _uartConfig.rxPin, _uartConfig.txPin);

    delay(100);

    // Clear buffer
    while (_serial.available()) {
        _serial.read();
    }

    resetParser();
    _connected = true;
    return _connected;
}

const SensorValueDescriptor* LD2420Sensor::getValueDescriptor(uint8_t index) const {
    if (index >= NUM_VALUES) return nullptr;
    return &DESCRIPTORS[index];
}

void LD2420Sensor::resetParser() {
    _parseState = ParseState::WAIT_HEADER;
    _headerIndex = 0;
    _footerIndex = 0;
    _dataLength = 0;
    _dataIndex = 0;
}

bool LD2420Sensor::read() {
    if (!_connected) {
        for (uint8_t i = 0; i < NUM_VALUES; i++) {
            _readings[i].valid = false;
        }
        return false;
    }

    // Process available bytes
    while (_serial.available()) {
        uint8_t byte = _serial.read();
        if (processByte(byte)) {
            // Complete frame received and parsed
            return true;
        }
    }

    return true;  // No complete frame yet, but not an error
}

bool LD2420Sensor::processByte(uint8_t byte) {
    switch (_parseState) {
        case ParseState::WAIT_HEADER:
            if (byte == HEADER[_headerIndex]) {
                _headerIndex++;
                if (_headerIndex >= 4) {
                    _parseState = ParseState::READ_LENGTH;
                    _dataIndex = 0;
                }
            } else {
                _headerIndex = (byte == HEADER[0]) ? 1 : 0;
            }
            break;

        case ParseState::READ_LENGTH:
            if (_dataIndex == 0) {
                _dataLength = byte;  // Low byte
                _dataIndex = 1;
            } else {
                _dataLength |= (byte << 8);  // High byte
                if (_dataLength > MAX_DATA_SIZE) {
                    resetParser();
                } else {
                    _parseState = ParseState::READ_DATA;
                    _dataIndex = 0;
                }
            }
            break;

        case ParseState::READ_DATA:
            if (_dataIndex < _dataLength && _dataIndex < MAX_DATA_SIZE) {
                _dataBuffer[_dataIndex++] = byte;
            }
            if (_dataIndex >= _dataLength) {
                _parseState = ParseState::WAIT_FOOTER;
                _footerIndex = 0;
            }
            break;

        case ParseState::WAIT_FOOTER:
            if (byte == FOOTER[_footerIndex]) {
                _footerIndex++;
                if (_footerIndex >= 4) {
                    // Complete frame received
                    bool success = parseData();
                    resetParser();
                    return success;
                }
            } else {
                // Invalid footer, reset
                resetParser();
            }
            break;
    }

    return false;
}

bool LD2420Sensor::parseData() {
    if (_dataLength < 4) return false;

    uint32_t now = millis();

    // LD2420 data format (simplified - actual format depends on mode):
    // Byte 0-1: Command type
    // Byte 2: Target status (0x00 = no target, 0x01 = moving, 0x02 = stationary, 0x03 = both)
    // Byte 3-4: Movement target distance (cm, little-endian)
    // Byte 5-6: Stationary target distance (cm, little-endian)

    uint8_t targetStatus = _dataBuffer[2];

    // Presence: any target detected
    bool presence = (targetStatus != 0x00);
    _readings[PRESENCE] = SensorReading::make(DESCRIPTORS[PRESENCE].id,
                                               presence ? 1.0f : 0.0f);
    _readings[PRESENCE].timestamp = now;

    // Motion: moving target detected
    bool moving = (targetStatus == 0x01 || targetStatus == 0x03);
    _readings[MOTION] = SensorReading::make(DESCRIPTORS[MOTION].id,
                                             moving ? 1.0f : 0.0f);
    _readings[MOTION].timestamp = now;

    // Distance: use movement distance if moving, else stationary distance
    uint16_t distance = 0;
    if (_dataLength >= 5) {
        if (moving && _dataLength >= 5) {
            distance = _dataBuffer[3] | (_dataBuffer[4] << 8);
        } else if (_dataLength >= 7) {
            distance = _dataBuffer[5] | (_dataBuffer[6] << 8);
        }
    }

    _readings[DISTANCE] = SensorReading::make(DESCRIPTORS[DISTANCE].id,
                                               (float)distance);
    _readings[DISTANCE].timestamp = now;

    return true;
}

SensorReading LD2420Sensor::getValue(uint8_t index) const {
    if (index >= NUM_VALUES) {
        return SensorReading::invalid("invalid");
    }
    return _readings[index];
}
