// src/core/SensorTypes.h
#pragma once

#include <stdint.h>
#include <Arduino.h>

/**
 * Sensor communication interface types.
 */
enum class SensorInterface : uint8_t {
    I2C,
    UART,
    SPI,
    GPIO,
    VIRTUAL  // For mock/software sensors
};

/**
 * UART configuration for serial sensors.
 */
struct UARTConfig {
    uint32_t baudRate;
    int8_t rxPin;
    int8_t txPin;

    static UARTConfig none() {
        return {0, -1, -1};
    }
};

/**
 * Analog/GPIO configuration for ADC-based sensors.
 */
struct AnalogConfig {
    int8_t pins[6];      // Up to 6 analog pins (-1 = unused)
    uint8_t pinCount;    // Number of pins used
    uint8_t resolution;  // ADC resolution in bits (default 12)
    float vRef;          // Reference voltage (default 3.3V)

    static AnalogConfig none() {
        return {{-1, -1, -1, -1, -1, -1}, 0, 12, 3.3f};
    }
};

/**
 * Describes a single value exposed by a sensor.
 */
struct SensorValueDescriptor {
    const char* id;           // Unique identifier: "temp", "co2"
    const char* name;         // Display name: "Temp", "CO2"
    const char* unit;         // Unit: "C", "ppm", "%"
    float minValue;           // Min for display scaling
    float maxValue;           // Max for display scaling
    uint16_t displayColor;    // RGB565 color
    bool showGraph;           // Include in trend graph
    uint8_t precision;        // Decimal places (0-2)
};

/**
 * A single sensor reading.
 */
struct SensorReading {
    const char* valueId;
    float value;
    bool valid;
    uint32_t timestamp;

    static SensorReading invalid(const char* id) {
        return {id, 0.0f, false, 0};
    }

    static SensorReading make(const char* id, float val) {
        return {id, val, true, millis()};
    }
};

/**
 * Common color palette (RGB565)
 */
namespace Colors {
    constexpr uint16_t TEMP_ORANGE   = 0xFD20;
    constexpr uint16_t HUMID_CYAN    = 0x07FF;
    constexpr uint16_t PRESS_GREEN   = 0x07E0;
    constexpr uint16_t CO2_YELLOW    = 0xFFE0;
    constexpr uint16_t LIGHT_WHITE   = 0xFFFF;
    constexpr uint16_t DIST_MAGENTA  = 0xF81F;
    constexpr uint16_t MOTION_RED    = 0xF800;
    constexpr uint16_t ACCEL_BLUE    = 0x001F;
    constexpr uint16_t PRESENCE_LIME = 0x87E0;
    constexpr uint16_t LABEL_GRAY    = 0x8410;
    constexpr uint16_t GRAPH_BG      = 0x1082;
    constexpr uint16_t BACKGROUND    = 0x0000;
}
