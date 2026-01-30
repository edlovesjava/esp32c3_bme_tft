// src/core/SensorTypes.h
#pragma once

#include <stdint.h>
#include <Arduino.h>

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
    constexpr uint16_t TEMP_ORANGE  = 0xFD20;
    constexpr uint16_t HUMID_CYAN   = 0x07FF;
    constexpr uint16_t PRESS_GREEN  = 0x07E0;
    constexpr uint16_t CO2_YELLOW   = 0xFFE0;
    constexpr uint16_t LABEL_GRAY   = 0x8410;
    constexpr uint16_t GRAPH_BG     = 0x1082;
    constexpr uint16_t BACKGROUND   = 0x0000;
}
