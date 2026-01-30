# Modular Sensor Architecture - Detailed Design

**Version:** 1.0
**Date:** 2026-01-30
**Parent:** modular-sensor-architecture-spec.md

---

## 1. Introduction

This document provides implementation details for the modular sensor architecture. It expands on the specification with concrete class definitions, memory layouts, and implementation guidance.

---

## 2. Class Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              <<struct>>                                     │
│                          SensorValueDescriptor                              │
├─────────────────────────────────────────────────────────────────────────────┤
│ + id: const char*                                                           │
│ + name: const char*                                                         │
│ + unit: const char*                                                         │
│ + minValue: float                                                           │
│ + maxValue: float                                                           │
│ + displayColor: uint16_t                                                    │
│ + showGraph: bool                                                           │
│ + precision: uint8_t                                                        │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│                              <<struct>>                                     │
│                            SensorReading                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│ + valueId: const char*                                                      │
│ + value: float                                                              │
│ + valid: bool                                                               │
│ + timestamp: uint32_t                                                       │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│                            <<interface>>                                    │
│                               ISensor                                       │
├─────────────────────────────────────────────────────────────────────────────┤
│ + getSensorId(): const char*                                                │
│ + getSensorName(): const char*                                              │
│ + getI2CAddress(): uint8_t                                                  │
│ + begin(): bool                                                             │
│ + isConnected(): bool                                                       │
│ + getValueCount(): uint8_t                                                  │
│ + getValueDescriptor(index: uint8_t): const SensorValueDescriptor*          │
│ + read(): bool                                                              │
│ + getValue(index: uint8_t): SensorReading                                   │
└─────────────────────────────────────────────────────────────────────────────┘
                                     △
                                     │ implements
           ┌─────────────────────────┼─────────────────────────┐
           │                         │                         │
┌──────────┴──────────┐  ┌──────────┴──────────┐  ┌──────────┴──────────┐
│    BME280Sensor     │  │    SCD40Sensor      │  │    SGP30Sensor      │
├─────────────────────┤  ├─────────────────────┤  ├─────────────────────┤
│ - _bme: BME280      │  │ - _scd: SCD4x       │  │ - _sgp: SGP30       │
│ - _readings[3]      │  │ - _readings[3]      │  │ - _readings[2]      │
│ - _connected: bool  │  │ - _connected: bool  │  │ - _connected: bool  │
├─────────────────────┤  ├─────────────────────┤  ├─────────────────────┤
│ + begin()           │  │ + begin()           │  │ + begin()           │
│ + read()            │  │ + read()            │  │ + read()            │
│ ...                 │  │ ...                 │  │ ...                 │
└─────────────────────┘  └─────────────────────┘  └─────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│                           SensorRegistry                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│ - _sensors[MAX_SENSORS]: ISensor*                                           │
│ - _sensorCount: uint8_t                                                     │
│ - _valueMap[MAX_VALUES]: ValueMapping                                       │
│ - _totalValues: uint8_t                                                     │
├─────────────────────────────────────────────────────────────────────────────┤
│ + registerSensor(sensor: ISensor*): bool                                    │
│ + initializeAll(): bool                                                     │
│ + pollAll(): void                                                           │
│ + getTotalValueCount(): uint8_t                                             │
│ + getReading(globalIndex: uint8_t): SensorReading                           │
│ + getDescriptor(globalIndex: uint8_t): const SensorValueDescriptor*         │
│ + isSensorConnected(sensorIndex: uint8_t): bool                             │
└─────────────────────────────────────────────────────────────────────────────┘
                              │
                              │ uses
                              ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                           DisplayManager                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│ - _tft: Adafruit_ST7735*                                                    │
│ - _layout: LayoutMode                                                       │
│ - _history[MAX_VALUES]: ValueBuffer                                         │
│ - _graphValues: uint8_t (bitmask)                                           │
├─────────────────────────────────────────────────────────────────────────────┤
│ + begin(tft: Adafruit_ST7735*): void                                        │
│ + setLayout(mode: LayoutMode): void                                         │
│ + setGraphValues(mask: uint8_t): void                                       │
│ + render(readings, descriptors, count): void                                │
│ + updateHistory(readings, count): void                                      │
│ - renderValueCell(slot, reading, desc): void                                │
│ - renderGraph(): void                                                       │
│ - renderCompactValue(slot, reading, desc): void                             │
└─────────────────────────────────────────────────────────────────────────────┘
                              │
                              │ uses
                              ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                            ValueBuffer                                      │
├─────────────────────────────────────────────────────────────────────────────┤
│ - _buffer[HISTORY_SIZE]: float                                              │
│ - _index: uint8_t                                                           │
│ - _filled: bool                                                             │
│ - _min: float                                                               │
│ - _max: float                                                               │
├─────────────────────────────────────────────────────────────────────────────┤
│ + push(value: float): void                                                  │
│ + get(age: uint8_t): float                                                  │
│ + getCount(): uint8_t                                                       │
│ + getMin(): float                                                           │
│ + getMax(): float                                                           │
│ + getTrend(): int8_t  // -1, 0, +1                                          │
│ + clear(): void                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. Memory Budget

### 3.1 Per-Component Estimates

| Component | RAM (bytes) | Flash (bytes) | Notes |
|-----------|-------------|---------------|-------|
| **SensorValueDescriptor** | 0 (const) | 24 | Stored in flash |
| **SensorReading** | 12 | 0 | Per-value runtime |
| **ValueBuffer** | 204 | 100 | 50 floats + state |
| **BME280Sensor** | 48 | 800 | Adafruit object + readings |
| **SCD40Sensor** | 52 | 900 | Sensirion object + readings |
| **SensorRegistry** | 64 | 600 | Pointers + mapping |
| **DisplayManager** | 1300 | 2000 | 6 ValueBuffers + state |

### 3.2 Total Memory (4 sensors, 6 values)

| Category | Estimate |
|----------|----------|
| **Static RAM** | ~1.8 KB |
| **Flash (code)** | ~8 KB (excluding libraries) |
| **Heap** | 0 (no dynamic allocation) |

ESP32-C3 has 400KB SRAM, 4MB Flash - well within budget.

---

## 4. Detailed Interface Definitions

### 4.1 SensorTypes.h

```cpp
// src/core/SensorTypes.h
#pragma once

#include <stdint.h>

/**
 * Describes a single value exposed by a sensor.
 * Stored in flash (PROGMEM) to save RAM.
 */
struct SensorValueDescriptor {
    const char* id;           // Unique identifier: "temp", "co2", "tvoc"
    const char* name;         // Human-readable: "Temperature", "CO2"
    const char* unit;         // Display unit: "°C", "ppm", "%"
    float minValue;           // Minimum expected value (for display scaling)
    float maxValue;           // Maximum expected value (for display scaling)
    uint16_t displayColor;    // RGB565 color for display
    bool showGraph;           // Include in composite graph
    uint8_t precision;        // Decimal places (0-2)
};

/**
 * A single sensor reading with metadata.
 */
struct SensorReading {
    const char* valueId;      // Matches SensorValueDescriptor.id
    float value;              // The actual reading
    bool valid;               // False if sensor error
    uint32_t timestamp;       // millis() of reading

    // Convenience constructor
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
    constexpr uint16_t TEMP_ORANGE = 0xFD20;
    constexpr uint16_t HUMID_CYAN  = 0x07FF;
    constexpr uint16_t PRESS_GREEN = 0x07E0;
    constexpr uint16_t CO2_YELLOW  = 0xFFE0;
    constexpr uint16_t TVOC_MAGENTA = 0xF81F;
    constexpr uint16_t PM_RED      = 0xF800;
    constexpr uint16_t LABEL_GRAY  = 0x8410;
    constexpr uint16_t GRAPH_BG    = 0x1082;
    constexpr uint16_t BACKGROUND  = 0x0000;
}
```

### 4.2 ISensor.h

```cpp
// src/core/ISensor.h
#pragma once

#include "SensorTypes.h"

/**
 * Abstract interface for all I2C sensors.
 *
 * Implementation checklist:
 * 1. Define static SENSOR_ID, SENSOR_NAME, I2C_ADDRESS, VALUE_COUNT
 * 2. Create static const _descriptors[] array
 * 3. Store readings in _readings[] array
 * 4. Implement all virtual methods
 */
class ISensor {
public:
    virtual ~ISensor() = default;

    //----------------------------------------------------------
    // Identification
    //----------------------------------------------------------

    /**
     * Unique sensor type identifier (e.g., "BME280", "SCD40").
     * Used for logging and diagnostics.
     */
    virtual const char* getSensorId() const = 0;

    /**
     * Human-friendly sensor name (e.g., "Environmental", "Air Quality").
     * May be displayed in UI.
     */
    virtual const char* getSensorName() const = 0;

    /**
     * The I2C address this sensor uses.
     * Used for bus scanning and conflict detection.
     */
    virtual uint8_t getI2CAddress() const = 0;

    //----------------------------------------------------------
    // Lifecycle
    //----------------------------------------------------------

    /**
     * Initialize the sensor hardware.
     * Called once during setup.
     *
     * @return true if sensor detected and configured successfully
     */
    virtual bool begin() = 0;

    /**
     * Check if sensor is currently responding.
     * Should be quick (no I2C transaction if possible).
     *
     * @return true if sensor appears connected
     */
    virtual bool isConnected() const = 0;

    //----------------------------------------------------------
    // Value Descriptors
    //----------------------------------------------------------

    /**
     * Number of values this sensor exposes.
     * E.g., BME280 = 3 (temp, humidity, pressure)
     */
    virtual uint8_t getValueCount() const = 0;

    /**
     * Get metadata for a specific value.
     *
     * @param index Value index (0 to getValueCount()-1)
     * @return Pointer to descriptor (static/const), or nullptr if invalid
     */
    virtual const SensorValueDescriptor* getValueDescriptor(uint8_t index) const = 0;

    //----------------------------------------------------------
    // Data Acquisition
    //----------------------------------------------------------

    /**
     * Perform a sensor reading.
     * Updates internal readings array.
     *
     * @return true if read succeeded for all values
     */
    virtual bool read() = 0;

    /**
     * Get the last reading for a specific value.
     *
     * @param index Value index (0 to getValueCount()-1)
     * @return Reading struct (check .valid before using .value)
     */
    virtual SensorReading getValue(uint8_t index) const = 0;
};
```

### 4.3 SensorRegistry.h

```cpp
// src/core/SensorRegistry.h
#pragma once

#include "ISensor.h"
#include "Config.h"

/**
 * Manages all registered sensors.
 * Provides unified access to readings across multiple sensors.
 */
class SensorRegistry {
public:
    SensorRegistry();

    /**
     * Register a sensor instance.
     * Must be called before initializeAll().
     *
     * @param sensor Pointer to sensor (must remain valid)
     * @return true if registered (false if full or null)
     */
    bool registerSensor(ISensor* sensor);

    /**
     * Initialize all registered sensors.
     * Calls begin() on each sensor.
     *
     * @return true if all sensors initialized successfully
     */
    bool initializeAll();

    /**
     * Poll all sensors.
     * Calls read() on each connected sensor.
     */
    void pollAll();

    //----------------------------------------------------------
    // Unified Value Access
    //----------------------------------------------------------

    /**
     * Total number of values across all sensors.
     * Used for display iteration.
     */
    uint8_t getTotalValueCount() const;

    /**
     * Get a reading by global index.
     * Global indices are assigned in registration order.
     *
     * @param globalIndex Index from 0 to getTotalValueCount()-1
     */
    SensorReading getReading(uint8_t globalIndex) const;

    /**
     * Get a value descriptor by global index.
     */
    const SensorValueDescriptor* getDescriptor(uint8_t globalIndex) const;

    //----------------------------------------------------------
    // Diagnostics
    //----------------------------------------------------------

    uint8_t getSensorCount() const;
    ISensor* getSensor(uint8_t index) const;
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
```

### 4.4 DisplayManager.h

```cpp
// src/display/DisplayManager.h
#pragma once

#include <Adafruit_ST7735.h>
#include "core/SensorTypes.h"
#include "ValueBuffer.h"
#include "Config.h"

/**
 * Layout modes for the display.
 */
enum class LayoutMode : uint8_t {
    TWO_COLUMN,              // 6 values in 2x3 grid
    VALUES_WITH_GRAPH,       // 3 values left, graph right
    COMPACT_WITH_LARGE_GRAPH // Compact row top, large graph below
};

/**
 * Manages rendering of sensor values to TFT display.
 * Handles layout, value formatting, and graphing.
 */
class DisplayManager {
public:
    DisplayManager();

    /**
     * Initialize with TFT instance.
     * @param tft Pointer to initialized Adafruit_ST7735
     */
    void begin(Adafruit_ST7735* tft);

    /**
     * Set display layout mode.
     */
    void setLayout(LayoutMode mode);

    /**
     * Configure which values appear in graph.
     * @param mask Bitmask of value indices (bit 0 = value 0, etc.)
     */
    void setGraphValues(uint8_t mask);

    /**
     * Render all values and graph.
     * Call this each update cycle.
     */
    void render(const SensorReading* readings,
                const SensorValueDescriptor* const* descriptors,
                uint8_t count);

    /**
     * Update history buffers for graphing.
     * Call before render() with same readings.
     */
    void updateHistory(const SensorReading* readings, uint8_t count);

    /**
     * Force full screen redraw next render.
     */
    void invalidate();

private:
    Adafruit_ST7735* _tft;
    LayoutMode _layout;
    ValueBuffer _history[MAX_DISPLAY_VALUES];
    uint8_t _graphMask;
    bool _needsFullRedraw;

    // Cached for change detection
    float _lastValues[MAX_DISPLAY_VALUES];

    // Layout helpers
    void renderTwoColumn(const SensorReading* readings,
                         const SensorValueDescriptor* const* descriptors,
                         uint8_t count);

    void renderValuesWithGraph(const SensorReading* readings,
                               const SensorValueDescriptor* const* descriptors,
                               uint8_t count);

    void renderCompactWithGraph(const SensorReading* readings,
                                const SensorValueDescriptor* const* descriptors,
                                uint8_t count);

    // Rendering primitives
    void renderValueCell(int16_t x, int16_t y, int16_t w, int16_t h,
                        const SensorReading& reading,
                        const SensorValueDescriptor& desc);

    void renderCompactValue(int16_t x, int16_t y,
                           const SensorReading& reading,
                           const SensorValueDescriptor& desc);

    void renderGraph(int16_t x, int16_t y, int16_t w, int16_t h,
                    const SensorValueDescriptor* const* descriptors,
                    uint8_t count);

    void drawProgressBar(int16_t x, int16_t y, int16_t w, int16_t h,
                        float value, float minVal, float maxVal,
                        uint16_t color);

    // Color utilities
    uint16_t getTemperatureColor(float temp);
};
```

### 4.5 ValueBuffer.h

```cpp
// src/display/ValueBuffer.h
#pragma once

#include <stdint.h>
#include "Config.h"

/**
 * Circular buffer for storing value history.
 * Used for trend graphs and change detection.
 */
class ValueBuffer {
public:
    ValueBuffer();

    /**
     * Add a new value to the buffer.
     * Overwrites oldest value when full.
     */
    void push(float value);

    /**
     * Get a historical value.
     * @param age 0 = most recent, 1 = previous, etc.
     * @return Value at that age, or 0 if not enough history
     */
    float get(uint8_t age) const;

    /**
     * Number of values stored.
     */
    uint8_t getCount() const;

    /**
     * Minimum value in buffer.
     */
    float getMin() const;

    /**
     * Maximum value in buffer.
     */
    float getMax() const;

    /**
     * Calculate trend direction.
     * Compares recent average to older average.
     * @return -1 (falling), 0 (stable), +1 (rising)
     */
    int8_t getTrend() const;

    /**
     * Clear all history.
     */
    void clear();

private:
    float _buffer[HISTORY_SIZE];
    uint8_t _index;
    bool _filled;

    // Cached statistics (updated on push)
    float _min;
    float _max;

    void updateStats();
};
```

---

## 5. Implementation: BME280Sensor

### 5.1 BME280Sensor.h

```cpp
// src/sensors/BME280Sensor.h
#pragma once

#include "core/ISensor.h"
#include <Adafruit_BME280.h>

class BME280Sensor : public ISensor {
public:
    // Sensor metadata
    static constexpr const char* SENSOR_ID = "BME280";
    static constexpr const char* SENSOR_NAME = "Environmental";
    static constexpr uint8_t I2C_ADDR = 0x76;
    static constexpr uint8_t NUM_VALUES = 3;

    // Value indices
    enum ValueIndex : uint8_t {
        TEMPERATURE = 0,
        HUMIDITY = 1,
        PRESSURE = 2
    };

    BME280Sensor();

    // ISensor implementation
    const char* getSensorId() const override { return SENSOR_ID; }
    const char* getSensorName() const override { return SENSOR_NAME; }
    uint8_t getI2CAddress() const override { return I2C_ADDR; }

    bool begin() override;
    bool isConnected() const override { return _connected; }

    uint8_t getValueCount() const override { return NUM_VALUES; }
    const SensorValueDescriptor* getValueDescriptor(uint8_t index) const override;

    bool read() override;
    SensorReading getValue(uint8_t index) const override;

private:
    Adafruit_BME280 _bme;
    SensorReading _readings[NUM_VALUES];
    bool _connected;

    static const SensorValueDescriptor DESCRIPTORS[NUM_VALUES];
};
```

### 5.2 BME280Sensor.cpp

```cpp
// src/sensors/BME280Sensor.cpp

#include "BME280Sensor.h"
#include <Arduino.h>

// Static descriptor definitions (stored in flash)
const SensorValueDescriptor BME280Sensor::DESCRIPTORS[NUM_VALUES] = {
    // Temperature
    {
        .id = "bme_temp",
        .name = "Temp",
        .unit = "\xB0" "C",  // Degree symbol + C
        .minValue = 15.0f,
        .maxValue = 35.0f,
        .displayColor = Colors::TEMP_ORANGE,
        .showGraph = true,
        .precision = 1
    },
    // Humidity
    {
        .id = "bme_humid",
        .name = "Humid",
        .unit = "%",
        .minValue = 0.0f,
        .maxValue = 100.0f,
        .displayColor = Colors::HUMID_CYAN,
        .showGraph = false,
        .precision = 0
    },
    // Pressure
    {
        .id = "bme_press",
        .name = "Press",
        .unit = "hPa",
        .minValue = 980.0f,
        .maxValue = 1040.0f,
        .displayColor = Colors::PRESS_GREEN,
        .showGraph = false,
        .precision = 0
    }
};

BME280Sensor::BME280Sensor()
    : _connected(false)
{
    for (uint8_t i = 0; i < NUM_VALUES; i++) {
        _readings[i] = SensorReading::invalid(DESCRIPTORS[i].id);
    }
}

bool BME280Sensor::begin() {
    _connected = _bme.begin(I2C_ADDR);
    if (_connected) {
        // Configure for weather monitoring (low power)
        _bme.setSampling(
            Adafruit_BME280::MODE_FORCED,
            Adafruit_BME280::SAMPLING_X1,  // Temperature
            Adafruit_BME280::SAMPLING_X1,  // Pressure
            Adafruit_BME280::SAMPLING_X1,  // Humidity
            Adafruit_BME280::FILTER_OFF
        );
    }
    return _connected;
}

const SensorValueDescriptor* BME280Sensor::getValueDescriptor(uint8_t index) const {
    if (index >= NUM_VALUES) return nullptr;
    return &DESCRIPTORS[index];
}

bool BME280Sensor::read() {
    if (!_connected) {
        for (uint8_t i = 0; i < NUM_VALUES; i++) {
            _readings[i].valid = false;
        }
        return false;
    }

    // Trigger measurement (forced mode)
    _bme.takeForcedMeasurement();

    uint32_t now = millis();

    _readings[TEMPERATURE] = SensorReading::make(
        DESCRIPTORS[TEMPERATURE].id,
        _bme.readTemperature()
    );
    _readings[TEMPERATURE].timestamp = now;

    _readings[HUMIDITY] = SensorReading::make(
        DESCRIPTORS[HUMIDITY].id,
        _bme.readHumidity()
    );
    _readings[HUMIDITY].timestamp = now;

    _readings[PRESSURE] = SensorReading::make(
        DESCRIPTORS[PRESSURE].id,
        _bme.readPressure() / 100.0f  // Convert Pa to hPa
    );
    _readings[PRESSURE].timestamp = now;

    return true;
}

SensorReading BME280Sensor::getValue(uint8_t index) const {
    if (index >= NUM_VALUES) {
        return SensorReading::invalid("invalid");
    }
    return _readings[index];
}
```

---

## 6. Main Application Flow

### 6.1 main.cpp Structure

```cpp
// src/main.cpp

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ST7735.h>

#include "Config.h"
#include "core/SensorRegistry.h"
#include "display/DisplayManager.h"
#include "sensors/BME280Sensor.h"
// #include "sensors/SCD40Sensor.h"  // Uncomment to add

// Hardware objects
Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// Core system
SensorRegistry registry;
DisplayManager display;

// Sensor instances
BME280Sensor bme280;
// SCD40Sensor scd40;  // Uncomment to add

// Timing
unsigned long lastUpdate = 0;

void setup() {
    Serial.begin(115200);

    // Initialize I2C
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_FREQUENCY);

    // Initialize display hardware
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
    tft.initR(INITR_MINI160x80);
    tft.setRotation(DISPLAY_ROTATION);
    tft.fillScreen(Colors::BACKGROUND);

    // Register sensors
    registry.registerSensor(&bme280);
    // registry.registerSensor(&scd40);  // Uncomment to add

    // Initialize all sensors
    if (!registry.initializeAll()) {
        tft.setTextColor(0xF800);  // Red
        tft.setCursor(10, 30);
        tft.print("Sensor Error!");
        while (1) delay(1000);
    }

    // Initialize display manager
    display.begin(&tft);
    display.setLayout(LayoutMode::VALUES_WITH_GRAPH);
    display.setGraphValues(0x01);  // Graph first value (temperature)

    Serial.printf("Sensors: %d, Values: %d\n",
                  registry.getSensorCount(),
                  registry.getTotalValueCount());
}

void loop() {
    unsigned long now = millis();

    if (now - lastUpdate >= SENSOR_POLL_INTERVAL_MS) {
        lastUpdate = now;

        // Poll all sensors
        registry.pollAll();

        // Prepare readings array for display
        uint8_t count = registry.getTotalValueCount();
        SensorReading readings[MAX_DISPLAY_VALUES];
        const SensorValueDescriptor* descriptors[MAX_DISPLAY_VALUES];

        for (uint8_t i = 0; i < count && i < MAX_DISPLAY_VALUES; i++) {
            readings[i] = registry.getReading(i);
            descriptors[i] = registry.getDescriptor(i);
        }

        // Update display
        display.updateHistory(readings, count);
        display.render(readings, descriptors, count);

        // Debug output
        for (uint8_t i = 0; i < count; i++) {
            if (readings[i].valid) {
                Serial.printf("%s: %.1f %s\n",
                              descriptors[i]->name,
                              readings[i].value,
                              descriptors[i]->unit);
            }
        }
    }
}
```

---

## 7. Adding a New Sensor: Step-by-Step

### 7.1 Example: Adding SCD40 CO2 Sensor

**Step 1:** Create header file `src/sensors/SCD40Sensor.h`

```cpp
#pragma once

#include "core/ISensor.h"
#include <SensirionI2CScd4x.h>

class SCD40Sensor : public ISensor {
public:
    static constexpr const char* SENSOR_ID = "SCD40";
    static constexpr const char* SENSOR_NAME = "Air Quality";
    static constexpr uint8_t I2C_ADDR = 0x62;
    static constexpr uint8_t NUM_VALUES = 3;

    enum ValueIndex : uint8_t {
        CO2 = 0,
        TEMPERATURE = 1,
        HUMIDITY = 2
    };

    // ... standard ISensor implementation ...

private:
    SensirionI2CScd4x _scd;
    SensorReading _readings[NUM_VALUES];
    bool _connected;
    static const SensorValueDescriptor DESCRIPTORS[NUM_VALUES];
};
```

**Step 2:** Create implementation `src/sensors/SCD40Sensor.cpp`

```cpp
#include "SCD40Sensor.h"

const SensorValueDescriptor SCD40Sensor::DESCRIPTORS[NUM_VALUES] = {
    {"co2", "CO2", "ppm", 400.0f, 2000.0f, Colors::CO2_YELLOW, true, 0},
    {"scd_temp", "Temp", "°C", 15.0f, 35.0f, Colors::TEMP_ORANGE, false, 1},
    {"scd_humid", "RH", "%", 0.0f, 100.0f, Colors::HUMID_CYAN, false, 0}
};

bool SCD40Sensor::begin() {
    _scd.begin(Wire);
    uint16_t error = _scd.startPeriodicMeasurement();
    _connected = (error == 0);
    return _connected;
}

bool SCD40Sensor::read() {
    uint16_t co2;
    float temp, humid;
    bool ready = false;

    _scd.getDataReadyFlag(ready);
    if (!ready) return true;  // Not an error, just not ready

    uint16_t error = _scd.readMeasurement(co2, temp, humid);
    if (error) {
        _readings[CO2].valid = false;
        return false;
    }

    uint32_t now = millis();
    _readings[CO2] = {DESCRIPTORS[CO2].id, (float)co2, true, now};
    _readings[TEMPERATURE] = {DESCRIPTORS[TEMPERATURE].id, temp, true, now};
    _readings[HUMIDITY] = {DESCRIPTORS[HUMIDITY].id, humid, true, now};

    return true;
}
```

**Step 3:** Add library to `platformio.ini`

```ini
lib_deps =
    ...existing...
    sensirion/Sensirion I2C SCD4x@^0.4.0
```

**Step 4:** Register in `main.cpp`

```cpp
#include "sensors/SCD40Sensor.h"

SCD40Sensor scd40;

void setup() {
    ...
    registry.registerSensor(&scd40);
    ...
}
```

**Done!** The display automatically shows 6 values (3 from BME280 + 3 from SCD40).

---

## 8. Display Layout Details

### 8.1 Two-Column Layout Geometry

```
160px width × 80px height

┌────────────────────────────────────────────────────────────┐
│ Col 0 (0-79px)           │ Col 1 (80-159px)               │
├────────────────────────────────────────────────────────────┤
│ Row 0: y=0-25   (26px)   │ Row 0: y=0-25                  │
│   Value 0                │   Value 3                      │
├────────────────────────────────────────────────────────────┤
│ Row 1: y=26-51  (26px)   │ Row 1: y=26-51                 │
│   Value 1                │   Value 4                      │
├────────────────────────────────────────────────────────────┤
│ Row 2: y=52-77  (26px)   │ Row 2: y=52-77                 │
│   Value 2                │   Value 5                      │
└────────────────────────────────────────────────────────────┘

Cell layout (80x26 each):
┌─────────────────────────────────────────┐
│ Label (gray, small)         y+2         │
│ Value + Unit (colored)      y+10        │
│ ████████████████░░░░░░░░░  y+20 (bar)   │
└─────────────────────────────────────────┘
```

### 8.2 Values + Graph Layout

```
┌─────────────────────────────────────────────────────────────┐
│ Values (0-94px)        │ Graph (95-159px, 65px wide)       │
├─────────────────────────────────────────────────────────────┤
│ ┌────────────────────┐ │ ┌─────────────────────────────────┐│
│ │ Value 0            │ │ │                                 ││
│ │ 23.4°C             │ │ │      Temp ─────                 ││
│ │ ████████░░░░░░░░░░ │ │ │          ╲    ╱─────            ││
│ └────────────────────┘ │ │           ╲──╱                  ││
│ ┌────────────────────┐ │ │                                 ││
│ │ Value 1            │ │ │      CO2  ═════════             ││
│ │ 65%                │ │ │                   ════          ││
│ │ ██████░░░░░░░░░░░░ │ │ │                                 ││
│ └────────────────────┘ │ │                                 ││
│ ┌────────────────────┐ │ │                                 ││
│ │ Value 2            │ │ │                                 ││
│ │ 1013hPa            │ │ │   Auto-scaled Y axis            ││
│ │ ████████████░░░░░░ │ │ │   X: 50 samples (history)       ││
│ └────────────────────┘ │ └─────────────────────────────────┘│
└─────────────────────────────────────────────────────────────┘
```

---

## 9. Error Handling Strategy

### 9.1 Sensor Errors

| Scenario | Behavior |
|----------|----------|
| Sensor not found at init | Log error, exclude from registry |
| Sensor read fails | Mark readings as invalid, show "---" |
| Sensor disconnects | Continue with last valid reading, show indicator |
| I2C bus error | Retry on next poll cycle |

### 9.2 Display Errors

| Scenario | Behavior |
|----------|----------|
| No sensors registered | Show "No Sensors" message |
| All readings invalid | Show "---" for values, skip graph |
| <3 values for layout | Use simpler layout automatically |

---

## 10. Testing Strategy

### 10.1 Unit Tests (Host-based)

- SensorValueDescriptor validation
- ValueBuffer push/get/statistics
- SensorRegistry mapping logic

### 10.2 Integration Tests (Hardware)

- I2C bus scan detects expected sensors
- Each sensor reads valid values
- Display renders without glitches
- 2-second update rate maintained

### 10.3 Mock Sensor for Development

```cpp
// src/sensors/MockSensor.h
class MockSensor : public ISensor {
    // Returns configurable fake data
    // Useful for UI testing without hardware
};
```

---

## 11. Future Enhancements

| Enhancement | Complexity | Notes |
|-------------|------------|-------|
| Auto-detect sensors via I2C scan | Medium | Scan 0x00-0x7F, match known addresses |
| WiFi data logging | High | Send readings to MQTT/InfluxDB |
| Button to cycle layouts | Low | Single GPIO, state machine |
| Configurable value priority | Medium | Choose which values display when >6 |
| Alarm thresholds | Medium | Flash display when value exceeds limit |
| Deep sleep support | High | Wake on timer, preserve history |

---

*End of Design Document*
