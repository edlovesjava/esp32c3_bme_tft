# Modular Sensor Architecture Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Refactor monolithic sensor/display code into extensible architecture where sensors self-describe and display adapts automatically.

**Architecture:** Sensor interface (ISensor) with self-describing value descriptors. SensorRegistry aggregates sensors. DisplayManager renders any 1-6 values. Each sensor isolated in own files.

**Tech Stack:** PlatformIO, ESP32-C3, Adafruit GFX/ST7735/BME280 libraries, C++11

---

## Task 1: Create Config.h with Pin Definitions

**Files:**
- Create: `include/Config.h`

**Step 1: Create the configuration header**

```cpp
// include/Config.h
#pragma once

// I2C Configuration
#define I2C_SDA_PIN         8
#define I2C_SCL_PIN         9
#define I2C_FREQUENCY       100000

// SPI/Display Configuration
#define TFT_SCLK            4
#define TFT_MOSI            6
#define TFT_CS              7
#define TFT_DC              5
#define TFT_RST             10
#define TFT_BL              3

// Display Settings
#define DISPLAY_WIDTH       160
#define DISPLAY_HEIGHT      80
#define DISPLAY_ROTATION    1

// System Configuration
#define SENSOR_POLL_INTERVAL_MS  2000
#define HISTORY_SIZE             50
#define MAX_DISPLAY_VALUES       6
#define MAX_SENSORS              4
```

**Step 2: Verify file created**

Run: `ls include/`
Expected: `Config.h`

**Step 3: Commit**

```bash
git add include/Config.h
git commit -m "feat: add Config.h with pin and system definitions

Co-Authored-By: Claude Opus 4.5 <noreply@anthropic.com>"
```

---

## Task 2: Create Core Type Definitions

**Files:**
- Create: `src/core/SensorTypes.h`

**Step 1: Create sensor types header**

```cpp
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
```

**Step 2: Verify directory structure**

Run: `ls src/core/`
Expected: `SensorTypes.h`

**Step 3: Commit**

```bash
git add src/core/SensorTypes.h
git commit -m "feat: add SensorTypes.h with value descriptors and readings

Co-Authored-By: Claude Opus 4.5 <noreply@anthropic.com>"
```

---

## Task 3: Create ISensor Interface

**Files:**
- Create: `src/core/ISensor.h`

**Step 1: Create sensor interface**

```cpp
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
```

**Step 2: Verify file exists**

Run: `ls src/core/`
Expected: `ISensor.h  SensorTypes.h`

**Step 3: Commit**

```bash
git add src/core/ISensor.h
git commit -m "feat: add ISensor abstract interface

Co-Authored-By: Claude Opus 4.5 <noreply@anthropic.com>"
```

---

## Task 4: Create SensorRegistry

**Files:**
- Create: `src/core/SensorRegistry.h`
- Create: `src/core/SensorRegistry.cpp`

**Step 1: Create registry header**

```cpp
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
```

**Step 2: Create registry implementation**

```cpp
// src/core/SensorRegistry.cpp
#include "SensorRegistry.h"

SensorRegistry::SensorRegistry()
    : _sensorCount(0), _totalValues(0) {
    for (uint8_t i = 0; i < MAX_SENSORS; i++) {
        _sensors[i] = nullptr;
    }
}

bool SensorRegistry::registerSensor(ISensor* sensor) {
    if (sensor == nullptr || _sensorCount >= MAX_SENSORS) {
        return false;
    }
    _sensors[_sensorCount++] = sensor;
    return true;
}

bool SensorRegistry::initializeAll() {
    bool allOk = true;
    for (uint8_t i = 0; i < _sensorCount; i++) {
        if (!_sensors[i]->begin()) {
            allOk = false;
        }
    }
    buildValueMap();
    return allOk;
}

void SensorRegistry::pollAll() {
    for (uint8_t i = 0; i < _sensorCount; i++) {
        if (_sensors[i]->isConnected()) {
            _sensors[i]->read();
        }
    }
}

void SensorRegistry::buildValueMap() {
    _totalValues = 0;
    for (uint8_t s = 0; s < _sensorCount && _totalValues < MAX_DISPLAY_VALUES; s++) {
        if (!_sensors[s]->isConnected()) continue;
        uint8_t count = _sensors[s]->getValueCount();
        for (uint8_t v = 0; v < count && _totalValues < MAX_DISPLAY_VALUES; v++) {
            _valueMap[_totalValues].sensorIndex = s;
            _valueMap[_totalValues].valueIndex = v;
            _totalValues++;
        }
    }
}

uint8_t SensorRegistry::getTotalValueCount() const {
    return _totalValues;
}

SensorReading SensorRegistry::getReading(uint8_t globalIndex) const {
    if (globalIndex >= _totalValues) {
        return SensorReading::invalid("invalid");
    }
    const ValueMapping& m = _valueMap[globalIndex];
    return _sensors[m.sensorIndex]->getValue(m.valueIndex);
}

const SensorValueDescriptor* SensorRegistry::getDescriptor(uint8_t globalIndex) const {
    if (globalIndex >= _totalValues) {
        return nullptr;
    }
    const ValueMapping& m = _valueMap[globalIndex];
    return _sensors[m.sensorIndex]->getValueDescriptor(m.valueIndex);
}

uint8_t SensorRegistry::getSensorCount() const {
    return _sensorCount;
}

bool SensorRegistry::isSensorConnected(uint8_t sensorIndex) const {
    if (sensorIndex >= _sensorCount) return false;
    return _sensors[sensorIndex]->isConnected();
}
```

**Step 3: Verify files**

Run: `ls src/core/`
Expected: `ISensor.h  SensorRegistry.cpp  SensorRegistry.h  SensorTypes.h`

**Step 4: Commit**

```bash
git add src/core/SensorRegistry.h src/core/SensorRegistry.cpp
git commit -m "feat: add SensorRegistry for managing multiple sensors

Co-Authored-By: Claude Opus 4.5 <noreply@anthropic.com>"
```

---

## Task 5: Create ValueBuffer for History

**Files:**
- Create: `src/display/ValueBuffer.h`
- Create: `src/display/ValueBuffer.cpp`

**Step 1: Create value buffer header**

```cpp
// src/display/ValueBuffer.h
#pragma once

#include <stdint.h>
#include "Config.h"

class ValueBuffer {
public:
    ValueBuffer();

    void push(float value);
    float get(uint8_t age) const;
    uint8_t getCount() const;
    float getMin() const;
    float getMax() const;
    int8_t getTrend() const;
    void clear();

private:
    float _buffer[HISTORY_SIZE];
    uint8_t _index;
    bool _filled;
    float _min;
    float _max;

    void updateStats();
};
```

**Step 2: Create value buffer implementation**

```cpp
// src/display/ValueBuffer.cpp
#include "ValueBuffer.h"
#include <float.h>

ValueBuffer::ValueBuffer()
    : _index(0), _filled(false), _min(0), _max(0) {
    for (uint8_t i = 0; i < HISTORY_SIZE; i++) {
        _buffer[i] = 0;
    }
}

void ValueBuffer::push(float value) {
    _buffer[_index] = value;
    _index = (_index + 1) % HISTORY_SIZE;
    if (_index == 0) _filled = true;
    updateStats();
}

float ValueBuffer::get(uint8_t age) const {
    if (age >= getCount()) return 0;
    int idx = (_index - 1 - age + HISTORY_SIZE) % HISTORY_SIZE;
    return _buffer[idx];
}

uint8_t ValueBuffer::getCount() const {
    return _filled ? HISTORY_SIZE : _index;
}

float ValueBuffer::getMin() const { return _min; }
float ValueBuffer::getMax() const { return _max; }

int8_t ValueBuffer::getTrend() const {
    uint8_t count = getCount();
    if (count < 5) return 0;

    float recent = (get(0) + get(1) + get(2)) / 3.0f;
    float older = (get(count-1) + get(count-2) + get(count-3)) / 3.0f;
    float diff = recent - older;

    if (diff > 0.5f) return 1;
    if (diff < -0.5f) return -1;
    return 0;
}

void ValueBuffer::clear() {
    _index = 0;
    _filled = false;
    _min = 0;
    _max = 0;
}

void ValueBuffer::updateStats() {
    uint8_t count = getCount();
    if (count == 0) return;

    _min = _buffer[0];
    _max = _buffer[0];
    for (uint8_t i = 1; i < count; i++) {
        if (_buffer[i] < _min) _min = _buffer[i];
        if (_buffer[i] > _max) _max = _buffer[i];
    }
}
```

**Step 3: Verify files**

Run: `ls src/display/`
Expected: `ValueBuffer.cpp  ValueBuffer.h`

**Step 4: Commit**

```bash
git add src/display/ValueBuffer.h src/display/ValueBuffer.cpp
git commit -m "feat: add ValueBuffer for history tracking and trends

Co-Authored-By: Claude Opus 4.5 <noreply@anthropic.com>"
```

---

## Task 6: Create DisplayManager

**Files:**
- Create: `src/display/DisplayManager.h`
- Create: `src/display/DisplayManager.cpp`

**Step 1: Create display manager header**

```cpp
// src/display/DisplayManager.h
#pragma once

#include <Adafruit_ST7735.h>
#include "core/SensorTypes.h"
#include "ValueBuffer.h"
#include "Config.h"

enum class LayoutMode : uint8_t {
    TWO_COLUMN,
    VALUES_WITH_GRAPH,
    COMPACT_WITH_LARGE_GRAPH
};

class DisplayManager {
public:
    DisplayManager();

    void begin(Adafruit_ST7735* tft);
    void setLayout(LayoutMode mode);
    void setGraphValues(uint8_t mask);

    void render(const SensorReading* readings,
                const SensorValueDescriptor* const* descriptors,
                uint8_t count);
    void updateHistory(const SensorReading* readings, uint8_t count);
    void invalidate();

private:
    Adafruit_ST7735* _tft;
    LayoutMode _layout;
    ValueBuffer _history[MAX_DISPLAY_VALUES];
    uint8_t _graphMask;
    bool _needsFullRedraw;

    void renderValuesWithGraph(const SensorReading* readings,
                               const SensorValueDescriptor* const* descriptors,
                               uint8_t count);

    void renderValueCell(int16_t x, int16_t y, int16_t w, int16_t h,
                        const SensorReading& reading,
                        const SensorValueDescriptor& desc);

    void renderGraph(int16_t x, int16_t y, int16_t w, int16_t h);

    uint16_t getTemperatureColor(float temp, float minT, float maxT);
};
```

**Step 2: Create display manager implementation**

```cpp
// src/display/DisplayManager.cpp
#include "DisplayManager.h"

DisplayManager::DisplayManager()
    : _tft(nullptr), _layout(LayoutMode::VALUES_WITH_GRAPH),
      _graphMask(0x01), _needsFullRedraw(true) {}

void DisplayManager::begin(Adafruit_ST7735* tft) {
    _tft = tft;
    _needsFullRedraw = true;
}

void DisplayManager::setLayout(LayoutMode mode) {
    _layout = mode;
    _needsFullRedraw = true;
}

void DisplayManager::setGraphValues(uint8_t mask) {
    _graphMask = mask;
}

void DisplayManager::invalidate() {
    _needsFullRedraw = true;
}

void DisplayManager::updateHistory(const SensorReading* readings, uint8_t count) {
    for (uint8_t i = 0; i < count && i < MAX_DISPLAY_VALUES; i++) {
        if (readings[i].valid) {
            _history[i].push(readings[i].value);
        }
    }
}

void DisplayManager::render(const SensorReading* readings,
                            const SensorValueDescriptor* const* descriptors,
                            uint8_t count) {
    if (_tft == nullptr || count == 0) return;

    if (_needsFullRedraw) {
        _tft->fillScreen(Colors::BACKGROUND);
        _needsFullRedraw = false;
    }

    // Currently only VALUES_WITH_GRAPH implemented
    renderValuesWithGraph(readings, descriptors, count);
}

void DisplayManager::renderValuesWithGraph(const SensorReading* readings,
                                           const SensorValueDescriptor* const* descriptors,
                                           uint8_t count) {
    // Left side: values (0-95), up to 3 values
    uint8_t displayCount = (count > 3) ? 3 : count;
    int16_t cellHeight = 26;

    for (uint8_t i = 0; i < displayCount; i++) {
        renderValueCell(0, i * cellHeight, 96, cellHeight,
                       readings[i], *descriptors[i]);
    }

    // Right side: graph (96-159)
    _tft->drawRect(96, 0, 64, 80, Colors::LABEL_GRAY);
    renderGraph(97, 12, 62, 66);
}

void DisplayManager::renderValueCell(int16_t x, int16_t y, int16_t w, int16_t h,
                                     const SensorReading& reading,
                                     const SensorValueDescriptor& desc) {
    // Clear cell
    _tft->fillRect(x, y, w, h - 2, Colors::BACKGROUND);

    // Label
    _tft->setTextColor(Colors::LABEL_GRAY);
    _tft->setTextSize(1);
    _tft->setCursor(x + 2, y + 2);
    _tft->print(desc.name);

    // Value
    _tft->setTextColor(desc.displayColor);
    _tft->setTextSize(2);
    _tft->setCursor(x + 2, y + 10);

    if (!reading.valid) {
        _tft->print("---");
    } else if (desc.precision == 0) {
        _tft->print((int)reading.value);
    } else {
        _tft->print(reading.value, desc.precision);
    }

    // Unit
    _tft->setTextSize(1);
    _tft->print(desc.unit);

    // Progress bar
    int16_t barY = y + h - 6;
    int16_t barW = w - 4;
    _tft->fillRect(x + 2, barY, barW, 4, Colors::GRAPH_BG);

    if (reading.valid) {
        float pct = constrain((reading.value - desc.minValue) /
                             (desc.maxValue - desc.minValue), 0.0f, 1.0f);
        int16_t fillW = (int16_t)(pct * barW);
        _tft->fillRect(x + 2, barY, fillW, 4, desc.displayColor);
    }
}

void DisplayManager::renderGraph(int16_t x, int16_t y, int16_t w, int16_t h) {
    // Clear graph area
    _tft->fillRect(x, y, w, h, Colors::GRAPH_BG);

    // Only graph first value for now (index 0)
    if (_history[0].getCount() < 2) return;

    float minT = _history[0].getMin();
    float maxT = _history[0].getMax();

    // Ensure minimum range
    if (maxT - minT < 2.0f) {
        float mid = (maxT + minT) / 2.0f;
        minT = mid - 1.0f;
        maxT = mid + 1.0f;
    }

    // Draw center line
    int16_t centerY = y + h / 2;
    for (int16_t px = x; px < x + w; px += 3) {
        _tft->drawPixel(px, centerY, Colors::LABEL_GRAY);
    }

    // Draw temperature line
    uint8_t count = _history[0].getCount();
    for (uint8_t i = 1; i < count; i++) {
        float v1 = _history[0].get(count - i);
        float v2 = _history[0].get(count - i - 1);

        int16_t x1 = x + map(i - 1, 0, HISTORY_SIZE - 1, 0, w - 1);
        int16_t x2 = x + map(i, 0, HISTORY_SIZE - 1, 0, w - 1);

        int16_t y1 = y + h - 1 - (int16_t)((v1 - minT) / (maxT - minT) * (h - 1));
        int16_t y2 = y + h - 1 - (int16_t)((v2 - minT) / (maxT - minT) * (h - 1));

        y1 = constrain(y1, y, y + h - 1);
        y2 = constrain(y2, y, y + h - 1);

        uint16_t color = getTemperatureColor(v2, minT, maxT);
        _tft->drawLine(x1, y1, x2, y2, color);
    }

    // Current value dot
    float lastVal = _history[0].get(0);
    int16_t dotY = y + h - 1 - (int16_t)((lastVal - minT) / (maxT - minT) * (h - 1));
    dotY = constrain(dotY, y + 2, y + h - 3);
    _tft->fillCircle(x + w - 2, dotY, 2, 0xFFFF);
}

uint16_t DisplayManager::getTemperatureColor(float temp, float minT, float maxT) {
    float norm = constrain((temp - minT) / (maxT - minT), 0.0f, 1.0f);

    uint8_t r, g, b;
    if (norm < 0.25f) {
        float t = norm * 4.0f;
        r = 0; g = (uint8_t)(t * 255); b = 255;
    } else if (norm < 0.5f) {
        float t = (norm - 0.25f) * 4.0f;
        r = 0; g = 255; b = (uint8_t)((1.0f - t) * 255);
    } else if (norm < 0.75f) {
        float t = (norm - 0.5f) * 4.0f;
        r = (uint8_t)(t * 255); g = 255; b = 0;
    } else {
        float t = (norm - 0.75f) * 4.0f;
        r = 255; g = (uint8_t)((1.0f - t) * 255); b = 0;
    }

    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}
```

**Step 3: Verify files**

Run: `ls src/display/`
Expected: `DisplayManager.cpp  DisplayManager.h  ValueBuffer.cpp  ValueBuffer.h`

**Step 4: Commit**

```bash
git add src/display/DisplayManager.h src/display/DisplayManager.cpp
git commit -m "feat: add DisplayManager with layout rendering

Co-Authored-By: Claude Opus 4.5 <noreply@anthropic.com>"
```

---

## Task 7: Create BME280Sensor Adapter

**Files:**
- Create: `src/sensors/BME280Sensor.h`
- Create: `src/sensors/BME280Sensor.cpp`

**Step 1: Create BME280 sensor header**

```cpp
// src/sensors/BME280Sensor.h
#pragma once

#include "core/ISensor.h"
#include <Adafruit_BME280.h>

class BME280Sensor : public ISensor {
public:
    static constexpr const char* SENSOR_ID = "BME280";
    static constexpr const char* SENSOR_NAME = "Environmental";
    static constexpr uint8_t I2C_ADDR = 0x76;
    static constexpr uint8_t NUM_VALUES = 3;

    enum ValueIndex : uint8_t {
        TEMPERATURE = 0,
        HUMIDITY = 1,
        PRESSURE = 2
    };

    BME280Sensor();

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

**Step 2: Create BME280 sensor implementation**

```cpp
// src/sensors/BME280Sensor.cpp
#include "BME280Sensor.h"
#include <Arduino.h>

const SensorValueDescriptor BME280Sensor::DESCRIPTORS[NUM_VALUES] = {
    {"bme_temp", "Temp", "C", -10.0f, 50.0f, Colors::TEMP_ORANGE, true, 1},
    {"bme_humid", "Humid", "%", 0.0f, 100.0f, Colors::HUMID_CYAN, false, 0},
    {"bme_press", "Press", "hPa", 950.0f, 1050.0f, Colors::PRESS_GREEN, false, 0}
};

BME280Sensor::BME280Sensor() : _connected(false) {
    for (uint8_t i = 0; i < NUM_VALUES; i++) {
        _readings[i] = SensorReading::invalid(DESCRIPTORS[i].id);
    }
}

bool BME280Sensor::begin() {
    _connected = _bme.begin(I2C_ADDR);
    if (_connected) {
        _bme.setSampling(
            Adafruit_BME280::MODE_FORCED,
            Adafruit_BME280::SAMPLING_X1,
            Adafruit_BME280::SAMPLING_X1,
            Adafruit_BME280::SAMPLING_X1,
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

    _bme.takeForcedMeasurement();
    uint32_t now = millis();

    _readings[TEMPERATURE] = SensorReading::make(
        DESCRIPTORS[TEMPERATURE].id, _bme.readTemperature());
    _readings[TEMPERATURE].timestamp = now;

    _readings[HUMIDITY] = SensorReading::make(
        DESCRIPTORS[HUMIDITY].id, _bme.readHumidity());
    _readings[HUMIDITY].timestamp = now;

    _readings[PRESSURE] = SensorReading::make(
        DESCRIPTORS[PRESSURE].id, _bme.readPressure() / 100.0f);
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

**Step 3: Verify files**

Run: `ls src/sensors/`
Expected: `BME280Sensor.cpp  BME280Sensor.h`

**Step 4: Commit**

```bash
git add src/sensors/BME280Sensor.h src/sensors/BME280Sensor.cpp
git commit -m "feat: add BME280Sensor implementing ISensor interface

Co-Authored-By: Claude Opus 4.5 <noreply@anthropic.com>"
```

---

## Task 8: Refactor main.cpp

**Files:**
- Modify: `src/main.cpp`

**Step 1: Replace main.cpp with new orchestration code**

```cpp
// src/main.cpp
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ST7735.h>

#include "Config.h"
#include "core/SensorRegistry.h"
#include "display/DisplayManager.h"
#include "sensors/BME280Sensor.h"

// Hardware
Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// Core system
SensorRegistry registry;
DisplayManager display;

// Sensors
BME280Sensor bme280;

// Timing
unsigned long lastUpdate = 0;

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\nModular Sensor System");

    // Backlight
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);

    // Init TFT
    tft.initR(INITR_MINI160x80);
    tft.setRotation(DISPLAY_ROTATION);
    tft.fillScreen(Colors::BACKGROUND);

    // Splash
    tft.setTextColor(0xFFFF);
    tft.setCursor(20, 30);
    tft.print("Sensor Monitor");

    // Init I2C
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_FREQUENCY);

    // Register sensors
    registry.registerSensor(&bme280);

    // Initialize sensors
    if (!registry.initializeAll()) {
        tft.fillScreen(Colors::BACKGROUND);
        tft.setTextColor(0xF800);
        tft.setCursor(10, 35);
        tft.print("Sensor Error!");
        while (1) delay(1000);
    }

    // Initialize display
    display.begin(&tft);
    display.setLayout(LayoutMode::VALUES_WITH_GRAPH);

    Serial.printf("Sensors: %d, Values: %d\n",
                  registry.getSensorCount(),
                  registry.getTotalValueCount());

    delay(500);
    display.invalidate();
}

void loop() {
    unsigned long now = millis();

    if (now - lastUpdate >= SENSOR_POLL_INTERVAL_MS) {
        lastUpdate = now;

        // Poll sensors
        registry.pollAll();

        // Collect readings
        uint8_t count = registry.getTotalValueCount();
        SensorReading readings[MAX_DISPLAY_VALUES];
        const SensorValueDescriptor* descriptors[MAX_DISPLAY_VALUES];

        for (uint8_t i = 0; i < count; i++) {
            readings[i] = registry.getReading(i);
            descriptors[i] = registry.getDescriptor(i);
        }

        // Update display
        display.updateHistory(readings, count);
        display.render(readings, descriptors, count);

        // Debug output
        for (uint8_t i = 0; i < count; i++) {
            if (readings[i].valid && descriptors[i]) {
                Serial.printf("%s: %.1f %s\n",
                              descriptors[i]->name,
                              readings[i].value,
                              descriptors[i]->unit);
            }
        }
    }
}
```

**Step 2: Verify build compiles**

Run: `pio run`
Expected: Build succeeds

**Step 3: Commit**

```bash
git add src/main.cpp
git commit -m "refactor: main.cpp to use modular sensor architecture

Co-Authored-By: Claude Opus 4.5 <noreply@anthropic.com>"
```

---

## Task 9: Build and Test

**Files:**
- None (verification only)

**Step 1: Clean build**

Run: `pio run --target clean && pio run`
Expected: BUILD SUCCESSFUL

**Step 2: Upload and verify**

Run: `pio run --target upload && pio device monitor`
Expected: Serial output shows sensor readings, display shows values

**Step 3: Commit final state**

```bash
git add -A
git commit -m "feat: complete modular sensor architecture implementation

- Config.h with pin definitions
- ISensor interface with self-describing values
- SensorRegistry for multi-sensor management
- DisplayManager with layout rendering
- ValueBuffer for history tracking
- BME280Sensor adapter

Co-Authored-By: Claude Opus 4.5 <noreply@anthropic.com>"
```

---

## Task 10: Create MockSensor for Validation

**Files:**
- Create: `src/sensors/MockSensor.h`
- Create: `src/sensors/MockSensor.cpp`

**Step 1: Create mock sensor header**

```cpp
// src/sensors/MockSensor.h
#pragma once

#include "core/ISensor.h"

class MockSensor : public ISensor {
public:
    static constexpr const char* SENSOR_ID = "MOCK";
    static constexpr const char* SENSOR_NAME = "Test Sensor";
    static constexpr uint8_t I2C_ADDR = 0x00;
    static constexpr uint8_t NUM_VALUES = 2;

    MockSensor();

    const char* getSensorId() const override { return SENSOR_ID; }
    const char* getSensorName() const override { return SENSOR_NAME; }
    uint8_t getI2CAddress() const override { return I2C_ADDR; }

    bool begin() override { _connected = true; return true; }
    bool isConnected() const override { return _connected; }

    uint8_t getValueCount() const override { return NUM_VALUES; }
    const SensorValueDescriptor* getValueDescriptor(uint8_t index) const override;

    bool read() override;
    SensorReading getValue(uint8_t index) const override;

private:
    SensorReading _readings[NUM_VALUES];
    bool _connected;
    float _counter;

    static const SensorValueDescriptor DESCRIPTORS[NUM_VALUES];
};
```

**Step 2: Create mock sensor implementation**

```cpp
// src/sensors/MockSensor.cpp
#include "MockSensor.h"
#include <Arduino.h>

const SensorValueDescriptor MockSensor::DESCRIPTORS[NUM_VALUES] = {
    {"mock_sin", "Wave", "", -1.0f, 1.0f, 0xF81F, true, 2},
    {"mock_cnt", "Count", "", 0.0f, 100.0f, 0xFFE0, false, 0}
};

MockSensor::MockSensor() : _connected(false), _counter(0) {
    for (uint8_t i = 0; i < NUM_VALUES; i++) {
        _readings[i] = SensorReading::invalid(DESCRIPTORS[i].id);
    }
}

const SensorValueDescriptor* MockSensor::getValueDescriptor(uint8_t index) const {
    if (index >= NUM_VALUES) return nullptr;
    return &DESCRIPTORS[index];
}

bool MockSensor::read() {
    if (!_connected) return false;

    uint32_t now = millis();
    _counter += 0.1f;
    if (_counter > 100.0f) _counter = 0;

    _readings[0] = SensorReading::make(DESCRIPTORS[0].id, sin(_counter));
    _readings[0].timestamp = now;

    _readings[1] = SensorReading::make(DESCRIPTORS[1].id, _counter);
    _readings[1].timestamp = now;

    return true;
}

SensorReading MockSensor::getValue(uint8_t index) const {
    if (index >= NUM_VALUES) return SensorReading::invalid("invalid");
    return _readings[index];
}
```

**Step 3: Test adding mock sensor (optional - just verify it compiles)**

Temporarily add to main.cpp setup():
```cpp
// #include "sensors/MockSensor.h"
// MockSensor mockSensor;
// registry.registerSensor(&mockSensor);  // Now shows 5 values!
```

**Step 4: Commit**

```bash
git add src/sensors/MockSensor.h src/sensors/MockSensor.cpp
git commit -m "feat: add MockSensor for testing extensibility

Validates that new sensors can be added with only:
- New source files
- One registration call

Co-Authored-By: Claude Opus 4.5 <noreply@anthropic.com>"
```

---

## Summary

After completing all tasks:

1. **Core abstractions** in `src/core/` - ISensor, SensorTypes, SensorRegistry
2. **Display system** in `src/display/` - DisplayManager, ValueBuffer
3. **Sensors** in `src/sensors/` - BME280Sensor, MockSensor
4. **Configuration** in `include/Config.h`
5. **Orchestration** in `src/main.cpp` - clean, minimal

**Adding a new sensor requires only:**
1. Create `sensors/NewSensor.h` and `.cpp`
2. Add `registry.registerSensor(&newSensor);` in main.cpp

Display adapts automatically to 1-6 values.
