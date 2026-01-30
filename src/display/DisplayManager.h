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
