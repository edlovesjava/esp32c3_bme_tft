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
        if (descriptors[i] == nullptr) continue;
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
