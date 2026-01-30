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
