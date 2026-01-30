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
    for (uint8_t i = 0; i < HISTORY_SIZE; i++) {
        _buffer[i] = 0;
    }
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
