#pragma once
#include <Arduino.h>

namespace RGB {
    void begin();
    void setColor(uint8_t r, uint8_t g, uint8_t b);
    void off();
}