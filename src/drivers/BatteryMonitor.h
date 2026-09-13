#pragma once
#include <Arduino.h>

namespace BatteryMonitor {
    void begin();
    void update(uint32_t nowMs);

    float getVoltage();  // filtered, calibrated voltage
    bool isLow();
    bool isCritical();
}