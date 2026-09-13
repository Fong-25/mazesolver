#pragma once
#include <Arduino.h>

namespace IMU {
    // Requires Wire.begin() already called by caller (see SETUP_NOTES.md).
    bool begin();

    void update(uint32_t nowMs);

    // Call once during boot while the robot is stationary.
    void calibrateBias();

    void resetYawDeg();

    float getYawRateDegPerSec();  // signed, axis-mapped, bias-corrected
    float getYawDeg();            // integrated heading since last reset

    // Diagnostic only — not gated on, MPU6500 clones report inconsistent
    // WHO_AM_I values, so this is for you to sanity-check, not for begin()
    // to fail on.
    uint8_t getWhoAmI();
}