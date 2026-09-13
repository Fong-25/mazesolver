#pragma once
#include <Arduino.h>

namespace ToFManager {
    enum class SensorRole : uint8_t {
        FRONT_LEFT,
        FRONT_RIGHT,
        DIAGONAL_LEFT,
        DIAGONAL_RIGHT,
        ROLE_COUNT
    };

    // Returns false if any INSTALLED sensor failed to initialize.
    bool begin();

    // Call every loop tick. Internally rate-limited AND round-robins exactly
    // one physical sensor per call — a slow/stuck I2C transaction can only
    // ever cost one sensor's worth of time, never all four in a row.
    void update(uint32_t nowMs);

    // Last known reading in mm. Returns "clear" (max valid distance) if the
    // role isn't populated or hasn't produced a reading yet.
    uint16_t getDistanceMm(SensorRole role);

    // True only if the sensor is installed AND its last reading was in-range.
    bool isValid(SensorRole role);
}