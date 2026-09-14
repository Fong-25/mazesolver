#pragma once
#include <Arduino.h>

namespace Diagnostics {
    enum class ErrorCode : uint8_t {
        NONE,
        TOF_INIT_FAILED,
        IMU_INIT_FAILED,
        ENCODER_INVALID,
        BATTERY_LOW,
        BATTERY_CRITICAL,
        MAP_INVALID,
        MOTOR_TIMEOUT,
        START_SEQUENCE_TIMEOUT,
        // Extensions beyond your spec's own example list — added because
        // section 43 names these as stop conditions but section 53's
        // example enum didn't include codes for them yet.
        WATCHDOG_TIMEOUT,
        USER_ABORT,
        SOFTWARE_FAULT
    };

    void begin();

    // Single active slot — matches spec's "the current error" (singular),
    // not a severity-ranked queue.
    void setError(ErrorCode code);
    ErrorCode getError();
    bool hasError();

    void clearError();  // explicit recovery action only, never automatic
}