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

    // Boot self-check results, set once in main.cpp's setup() and never
    // touched again. These exist so STATUS can report them ON DEMAND,
    // any time something connects -- the boot self-check lines printed
    // during setup() (a bare OK, plus TOF FAIL/IMU FAIL only when a sensor
    // failed) are easy to miss over Bluetooth (HC-06 and
    // a reconnecting terminal both need real time after power-up, which
    // setup() doesn't wait for). Query STATUS instead of racing to catch
    // the live boot text.
    void setSelfCheck(bool tofOk, bool imuOk);
    bool tofSelfCheckOk();
    bool imuSelfCheckOk();
}