#pragma once
#include <Arduino.h>

// USER RELATED CONFIGURATION
namespace UserConfig {
    // BUTTON
    constexpr uint8_t BUTTON_LONG_PRESS_MS = 3000;  // 3 second
    constexpr uint8_t BUTTON_DEBOUNCE_MS = 30;  // TODO: Confirm feel on real SW

    // STANDBY BLINK INDICATION
    // TODO: confirm desired rate
    constexpr uint32_t STANDBY_BLINK_INTERVAL_MS = 500;

    // BATTERY
    constexpr uint32_t BATTERY_SAMPLE_INTERVAL_MS = 500;
    constexpr uint32_t DIP_DEBOUNCE_MS = 30;

    // TOF
    constexpr uint32_t TOF_POLL_INTERVAL_MS = 20;

    constexpr uint32_t BLUETOOTH_BAUD = 115200;
    constexpr uint32_t LOG_STREAM_INTERVAL_MS = 100;
    // RGB COLOR (placeholder — define per mode once ModeManager
    // exists)
    // TODO: assign real colors per state (SETTING / LOCK / STANDBY / RUN /
    // ERROR)

    // MAZE GOAL
    // TODO: goal cell coordinates depend on maze size convention

    // -------- Exploration / fast-run behavior --------
    // TODO: exploration speed cap, fast-run speed, sensor-covered-start
    // threshold — these depend on RobotConfig motion limits above, fill in once
    // MotorControl exists
}