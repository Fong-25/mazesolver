#pragma once
#include <Arduino.h>

// USER RELATED CONFIGURATION
namespace UserConfig {
    // BUTTON
    constexpr uint32_t BUTTON_LONG_PRESS_MS = 3000;  // 3 second
    constexpr uint32_t BUTTON_DEBOUNCE_MS =
        30;  // TODO: Confirm feel on real SW

    // STANDBY BLINK INDICATION
    // TODO: confirm desired rate
    constexpr uint32_t STANDBY_BLINK_INTERVAL_MS = 500;

    // BATTERY
    constexpr uint32_t BATTERY_SAMPLE_INTERVAL_MS = 500;
    constexpr uint32_t DIP_DEBOUNCE_MS = 30;

    // TOF
    constexpr uint32_t TOF_POLL_INTERVAL_MS = 20;
    // Which ToF triggers RUN start
    // 0=FRONT_LEFT, 1=FRONT_RIGHT, 2=DIAGONAL_LEFT, 3=DIAGONAL_RIGHT
    constexpr uint8_t START_SENSOR_ROLE_INDEX = 0;

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

    enum class WheelGesture : uint8_t { FORWARD, BACKWARD };

    constexpr uint8_t MODE_SEQUENCE_LENGTH = 3;

    struct ModeSequenceEntry {
        uint8_t modeIndex;  // matches Bluetooth's MODE <n> numbering
        WheelGesture sequence[MODE_SEQUENCE_LENGTH];
    };

    constexpr ModeSequenceEntry MODE_SEQUENCES[] = {
        {0,
         {WheelGesture::FORWARD, WheelGesture::FORWARD,
          WheelGesture::BACKWARD}},  // EXPLORE
        {1,
         {WheelGesture::FORWARD, WheelGesture::BACKWARD,
          WheelGesture::FORWARD}},  // FAST_RUN
        {2,
         {WheelGesture::BACKWARD, WheelGesture::BACKWARD,
          WheelGesture::FORWARD}},  // DIAGNOSTIC
        {3,
         {WheelGesture::FORWARD, WheelGesture::FORWARD,
          WheelGesture::FORWARD}},  // DEBUG_LOG
        {4,
         {WheelGesture::BACKWARD, WheelGesture::BACKWARD,
          WheelGesture::BACKWARD}},  // MOTION_TEST
    };
    constexpr uint8_t MODE_SEQUENCE_COUNT =
        sizeof(MODE_SEQUENCES) / sizeof(MODE_SEQUENCES[0]);

    constexpr uint32_t MODE_GESTURE_TIMEOUT_MS = 2000;
}