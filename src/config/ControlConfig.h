#pragma once
#include <stdint.h>

// MOSTLY PID GAINS, ONLY PLACEHOLDER
namespace ControlConfig {
    // Symmetric clamp on the PID integral term — prevents windup during
    // saturation (e.g. commanded speed the motor physically can't reach).
    constexpr float WHEEL_PID_INTEGRAL_LIMIT = 150.0f;

    // 1.0 = no derivative filtering. Lower = more smoothing, more lag.
    constexpr float WHEEL_PID_DERIVATIVE_FILTER_ALPHA = 1.0f;

    // Minimum PWM needed to actually overcome static friction and move —
    // per spec section 10.2. TODO: measure empirically (see SETUP_NOTES.md).
    constexpr int16_t MOTOR_DEADBAND_PWM = 30;

    // WHEEL VELOCITY
    constexpr float LEFT_KP = 1.0f;
    constexpr float LEFT_KI = 0.0f;
    constexpr float LEFT_KD = 0.0f;

    constexpr float RIGHT_KP = 1.0f;
    constexpr float RIGHT_KI = 0.0f;
    constexpr float RIGHT_KD = 0.0f;

    // HEADING
    constexpr float HEADING_KP = 1.0f;
    constexpr float HEADING_KI = 0.0f;
    constexpr float HEADING_KD = 0.0f;

    // WALL CENTERING
    constexpr float WALL_KP = 1.0f;
    constexpr float WALL_KI = 0.0f;
    constexpr float WALL_KD = 0.0f;

    constexpr uint32_t CONTROL_PERIOD_US = 2000;  // 2ms
    constexpr uint32_t IMU_PERIOD_US = 5000;      // 5ms
}