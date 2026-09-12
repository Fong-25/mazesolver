#pragma once

// OTHER HARDWARE RELATED
namespace RobotConfig {
    // MOTOR & ENCODER POLARITY
    constexpr bool LEFT_MOTOR_REVERSE = false;
    constexpr bool RIGHT_MOTOR_REVERSE = false;

    constexpr bool LEFT_ENCODER_REVERSE = false;
    constexpr bool RIGHT_ENCODER_REVERSE = false;

    // MECHANICAL GEOMETRY
    constexpr float WHEEL_DIAMETER_MM = 34.0f;  // TODO: confirm
    constexpr float WHEEL_BASE_MM = 72.0f;      // TODO: confirm
    constexpr float WHEEL_TRACK_MM = 72.0f;     // TODO: confirm

    // ENCODER
    // TODO: confirm gearbox ratio × CPR
    constexpr float ENCODER_PULSES_PER_OUTPUT_REV = 350.0f;
    // x4 quadrature decoding
    constexpr float ENCODER_COUNTS_PER_OUTPUT_REV =
        ENCODER_PULSES_PER_OUTPUT_REV * 4.0f;

    // MOTION LIMITS (CAN BE RAISED ONCE TUNED)
    constexpr float MAX_LINEAR_SPEED_MM_S = 800.0f;
    constexpr float MAX_LINEAR_ACCEL_MM_S2 = 1500.0f;
    constexpr float MAX_TURN_RATE_DEG_S = 500.0f;
}