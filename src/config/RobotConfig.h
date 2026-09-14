#pragma once

// OTHER HARDWARE RELATED
namespace RobotConfig {
    // MOTOR & ENCODER
    constexpr bool LEFT_MOTOR_REVERSED = false;
    constexpr bool RIGHT_MOTOR_REVERSED = false;

    constexpr bool LEFT_ENCODER_REVERSED = false;
    constexpr bool RIGHT_ENCODER_REVERSED = false;

    // How much headroom before a measured speed is treated as a sensor glitch
    // rather than a real (if aggressive) motion. 1.5x max speed is generous —
    // tighten it only if you see false positives during hard runs.
    constexpr float ENCODER_GLITCH_SAFETY_MARGIN = 1.5f;

    // MECHANICAL GEOMETRY
    constexpr float LEFT_WHEEL_DIAMETER_MM =
        34.0f;  // TODO: measure independently
    constexpr float RIGHT_WHEEL_DIAMETER_MM =
        34.0f;  // TODO: measure independently

    constexpr float WHEEL_BASE_MM = 72.0f;   // TODO: confirm
    constexpr float WHEEL_TRACK_MM = 72.0f;  // TODO: confirm

    // TOF
    constexpr uint8_t TOF_INSTALLED_COUNT = 4;

    constexpr uint8_t TOF_ROLE_INDEX_FRONT_LEFT = 0;
    constexpr uint8_t TOF_ROLE_INDEX_FRONT_RIGHT = 1;
    constexpr uint8_t TOF_ROLE_INDEX_DIAGONAL_LEFT = 2;
    constexpr uint8_t TOF_ROLE_INDEX_DIAGONAL_RIGHT = 3;

    // IMU
    enum class ImuAxis : uint8_t { X, Y, Z };
    // Verify against the assembled robot — flip AXIS or SIGN, don't touch
    // IMU.cpp.
    constexpr ImuAxis IMU_YAW_AXIS = ImuAxis::Z;
    constexpr int8_t IMU_YAW_SIGN = +1;

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

    // BATTERY
    // TODO: confirm for 2S pack
    constexpr float BATTERY_LOW_VOLTAGE = 7.0f;
    constexpr float BATTERY_CRITICAL_VOLTAGE = 6.6f;  // TODO: confirm
    constexpr float BATTERY_FILTER_ALPHA = 0.1f;  // exponential filter weight
}