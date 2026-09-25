#pragma once
#include <stdint.h>

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

    constexpr float WHEEL_BASE_MM = 72.0f;  // TODO: confirm
    // WHEEL_TRACK_MM is unused currently
    constexpr float WHEEL_TRACK_MM = 72.0f;  // TODO: confirm

    // Classic micromouse cell pitch
    constexpr float CELL_SIZE_MM = 180.0f;

    // Extra forward travel needed at the very start of a run to bring the
    // robot from where it's actually parked to the true center of the
    // start cell. If the robot starts with its rear against the start
    // cell's back wall, its center of rotation is ROBOT_CENTER_TO_REAR_MM
    // from that wall, not CELL_SIZE_MM/2. Every move after the first is
    // just a fixed CELL_SIZE_MM delta from wherever the robot actually
    // is, so without this the offset would carry into every "cell center"
    // for the rest of the run.
    //
    // FIRST_MOVE_DISTANCE_MM = CELL_SIZE_MM/2 - ROBOT_CENTER_TO_REAR_MM
    //
    // This is an ADDITION to the first cell move, not a replacement for
    // it: EXPLORE drives it as a standalone alignment move before its
    // first sense; FAST_RUN adds it onto the first straight run
    // (FIRST_MOVE_DISTANCE_MM + n * CELL_SIZE_MM). Either way the cell
    // counter doesn't change from it -- the robot is still "in" the start
    // cell, just centered now.
    //
    // TODO: measure ROBOT_CENTER_TO_REAR_MM (rear bumper to the
    // wheel-axle centerline; procedure in SETUP_NOTES.md section A). The
    // 0.0f placeholder assumes the axle is AT the rear wall, which is
    // almost certainly wrong -- this MUST be corrected before trusting
    // alignment on a real run.
    constexpr float ROBOT_CENTER_TO_REAR_MM = 0.0f;
    constexpr float FIRST_MOVE_DISTANCE_MM =
        (CELL_SIZE_MM / 2.0f) - ROBOT_CENTER_TO_REAR_MM;
    // A negative distance would mean the axle already sits past the cell
    // center at the wall -- a mismeasurement, not a valid geometry.
    static_assert(FIRST_MOVE_DISTANCE_MM >= 0.0f,
                  "ROBOT_CENTER_TO_REAR_MM must be <= CELL_SIZE_MM / 2");

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

    // which wheel drives gesture input
    constexpr bool MODE_SELECT_USE_LEFT_WHEEL = true;

    // Quarter wheel revolution per gesture
    constexpr float MODE_GESTURE_MIN_COUNTS =
        ENCODER_COUNTS_PER_OUTPUT_REV / 4.0f;
}