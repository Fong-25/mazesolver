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

    // MOTION PRIMITIVES (Motion.cpp) -- all first-pass placeholders, TODO
    // tune on the real robot.
    constexpr float FORWARD_BASE_SPEED_MM_S = 300.0f;
    // Higher-speed pass once the map's already known (FastRun) -- no
    // sensing to wait on mid-cell, so this can run well above the
    // EXPLORE baseline. TODO: tune once EXPLORE-speed heading-hold is
    // confirmed stable; raise this incrementally, don't jump straight to
    // whatever the motors can physically do.
    constexpr float FAST_RUN_SPEED_MM_S = 500.0f;
    // Clamp on the heading-hold PID's output -- added to/subtracted from
    // FORWARD_BASE_SPEED_MM_S per wheel, so this is how hard a straight
    // segment will fight to correct a heading error.
    constexpr float HEADING_CORRECTION_LIMIT_MM_S = 200.0f;

    constexpr float TURN_SPEED_DEG_S = 180.0f;
    constexpr float TURN_ANGLE_TOLERANCE_DEG = 2.0f;

    // Forward test distance for MOTION_TEST's ACCEL/CRUISE/DECEL --
    // deliberately longer than one CELL_SIZE_MM (180mm), since the whole
    // point of CRUISE in particular is isolating steady-state PID
    // behavior, which needs real distance at speed after the initial
    // transient settles out. TODO: tune -- needs to fit whatever open
    // bench space is actually available for this test.
    constexpr float MOTION_TEST_FORWARD_DISTANCE_MM = 720.0f;  // 4 cells

    // Both primitives end with target speed 0 held for this long before
    // reporting complete -- lets the wheel PID actually settle instead of
    // the next primitive getting armed on top of leftover momentum.
    constexpr uint32_t MOTION_SETTLE_MS = 80;
}