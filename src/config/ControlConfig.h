#pragma once

// MOSTLY PID GAINS, ONLY PLACEHOLDER
namespace ControlConfig {
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
}