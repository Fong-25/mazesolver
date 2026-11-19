#pragma once
#include <stdint.h>

namespace MazeConfig {
    constexpr uint8_t WIDTH = 16;
    constexpr uint8_t HEIGHT = 16;

    // Classic micromouse convention: start bottom-left corner, goal is the
    // 2x2 block at the maze center. Both configurable for non-standard
    // mazes -- nothing downstream should hardcode these.
    constexpr uint8_t START_X = 0;
    constexpr uint8_t START_Y = 0;

    constexpr uint8_t GOAL_X_MIN = WIDTH / 2 - 1;
    constexpr uint8_t GOAL_X_MAX = WIDTH / 2;
    constexpr uint8_t GOAL_Y_MIN = HEIGHT / 2 - 1;
    constexpr uint8_t GOAL_Y_MAX = HEIGHT / 2;
}