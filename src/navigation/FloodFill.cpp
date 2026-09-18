#include "FloodFill.h"

#include "../config/MazeConfig.h"
#include "Maze.h"

namespace {
    uint8_t distance[(uint16_t)MazeConfig::WIDTH * MazeConfig::HEIGHT];

    uint16_t indexOf(uint8_t x, uint8_t y) {
        return (uint16_t)y * MazeConfig::WIDTH + x;
    }

    bool isGoalCell(uint8_t x, uint8_t y) {
        return x >= MazeConfig::GOAL_X_MIN && x <= MazeConfig::GOAL_X_MAX &&
               y >= MazeConfig::GOAL_Y_MIN && y <= MazeConfig::GOAL_Y_MAX;
    }

    // Wavefront relaxation: repeatedly scan every cell, pulling any
    // neighbor's distance down to (this cell's distance + 1) across any
    // known-open wall, until a full pass changes nothing. No BFS queue
    // needed -- a real RAM cost at 16x16 on an ATmega328P -- at the cost
    // of worst-case (maze diameter) passes over 256 cells x 4 directions.
    // Done on demand, never every loop tick, so that trade is fine here.
    //
    // Note: distances near 255 would collide with UNREACHABLE. Only
    // possible if a single known path already snakes through nearly every
    // one of the 256 cells -- not a real micromouse maze layout, so this
    // is a documented theoretical limit, not something worth doubling the
    // array to uint16_t over.
    void relax() {
        bool changed = true;
        while (changed) {
            changed = false;
            for (uint8_t y = 0; y < MazeConfig::HEIGHT; y++) {
                for (uint8_t x = 0; x < MazeConfig::WIDTH; x++) {
                    uint8_t d = distance[indexOf(x, y)];
                    if (d == FloodFill::UNREACHABLE) continue;

                    for (uint8_t dirIdx = 0; dirIdx < 4; dirIdx++) {
                        Maze::Direction dir = (Maze::Direction)dirIdx;
                        if (Maze::hasWall(x, y, dir)) continue;

                        uint8_t nx, ny;
                        if (!Maze::neighborOf(x, y, dir, nx, ny)) continue;

                        uint16_t ni = indexOf(nx, ny);
                        if (distance[ni] > (uint8_t)(d + 1)) {
                            distance[ni] = d + 1;
                            changed = true;
                        }
                    }
                }
            }
        }
    }
}

namespace FloodFill {
    void computeToGoal() {
        for (uint8_t y = 0; y < MazeConfig::HEIGHT; y++) {
            for (uint8_t x = 0; x < MazeConfig::WIDTH; x++) {
                distance[indexOf(x, y)] = isGoalCell(x, y) ? 0 : UNREACHABLE;
            }
        }
        relax();
    }

    void computeToTarget(uint8_t targetX, uint8_t targetY) {
        for (uint16_t i = 0;
             i < (uint16_t)MazeConfig::WIDTH * MazeConfig::HEIGHT; i++) {
            distance[i] = UNREACHABLE;
        }
        if (Maze::isValidCell(targetX, targetY)) {
            distance[indexOf(targetX, targetY)] = 0;
        }
        relax();
    }

    uint8_t getDistance(uint8_t x, uint8_t y) {
        if (!Maze::isValidCell(x, y)) return UNREACHABLE;
        return distance[indexOf(x, y)];
    }
}