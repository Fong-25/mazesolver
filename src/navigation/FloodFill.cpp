#include "FloodFill.h"

#include "../config/MazeConfig.h"
#include "Maze.h"

namespace {
    uint8_t distance[(uint16_t)MazeConfig::WIDTH * MazeConfig::HEIGHT];

    // Raw-cell bit layout is Maze's documented one (Maze.h): N,E,S,W,
    // visited, confirmed -- bit 0..5.
    constexpr uint8_t RAW_NORTH = 1 << 0;
    constexpr uint8_t RAW_EAST = 1 << 1;
    constexpr uint8_t RAW_SOUTH = 1 << 2;
    constexpr uint8_t RAW_WEST = 1 << 3;
    constexpr uint8_t RAW_VISITED = 1 << 4;

    uint16_t indexOf(uint8_t x, uint8_t y) {
        return (uint16_t)y * MazeConfig::WIDTH + x;
    }

    bool isGoalCell(uint8_t x, uint8_t y) {
        return x >= MazeConfig::GOAL_X_MIN && x <= MazeConfig::GOAL_X_MAX &&
               y >= MazeConfig::GOAL_Y_MIN && y <= MazeConfig::GOAL_Y_MAX;
    }

    // Wavefront relaxation: repeatedly sweep every cell, pulling any
    // neighbor's distance down to (this cell's distance + 1) across any
    // known-open wall, until a full sweep changes nothing. No BFS queue --
    // that's a real RAM cost at 16x16 on an ATmega328P.
    //
    // Speed matters here, not just tidiness: this runs BLOCKING (nothing
    // else in loop() -- control, IMU, watchdog kick -- runs meanwhile) once
    // per cell in EXPLORE. Measured on a simulated ATmega328P @16 MHz the
    // old version (4x hasWall()+neighborOf() per cell, always sweeping in
    // the same direction) took ~85 ms on an open 16x16, ~210 ms on a typical
    // perfect maze and up to ~360 ms on a serpentine -- against a 500 ms
    // watchdog. Two changes, identical results (checked cell-for-cell on
    // random mazes): (1) sweeps ALTERNATE direction (ascending then
    // descending cell order), so a wave moving against the scan order no
    // longer costs one whole sweep per cell of path; (2) each cell's walls
    // come from ONE getRawCell() call. Now ~5 ms open, ~40 ms typical,
    // ~60 ms worst measured.
    //
    // knownOnly: expand only INTO cells the robot has actually visited (or
    // goal cells, which are the sources and never expanded into). An
    // unvisited cell's own walls are mostly unknown -- Maze treats unknown
    // as open -- so a path through one is a guess, fine for EXPLORE (it
    // wants to find out) but not for FAST_RUN, which would drive it at
    // speed. See computeToGoalKnownOnly().
    //
    // Note: distances near 255 would collide with UNREACHABLE. Only
    // possible if a single known path already snakes through nearly every
    // one of the 256 cells -- not a real micromouse maze layout, so this
    // is a documented theoretical limit, not worth doubling the array to
    // uint16_t over.
    void relax(bool knownOnly) {
        constexpr uint8_t W = MazeConfig::WIDTH;
        constexpr uint8_t H = MazeConfig::HEIGHT;
        constexpr uint16_t N = (uint16_t)W * H;

        bool changed = true;
        bool ascending = true;
        while (changed) {
            changed = false;
            for (uint16_t n = 0; n < N; n++) {
                uint16_t i = ascending ? n : (uint16_t)(N - 1 - n);
                uint8_t d = distance[i];
                if (d == FloodFill::UNREACHABLE) continue;

                uint8_t x = i % W;
                uint8_t y = i / W;
                uint8_t walls = Maze::getRawCell(x, y);
                uint8_t nd = d + 1;

                // Bounds are checked even though the border is always
                // walled: a corrupt EEPROM map must not index out of range.
                for (uint8_t dir = 0; dir < 4; dir++) {
                    uint8_t wallBit;
                    uint16_t ni;
                    uint8_t nx = x, ny = y;
                    switch (dir) {
                        case 0:
                            if (y + 1 >= H) continue;
                            wallBit = RAW_NORTH;
                            ny = y + 1;
                            break;
                        case 1:
                            if (x + 1 >= W) continue;
                            wallBit = RAW_EAST;
                            nx = x + 1;
                            break;
                        case 2:
                            if (y == 0) continue;
                            wallBit = RAW_SOUTH;
                            ny = y - 1;
                            break;
                        default:
                            if (x == 0) continue;
                            wallBit = RAW_WEST;
                            nx = x - 1;
                            break;
                    }
                    if (walls & wallBit) continue;
                    ni = indexOf(nx, ny);
                    if (distance[ni] <= nd) continue;
                    if (knownOnly && !(Maze::getRawCell(nx, ny) & RAW_VISITED))
                        continue;
                    distance[ni] = nd;
                    changed = true;
                }
            }
            ascending = !ascending;
        }
    }

    void seedGoal() {
        for (uint8_t y = 0; y < MazeConfig::HEIGHT; y++) {
            for (uint8_t x = 0; x < MazeConfig::WIDTH; x++) {
                distance[indexOf(x, y)] =
                    isGoalCell(x, y) ? 0 : FloodFill::UNREACHABLE;
            }
        }
    }
}

namespace FloodFill {
    void computeToGoal() {
        seedGoal();
        relax(false);
    }

    void computeToGoalKnownOnly() {
        seedGoal();
        relax(true);
    }

    void computeToTarget(uint8_t targetX, uint8_t targetY) {
        for (uint16_t i = 0;
             i < (uint16_t)MazeConfig::WIDTH * MazeConfig::HEIGHT; i++) {
            distance[i] = UNREACHABLE;
        }
        if (Maze::isValidCell(targetX, targetY)) {
            distance[indexOf(targetX, targetY)] = 0;
        }
        relax(false);
    }

    uint8_t getDistance(uint8_t x, uint8_t y) {
        if (!Maze::isValidCell(x, y)) return UNREACHABLE;
        return distance[indexOf(x, y)];
    }
}