#include "FastRun.h"

#include "../config/ControlConfig.h"
#include "../config/MazeConfig.h"
#include "../control/Motion.h"
#include "../control/MotorControl.h"
#include "../system/Diagnostics.h"
#include "../system/Safety.h"
#include "FloodFill.h"
#include "Maze.h"

namespace {
    enum class Phase : uint8_t { IDLE, DECIDE, TURNING, MOVING, DONE };

    Phase phase = Phase::IDLE;
    uint8_t cellX = 0, cellY = 0;
    Maze::Direction heading = Maze::Direction::NORTH;
    Maze::Direction pendingDir = Maze::Direction::NORTH;

    Maze::Direction turnLeftOf(Maze::Direction d) {
        return (Maze::Direction)(((uint8_t)d + 3) % 4);
    }
    Maze::Direction turnRightOf(Maze::Direction d) {
        return (Maze::Direction)(((uint8_t)d + 1) % 4);
    }

    bool isGoalCell(uint8_t x, uint8_t y) {
        return x >= MazeConfig::GOAL_X_MIN && x <= MazeConfig::GOAL_X_MAX &&
               y >= MazeConfig::GOAL_Y_MIN && y <= MazeConfig::GOAL_Y_MAX;
    }

    // Pure greedy descent along the already-known map -- no unvisited
    // preference like Explorer's TO_GOAL leg has, since the whole map is
    // assumed known already; following the shortest known path is
    // FastRun's entire job.
    bool pickNextDirection(Maze::Direction& outDir) {
        bool found = false;
        uint8_t bestDist = FloodFill::UNREACHABLE;
        Maze::Direction best = Maze::Direction::NORTH;

        for (uint8_t i = 0; i < 4; i++) {
            Maze::Direction dir = (Maze::Direction)i;
            if (Maze::hasWall(cellX, cellY, dir)) continue;

            uint8_t nx, ny;
            if (!Maze::neighborOf(cellX, cellY, dir, nx, ny)) continue;

            uint8_t dist = FloodFill::getDistance(nx, ny);
            if (dist == FloodFill::UNREACHABLE) continue;

            if (!found || dist < bestDist) {
                found = true;
                best = dir;
                bestDist = dist;
            }
        }

        if (found) outDir = best;
        return found;
    }

    void enterTurnOrMove(Maze::Direction targetDir) {
        pendingDir = targetDir;
        if (targetDir == heading) {
            phase = Phase::MOVING;
            Motion::moveForwardCell(ControlConfig::FAST_RUN_SPEED_MM_S);
        } else if (targetDir == turnLeftOf(heading)) {
            phase = Phase::TURNING;
            Motion::turnLeft90();
        } else if (targetDir == turnRightOf(heading)) {
            phase = Phase::TURNING;
            Motion::turnRight90();
        } else {
            phase = Phase::TURNING;
            Motion::turn180();
        }
    }

    void doDecide() {
        if (isGoalCell(cellX, cellY)) {
            Motion::stop();
            MotorControl::disable();  // run's over -- never drive outside
                                      // an active RUNNING dispatch
            phase = Phase::DONE;
            return;
        }

        Maze::Direction dir;
        if (!pickNextDirection(dir)) {
            // No known path to the goal from here -- either the loaded
            // map is incomplete/stale or something's wrong with it. Same
            // treatment as Explorer's boxed-in case.
            Safety::triggerFault(Diagnostics::ErrorCode::SOFTWARE_FAULT);
            // Safety::trip() already disables MotorControl.
            phase = Phase::DONE;
            return;
        }
        enterTurnOrMove(dir);
    }
}

namespace FastRun {
    void begin() {
        cellX = MazeConfig::START_X;
        cellY = MazeConfig::START_Y;
        // Same start-orientation assumption as Explorer -- keep the two
        // in sync if that ever changes.
        heading = Maze::Direction::NORTH;
        FloodFill::computeToGoal();
        phase = Phase::DECIDE;
        MotorControl::enable();
    }

    void update(uint32_t nowMs) {
        (void)nowMs;
        switch (phase) {
            case Phase::IDLE:
            case Phase::DONE:
                return;

            case Phase::DECIDE:
                doDecide();
                break;

            case Phase::TURNING:
                if (!Motion::isBusy()) {
                    heading = pendingDir;
                    phase = Phase::MOVING;
                    Motion::moveForwardCell(ControlConfig::FAST_RUN_SPEED_MM_S);
                }
                break;

            case Phase::MOVING:
                if (!Motion::isBusy()) {
                    uint8_t nx, ny;
                    if (Maze::neighborOf(cellX, cellY, heading, nx, ny)) {
                        cellX = nx;
                        cellY = ny;
                    }
                    phase = Phase::DECIDE;
                }
                break;
        }
    }

    bool isDone() { return phase == Phase::DONE; }
}