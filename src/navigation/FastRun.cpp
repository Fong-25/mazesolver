#include "FastRun.h"

#include "../config/ControlConfig.h"
#include "../config/MazeConfig.h"
#include "../config/RobotConfig.h"
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
    uint8_t pendingSteps = 1;
    bool firstMove = true;

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

    // The very first forward primitive of a run needs a shorter distance
    // than a full cell -- see RobotConfig::FIRST_MOVE_DISTANCE_MM's
    // comment. Every move after that is a normal full cell.
    float nextForwardDistance() {
        if (firstMove) {
            firstMove = false;
            return RobotConfig::FIRST_MOVE_DISTANCE_MM;
        }
        return RobotConfig::CELL_SIZE_MM;
    }

    // Pure greedy descent along the already-known map, parameterized on
    // an explicit (x,y) rather than reading cellX/cellY directly, so
    // countStraightRun() below can look ahead without touching real
    // state.
    bool pickNextDirection(uint8_t x, uint8_t y, Maze::Direction& outDir) {
        bool found = false;
        uint8_t bestDist = FloodFill::UNREACHABLE;
        Maze::Direction best = Maze::Direction::NORTH;

        for (uint8_t i = 0; i < 4; i++) {
            Maze::Direction dir = (Maze::Direction)i;
            if (Maze::hasWall(x, y, dir)) continue;

            uint8_t nx, ny;
            if (!Maze::neighborOf(x, y, dir, nx, ny)) continue;

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

    // Counts how many consecutive cells starting at (cellX,cellY)
    // continue straight via `dir` before either a turn becomes the
    // better choice or the goal is reached. Never counts past the goal
    // cell -- FastRun has to actually stop there, not run through it.
    // Always returns at least 1.
    uint8_t countStraightRun(Maze::Direction dir) {
        uint8_t steps = 0;
        uint8_t x = cellX, y = cellY;

        while (true) {
            if (Maze::hasWall(x, y, dir)) break;

            uint8_t nx, ny;
            if (!Maze::neighborOf(x, y, dir, nx, ny)) break;

            steps++;
            x = nx;
            y = ny;

            if (isGoalCell(x, y)) break;

            Maze::Direction nextDir;
            if (!pickNextDirection(x, y, nextDir)) break;
            if (nextDir != dir) break;
        }

        return steps == 0 ? 1 : steps;
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
        if (!pickNextDirection(cellX, cellY, dir)) {
            // No known path to the goal from here -- either the loaded
            // map is incomplete/stale or something's wrong with it. Same
            // treatment as Explorer's boxed-in case.
            Safety::triggerFault(Diagnostics::ErrorCode::SOFTWARE_FAULT);
            // Safety::trip() already disables MotorControl.
            phase = Phase::DONE;
            return;
        }

        if (dir != heading) {
            // Turn first -- re-DECIDE once heading catches up, which
            // naturally re-evaluates whether a straight run opens up
            // from the new heading rather than assuming just one cell.
            pendingDir = dir;
            phase = Phase::TURNING;
            if (dir == turnLeftOf(heading)) {
                Motion::turnLeft90();
            } else if (dir == turnRightOf(heading)) {
                Motion::turnRight90();
            } else {
                Motion::turn180();
            }
            return;
        }

        // Straight continuation -- batch as many consecutive
        // known-reachable cells as possible into one primitive instead of
        // stopping at every cell boundary. This is the whole point of
        // FastRun over Explorer: no sensing needed mid-path, so there's
        // no reason to stop until a turn is actually required.
        uint8_t steps = countStraightRun(dir);
        pendingSteps = steps;
        float distance = nextForwardDistance() +
                         (float)(steps - 1) * RobotConfig::CELL_SIZE_MM;
        phase = Phase::MOVING;
        Motion::moveForwardCell(ControlConfig::FAST_RUN_SPEED_MM_S, distance);
    }
}

namespace FastRun {
    void begin() {
        cellX = MazeConfig::START_X;
        cellY = MazeConfig::START_Y;
        // Same start-orientation assumption as Explorer -- keep the two
        // in sync if that ever changes.
        heading = Maze::Direction::NORTH;
        firstMove = true;
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
                    phase = Phase::DECIDE;
                }
                break;

            case Phase::MOVING:
                if (!Motion::isBusy()) {
                    uint8_t x = cellX, y = cellY;
                    for (uint8_t i = 0; i < pendingSteps; i++) {
                        uint8_t nx, ny;
                        if (!Maze::neighborOf(x, y, heading, nx, ny)) break;
                        x = nx;
                        y = ny;
                    }
                    cellX = x;
                    cellY = y;
                    phase = Phase::DECIDE;
                }
                break;
        }
    }

    bool isDone() { return phase == Phase::DONE; }
}