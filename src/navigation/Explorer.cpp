#include "Explorer.h"

#include "../config/ControlConfig.h"
#include "../config/MazeConfig.h"
#include "../config/RobotConfig.h"
#include "../config/SensorConfig.h"
#include "../control/Motion.h"
#include "../control/MotorControl.h"
#include "../drivers/ToFManager.h"
#include "../system/Diagnostics.h"
#include "../system/Safety.h"
#include "FloodFill.h"
#include "Maze.h"
#include "MazePersistence.h"

namespace {
    enum class Phase : uint8_t { IDLE, DECIDE, TURNING, MOVING, DONE };
    enum class Leg : uint8_t { TO_GOAL, TO_START };

    Phase phase = Phase::IDLE;
    Leg leg = Leg::TO_GOAL;

    uint8_t cellX = 0, cellY = 0;
    Maze::Direction heading = Maze::Direction::NORTH;
    Maze::Direction pendingDir = Maze::Direction::NORTH;
    bool firstMove = true;

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

    // Front is sensed by EITHER angled front sensor -- deliberately OR,
    // not AND: a false "wall" only costs one wasted re-evaluation step; a
    // missed real wall costs a collision. Side walls only have one sensor
    // each, so there's no OR/AND choice to make there.
    bool senseFront() {
        using ToFManager::SensorRole;
        bool left = ToFManager::isValid(SensorRole::FRONT_LEFT) &&
                    ToFManager::getDistanceMm(SensorRole::FRONT_LEFT) <=
                        SensorConfig::FRONT_WALL_MM;
        bool right = ToFManager::isValid(SensorRole::FRONT_RIGHT) &&
                     ToFManager::getDistanceMm(SensorRole::FRONT_RIGHT) <=
                         SensorConfig::FRONT_WALL_MM;
        return left || right;
    }

    bool senseLeft() {
        using ToFManager::SensorRole;
        return ToFManager::isValid(SensorRole::DIAGONAL_LEFT) &&
               ToFManager::getDistanceMm(SensorRole::DIAGONAL_LEFT) <=
                   SensorConfig::SIDE_WALL_MAX_MM;
    }

    bool senseRight() {
        using ToFManager::SensorRole;
        return ToFManager::isValid(SensorRole::DIAGONAL_RIGHT) &&
               ToFManager::getDistanceMm(SensorRole::DIAGONAL_RIGHT) <=
                   SensorConfig::SIDE_WALL_MAX_MM;
    }

    // Scans the four world-frame directions out of (cellX,cellY) and
    // returns the best one to move into next, per spec section 29's
    // priority order (unvisited-reachable first, then lowest flood
    // distance). Returns false if nothing is reachable at all (fully
    // boxed in by known walls).
    bool pickNextDirection(Maze::Direction& outDir) {
        bool found = false;
        uint8_t bestDist = FloodFill::UNREACHABLE;
        bool bestUnvisited = false;
        Maze::Direction best = Maze::Direction::NORTH;

        for (uint8_t i = 0; i < 4; i++) {
            Maze::Direction dir = (Maze::Direction)i;
            if (Maze::hasWall(cellX, cellY, dir)) continue;

            uint8_t nx, ny;
            if (!Maze::neighborOf(cellX, cellY, dir, nx, ny)) continue;

            uint8_t dist = FloodFill::getDistance(nx, ny);
            if (dist == FloodFill::UNREACHABLE) continue;

            // Only meaningful on the TO_GOAL leg -- on TO_START, both
            // sides of this comparison are always false, so it collapses
            // to a plain lowest-distance pick (correct: the return leg
            // just wants the known shortest path home).
            bool unvisited = (leg == Leg::TO_GOAL) && !Maze::isVisited(nx, ny);

            bool better;
            if (!found) {
                better = true;
            } else if (unvisited != bestUnvisited) {
                better = unvisited && !bestUnvisited;
            } else {
                better = dist < bestDist;
            }

            if (better) {
                found = true;
                best = dir;
                bestDist = dist;
                bestUnvisited = unvisited;
            }
        }

        if (found) outDir = best;
        return found;
    }

    void enterTurnOrMove(Maze::Direction targetDir) {
        pendingDir = targetDir;
        if (targetDir == heading) {
            phase = Phase::MOVING;
            Motion::moveForwardCell(ControlConfig::FORWARD_BASE_SPEED_MM_S,
                                    nextForwardDistance());
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
        // 1-2. Sense, convert robot-relative readings to world-frame
        // walls using the current heading.
        bool front = senseFront();
        bool left = senseLeft();
        bool right = senseRight();

        Maze::setWall(cellX, cellY, heading, front);
        Maze::setWall(cellX, cellY, turnLeftOf(heading), left);
        Maze::setWall(cellX, cellY, turnRightOf(heading), right);
        // Back is never sensed (no rear sensor) -- stays whatever it was
        // already known as: open, from the move that got us into this
        // cell, or unknown if this is the very first cell of the run.
        Maze::setVisited(cellX, cellY);

        // 3. Recompute flood-fill for whichever leg we're on.
        if (leg == Leg::TO_GOAL) {
            FloodFill::computeToGoal();
        } else {
            FloodFill::computeToTarget(MazeConfig::START_X,
                                       MazeConfig::START_Y);
        }

        // 4. Goal / home check.
        if (leg == Leg::TO_GOAL && isGoalCell(cellX, cellY)) {
            leg = Leg::TO_START;
            // "Successful exploration complete" -- spec section 48's
            // first save trigger. The second (explicit save command) is
            // a small follow-on for Bluetooth.cpp, not added yet.
            MazePersistence::save();
            FloodFill::computeToTarget(MazeConfig::START_X,
                                       MazeConfig::START_Y);
        }
        if (leg == Leg::TO_START && cellX == MazeConfig::START_X &&
            cellY == MazeConfig::START_Y) {
            Motion::stop();
            MotorControl::disable();  // run's over -- never drive outside
                                      // an active RUNNING dispatch
            phase = Phase::DONE;
            return;
        }

        // 5-6. Pick and issue the next move.
        Maze::Direction dir;
        if (!pickNextDirection(dir)) {
            // Fully boxed in by known walls -- shouldn't happen on a
            // valid maze; treat it as a software fault rather than
            // spinning here forever.
            Safety::triggerFault(Diagnostics::ErrorCode::SOFTWARE_FAULT);
            // Safety::trip() already disables MotorControl -- no need to
            // repeat it here.
            phase = Phase::DONE;
            return;
        }
        enterTurnOrMove(dir);
    }
}

namespace Explorer {
    void begin() {
        cellX = MazeConfig::START_X;
        cellY = MazeConfig::START_Y;
        // TODO: confirm the robot's physical start orientation actually
        // matches "north" the way Maze/MazeConfig define it.
        heading = Maze::Direction::NORTH;
        leg = Leg::TO_GOAL;
        phase = Phase::DECIDE;
        firstMove = true;
        MotorControl::enable();  // the one moment real motion is actually
                                 // authorized -- matches the "never drive
                                 // just because" boot-time disable() in
                                 // main.cpp
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
                    Motion::moveForwardCell(
                        ControlConfig::FORWARD_BASE_SPEED_MM_S,
                        nextForwardDistance());
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