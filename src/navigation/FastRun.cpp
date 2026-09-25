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
    enum class Phase : uint8_t {
        IDLE,
        DECIDE,
        ALIGNING,
        TURNING,
        MOVING,
        DONE
    };

    Phase phase = Phase::IDLE;
    uint8_t cellX = 0, cellY = 0;
    Maze::Direction heading = Maze::Direction::NORTH;
    Maze::Direction pendingDir = Maze::Direction::NORTH;

    // How many consecutive cells the pending forward move covers in one
    // Motion primitive (straight-run batching, spec section 33). Set by
    // doDecide(), consumed by startMove() and by the MOVING completion
    // (which advances cellX/cellY by this many cells).
    uint8_t runCells = 1;

    // True until the run's first forward travel has been issued -- that
    // travel also carries RobotConfig::FIRST_MOVE_DISTANCE_MM of extra
    // distance to bring the robot from "parked against the start cell's
    // rear wall" to the start cell's true center.
    bool alignPending = false;

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
    //
    // Takes the cell to decide FROM (not just the robot's current one) so
    // countStraightRun() can look ahead with the exact same rule the real
    // step-by-step decision would use. `prefer` breaks ties between
    // equal-distance neighbors: going straight over turning when the path
    // length is the same -- fewer turns is strictly faster on a timed run,
    // and it's also what makes the lookahead agree with the real decision
    // (at every cell of a straight run, heading == the run's direction).
    bool pickNextDirection(uint8_t x, uint8_t y, Maze::Direction prefer,
                           Maze::Direction& outDir) {
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

            if (!found || dist < bestDist ||
                (dist == bestDist && dir == prefer)) {
                found = true;
                best = dir;
                bestDist = dist;
            }
        }

        if (found) outDir = best;
        return found;
    }

    // Starting with one step from (cellX,cellY) in `dir`, keeps extending
    // while the path's next decision is "same direction again". Stops at
    // the first goal cell (the run ends on entering it, same as before
    // batching), at any turn, and at a hard cap of WIDTH+HEIGHT steps --
    // greedy descent strictly decreases distance every step so it can't
    // loop, but a corrupt map shouldn't be able to hang the control loop
    // either.
    uint8_t countStraightRun(Maze::Direction dir) {
        uint8_t x = cellX, y = cellY;
        uint8_t n = 0;

        while (n < MazeConfig::WIDTH + MazeConfig::HEIGHT) {
            uint8_t nx, ny;
            if (!Maze::neighborOf(x, y, dir, nx, ny)) break;
            x = nx;
            y = ny;
            n++;

            if (isGoalCell(x, y)) break;

            Maze::Direction next;
            if (!pickNextDirection(x, y, dir, next) || next != dir) break;
        }
        return n;
    }

    void startMove() {
        float distanceMm = runCells * RobotConfig::CELL_SIZE_MM;
        if (alignPending) {
            distanceMm += RobotConfig::FIRST_MOVE_DISTANCE_MM;
            alignPending = false;
        }
        Motion::moveForwardCell(ControlConfig::FAST_RUN_SPEED_MM_S, distanceMm);
    }

    void enterTurnOrMove(Maze::Direction targetDir) {
        pendingDir = targetDir;
        if (targetDir == heading) {
            phase = Phase::MOVING;
            startMove();
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
        if (!pickNextDirection(cellX, cellY, heading, dir)) {
            // No known path to the goal from here -- either the loaded
            // map is incomplete/stale or something's wrong with it. Same
            // treatment as Explorer's boxed-in case.
            Safety::triggerFault(Diagnostics::ErrorCode::SOFTWARE_FAULT);
            // Safety::trip() already disables MotorControl.
            phase = Phase::DONE;
            return;
        }

        // First travel of the run, but the path needs a turn before
        // going anywhere: turning in place from the un-centered start
        // position would put every later "cell center" off, so do the
        // alignment as its own short move first, then decide again.
        if (alignPending && dir != heading) {
            alignPending = false;
            phase = Phase::ALIGNING;
            Motion::moveForwardCell(ControlConfig::FORWARD_BASE_SPEED_MM_S,
                                    RobotConfig::FIRST_MOVE_DISTANCE_MM);
            return;
        }

        runCells = countStraightRun(dir);
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
        runCells = 1;
        alignPending = RobotConfig::FIRST_MOVE_DISTANCE_MM > 0.0f;
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

            case Phase::ALIGNING:
                if (!Motion::isBusy()) phase = Phase::DECIDE;
                break;

            case Phase::TURNING:
                if (!Motion::isBusy()) {
                    heading = pendingDir;
                    phase = Phase::MOVING;
                    startMove();
                }
                break;

            case Phase::MOVING:
                if (!Motion::isBusy()) {
                    for (uint8_t i = 0; i < runCells; i++) {
                        uint8_t nx, ny;
                        if (!Maze::neighborOf(cellX, cellY, heading, nx, ny))
                            break;
                        cellX = nx;
                        cellY = ny;
                    }
                    phase = Phase::DECIDE;
                }
                break;
        }
    }

    bool isDone() { return phase == Phase::DONE; }

    void abort() {
        Motion::stop();
        MotorControl::disable();
        phase = Phase::DONE;
    }
}