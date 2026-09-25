#pragma once
#include <Arduino.h>

// Runs the already-known map at speed (FIRMWARE_SPECS.md's "Fast run"
// phase, spec section 75's build order) -- no wall sensing, no
// Maze/FloodFill updates, just follows FloodFill's shortest known path
// from MazeConfig::START_* to the goal region using Motion primitives at
// ControlConfig::FAST_RUN_SPEED_MM_S.
//
// Consecutive same-direction steps are batched into ONE Motion primitive
// (no stop between cells, spec section 33); turns still stop and rotate
// in place. Ties between equal-length paths prefer going straight. The
// first travel of a run also centers the robot in the start cell
// (RobotConfig::FIRST_MOVE_DISTANCE_MM).
//
// Requires Maze to already hold a real map -- from a completed EXPLORE
// run this session, or MazePersistence::load() at boot. Doesn't verify
// that itself; ModeManager choosing to enter FAST_RUN at all is the
// implicit contract that a map exists.
namespace FastRun {
    // (Re)starts a run from MazeConfig::START_*, heading
    // Maze::Direction::NORTH, using whatever's currently in Maze.
    void begin();

    void update(uint32_t nowMs);

    void abort();

    // True once the goal region is reached. Unlike Explorer, there's no
    // return-to-start leg -- fast run is a one-way timed attempt.
    bool isDone();
}