#pragma once
#include <Arduino.h>

// Owns the flood-fill exploration run (FIRMWARE_SPECS.md section 29):
// senses walls from the current cell each stop, updates Maze, recomputes
// FloodFill, picks the next cell, and issues Motion primitives to get
// there -- one DECIDE/TURN/MOVE step per cell, non-blocking throughout.
// After reaching the goal it automatically flood-fills back toward
// MazeConfig::START_* and walks the known path home, then reports done.
//
// ModeManager doesn't dispatch to this yet (RUNNING is still a TODO in
// ModeManager.cpp) -- this is a standalone, ready-to-wire module.
namespace Explorer {
    // (Re)starts a run from MazeConfig::START_*, heading
    // Maze::Direction::NORTH. Call once per EXPLORE attempt -- after
    // Maze::reset() too, if this should be a fresh map rather than
    // continuing on a previously-loaded one.
    void begin();

    // Call every loop tick while a run is active.
    void update(uint32_t nowMs);

    void abort();

    // True once the goal was reached AND the return-to-start leg is also
    // complete -- ModeManager's RUNNING state should treat this as the
    // signal to move to FINISHED.
    bool isDone();
}