#pragma once
#include <Arduino.h>

// Distance-to-target flood fill over the current Maze state
// (FIRMWARE_SPECS.md section 29, step 4: "recompute or update flood-fill
// distances"). Pure grid algorithm -- reads Maze's known walls, writes
// nothing back to Maze, knows nothing about the robot's actual
// position/heading or sensors.
//
// Explorer (once it exists) owns selecting where to go next using these
// distances plus Maze::isVisited() -- that's a decision that also needs
// the robot's current cell/heading, neither of which FloodFill tracks, so
// it doesn't belong here.
namespace FloodFill {
    constexpr uint8_t UNREACHABLE = 255;

    // Recomputes distance for every cell reachable (through currently
    // known walls) from the goal region (MazeConfig::GOAL_*). Full
    // recompute, not incremental -- fine at 16x16 (worst case a few
    // thousand wall checks). Call after wall discovery changes the map,
    // never every loop tick.
    void computeToGoal();

    // Same, but flooding from a single arbitrary target cell instead of
    // the goal region -- what a return-to-start leg needs, once Explorer
    // calls computeToTarget(MazeConfig::START_X, MazeConfig::START_Y).
    void computeToTarget(uint8_t targetX, uint8_t targetY);

    // UNREACHABLE if no currently-known path exists from (x,y) to
    // whichever cell(s) computeToGoal()/computeToTarget() last flooded
    // from.
    uint8_t getDistance(uint8_t x, uint8_t y);
}