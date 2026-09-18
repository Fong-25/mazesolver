#pragma once
#include <Arduino.h>

// Pure maze data model (FIRMWARE_SPECS.md section 27) -- the RAM-resident
// runtime map. No sensing, no motion, no persistence: FloodFill/Explorer
// read and write this, a future persistence module reads/writes
// getRawCell()/setRawCell() to/from EEPROM. This module doesn't know
// either of those exist.
//
// Coordinates: (0,0) is the bottom-left cell. NORTH = +y, EAST = +x.
namespace Maze {
    enum class Direction : uint8_t { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

    // Clears every cell (no walls, unvisited, unconfirmed), then stamps
    // the four outer-border walls back in -- those are structural, never
    // something Explorer has to discover. Call once before a fresh
    // EXPLORE run.
    void reset();

    bool isValidCell(uint8_t x, uint8_t y);

    // Out-of-bounds reads as "walled off" -- safe default for a caller
    // that didn't check isValidCell() first.
    bool hasWall(uint8_t x, uint8_t y, Direction dir);

    // A wall is a property of the edge between two cells, never just one
    // side of it -- this mirrors the change onto the neighbor's opposite
    // side too, if that neighbor exists. Out-of-bounds (x,y) is a no-op.
    void setWall(uint8_t x, uint8_t y, Direction dir, bool present);

    bool isVisited(uint8_t x, uint8_t y);
    void setVisited(uint8_t x, uint8_t y);

    // Meaning is Explorer's call once it exists (e.g. "walls on this cell
    // agree from both directions of travel") -- Maze just stores the bit.
    bool isConfirmed(uint8_t x, uint8_t y);
    void setConfirmed(uint8_t x, uint8_t y);

    Direction opposite(Direction dir);

    // Grid geometry only -- does NOT check hasWall(), that's
    // FloodFill/Explorer's job when deciding where they're allowed to go.
    // Returns false (outX/outY untouched) if dir would step off the grid.
    bool neighborOf(uint8_t x, uint8_t y, Direction dir, uint8_t& outX,
                    uint8_t& outY);

    // Raw per-cell byte (bit layout: N,E,S,W,visited,confirmed) -- for the
    // persistence module once it exists. Nobody else should need this.
    uint8_t getRawCell(uint8_t x, uint8_t y);
    void setRawCell(uint8_t x, uint8_t y, uint8_t raw);
}