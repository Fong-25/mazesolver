#include "Maze.h"

#include "../config/MazeConfig.h"

namespace {
    constexpr uint8_t BIT_NORTH = 1 << 0;
    constexpr uint8_t BIT_EAST = 1 << 1;
    constexpr uint8_t BIT_SOUTH = 1 << 2;
    constexpr uint8_t BIT_WEST = 1 << 3;
    constexpr uint8_t BIT_VISITED = 1 << 4;
    constexpr uint8_t BIT_CONFIRMED = 1 << 5;

    // 256 bytes at the default 16x16 -- matches PERSISTENT_MAZE_FORMAT's
    // cells[256] one-for-one, deliberately.
    uint8_t cells[(uint16_t)MazeConfig::WIDTH * MazeConfig::HEIGHT];

    uint16_t indexOf(uint8_t x, uint8_t y) {
        return (uint16_t)y * MazeConfig::WIDTH + x;
    }

    uint8_t bitFor(Maze::Direction dir) {
        switch (dir) {
            case Maze::Direction::NORTH:
                return BIT_NORTH;
            case Maze::Direction::EAST:
                return BIT_EAST;
            case Maze::Direction::SOUTH:
                return BIT_SOUTH;
            case Maze::Direction::WEST:
                return BIT_WEST;
        }
        return 0;
    }
}

namespace Maze {
    void reset() {
        for (uint16_t i = 0;
             i < (uint16_t)MazeConfig::WIDTH * MazeConfig::HEIGHT; i++) {
            cells[i] = 0;
        }

        // Outer border is always walled -- structural, not something
        // Explorer should ever have to "discover".
        for (uint8_t x = 0; x < MazeConfig::WIDTH; x++) {
            cells[indexOf(x, 0)] |= BIT_SOUTH;
            cells[indexOf(x, MazeConfig::HEIGHT - 1)] |= BIT_NORTH;
        }
        for (uint8_t y = 0; y < MazeConfig::HEIGHT; y++) {
            cells[indexOf(0, y)] |= BIT_WEST;
            cells[indexOf(MazeConfig::WIDTH - 1, y)] |= BIT_EAST;
        }
    }

    bool isValidCell(uint8_t x, uint8_t y) {
        return x < MazeConfig::WIDTH && y < MazeConfig::HEIGHT;
    }

    bool hasWall(uint8_t x, uint8_t y, Direction dir) {
        if (!isValidCell(x, y)) return true;
        return cells[indexOf(x, y)] & bitFor(dir);
    }

    Direction opposite(Direction dir) {
        switch (dir) {
            case Direction::NORTH:
                return Direction::SOUTH;
            case Direction::EAST:
                return Direction::WEST;
            case Direction::SOUTH:
                return Direction::NORTH;
            case Direction::WEST:
                return Direction::EAST;
        }
        return Direction::NORTH;
    }

    bool neighborOf(uint8_t x, uint8_t y, Direction dir, uint8_t& outX,
                    uint8_t& outY) {
        int16_t nx = x;
        int16_t ny = y;
        switch (dir) {
            case Direction::NORTH:
                ny++;
                break;
            case Direction::EAST:
                nx++;
                break;
            case Direction::SOUTH:
                ny--;
                break;
            case Direction::WEST:
                nx--;
                break;
        }
        if (nx < 0 || ny < 0 || !isValidCell((uint8_t)nx, (uint8_t)ny)) {
            return false;
        }
        outX = (uint8_t)nx;
        outY = (uint8_t)ny;
        return true;
    }

    void setWall(uint8_t x, uint8_t y, Direction dir, bool present) {
        if (!isValidCell(x, y)) return;

        uint8_t mask = bitFor(dir);
        if (present) {
            cells[indexOf(x, y)] |= mask;
        } else {
            cells[indexOf(x, y)] &= ~mask;
        }

        uint8_t nx, ny;
        if (neighborOf(x, y, dir, nx, ny)) {
            uint8_t oppMask = bitFor(opposite(dir));
            if (present) {
                cells[indexOf(nx, ny)] |= oppMask;
            } else {
                cells[indexOf(nx, ny)] &= ~oppMask;
            }
        }
    }

    bool isVisited(uint8_t x, uint8_t y) {
        if (!isValidCell(x, y)) return false;
        return cells[indexOf(x, y)] & BIT_VISITED;
    }

    void setVisited(uint8_t x, uint8_t y) {
        if (!isValidCell(x, y)) return;
        cells[indexOf(x, y)] |= BIT_VISITED;
    }

    bool isConfirmed(uint8_t x, uint8_t y) {
        if (!isValidCell(x, y)) return false;
        return cells[indexOf(x, y)] & BIT_CONFIRMED;
    }

    void setConfirmed(uint8_t x, uint8_t y) {
        if (!isValidCell(x, y)) return;
        cells[indexOf(x, y)] |= BIT_CONFIRMED;
    }

    uint8_t getRawCell(uint8_t x, uint8_t y) {
        if (!isValidCell(x, y)) return 0;
        return cells[indexOf(x, y)];
    }

    void setRawCell(uint8_t x, uint8_t y, uint8_t raw) {
        if (!isValidCell(x, y)) return;
        cells[indexOf(x, y)] = raw;
    }
}