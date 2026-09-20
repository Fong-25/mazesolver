#include "MazePersistence.h"

#include <EEPROM.h>

#include "../config/MazeConfig.h"
#include "Maze.h"

namespace {
    constexpr uint16_t MAGIC = 0x4D5A;  // "MZ"
    constexpr uint8_t FORMAT_VERSION = 1;

    // Mirrors FIRMWARE_SPECS.md section 48's MazeStorage layout field for
    // field, addressed manually (not via EEPROM.put(struct)) so the
    // on-EEPROM layout never depends on this compiler's struct padding.
    constexpr uint16_t ADDR_MAGIC = 0;
    constexpr uint16_t ADDR_VERSION = ADDR_MAGIC + sizeof(uint16_t);
    constexpr uint16_t ADDR_WIDTH = ADDR_VERSION + sizeof(uint8_t);
    constexpr uint16_t ADDR_HEIGHT = ADDR_WIDTH + sizeof(uint8_t);
    constexpr uint16_t ADDR_CELLS = ADDR_HEIGHT + sizeof(uint8_t);
    constexpr uint16_t CELL_COUNT =
        (uint16_t)MazeConfig::WIDTH * MazeConfig::HEIGHT;
    constexpr uint16_t ADDR_CHECKSUM = ADDR_CELLS + CELL_COUNT;
    // 261 bytes at the default 16x16 -- comfortably inside the
    // ATmega328P's 1024-byte EEPROM.

    // Simple additive checksum over magic+version+width+height+cells --
    // not cryptographic, just enough to catch a torn write or genuinely
    // unrelated EEPROM contents, per spec's "use a version number /
    // checksum so corrupted or unrelated contents are rejected."
    uint16_t checksumOf(uint16_t magic, uint8_t version, uint8_t width,
                        uint8_t height,
                        uint8_t (*cellAt)(uint8_t x, uint8_t y)) {
        uint16_t sum = magic + version + width + height;
        for (uint8_t y = 0; y < MazeConfig::HEIGHT; y++) {
            for (uint8_t x = 0; x < MazeConfig::WIDTH; x++) {
                sum += cellAt(x, y);
            }
        }
        return sum;
    }

    uint8_t readLiveCell(uint8_t x, uint8_t y) {
        return Maze::getRawCell(x, y);
    }

    uint8_t readStoredCell(uint8_t x, uint8_t y) {
        return EEPROM.read(ADDR_CELLS + (uint16_t)y * MazeConfig::WIDTH + x);
    }
}

namespace MazePersistence {
    void save() {
        EEPROM.put(ADDR_MAGIC, MAGIC);
        EEPROM.update(ADDR_VERSION, FORMAT_VERSION);
        EEPROM.update(ADDR_WIDTH, MazeConfig::WIDTH);
        EEPROM.update(ADDR_HEIGHT, MazeConfig::HEIGHT);
        for (uint8_t y = 0; y < MazeConfig::HEIGHT; y++) {
            for (uint8_t x = 0; x < MazeConfig::WIDTH; x++) {
                EEPROM.update(ADDR_CELLS + (uint16_t)y * MazeConfig::WIDTH + x,
                              Maze::getRawCell(x, y));
            }
        }
        uint16_t checksum = checksumOf(MAGIC, FORMAT_VERSION, MazeConfig::WIDTH,
                                       MazeConfig::HEIGHT, readLiveCell);
        EEPROM.put(ADDR_CHECKSUM, checksum);
    }

    bool hasValidSave() {
        uint16_t magic;
        EEPROM.get(ADDR_MAGIC, magic);
        if (magic != MAGIC) return false;
        if (EEPROM.read(ADDR_VERSION) != FORMAT_VERSION) return false;
        if (EEPROM.read(ADDR_WIDTH) != MazeConfig::WIDTH) return false;
        if (EEPROM.read(ADDR_HEIGHT) != MazeConfig::HEIGHT) return false;

        uint16_t storedChecksum;
        EEPROM.get(ADDR_CHECKSUM, storedChecksum);
        uint16_t expected = checksumOf(
            magic, EEPROM.read(ADDR_VERSION), EEPROM.read(ADDR_WIDTH),
            EEPROM.read(ADDR_HEIGHT), readStoredCell);
        return storedChecksum == expected;
    }

    bool load() {
        if (!hasValidSave()) return false;

        for (uint8_t y = 0; y < MazeConfig::HEIGHT; y++) {
            for (uint8_t x = 0; x < MazeConfig::WIDTH; x++) {
                Maze::setRawCell(x, y, readStoredCell(x, y));
            }
        }
        return true;
    }
}