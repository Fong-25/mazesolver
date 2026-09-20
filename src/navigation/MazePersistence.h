#pragma once
#include <Arduino.h>

// EEPROM persistence for the Maze module (FIRMWARE_SPECS.md section 48).
// Reads/writes Maze's raw per-cell bytes only -- knows nothing about
// walls/flood-fill/exploration, just moves bytes between Maze and EEPROM
// behind a magic/version/checksum wrapper so garbage or a stale format
// never gets loaded as a real map.
namespace MazePersistence {
    // Writes the current Maze contents to EEPROM. Call only at a
    // controlled event -- successful exploration complete, or an
    // explicit save command -- never on a timer or every tick. EEPROM
    // write endurance is finite (spec sections 44/48).
    void save();

    // Loads EEPROM contents into Maze if -- and only if -- the stored
    // magic/version/width/height/checksum all check out. Returns false
    // (and leaves Maze untouched) on any mismatch, so the caller can fall
    // back to Maze::reset() itself.
    bool load();

    // True if EEPROM currently holds a maze that load() would accept --
    // lets a caller check before committing to a load.
    bool hasValidSave();
}