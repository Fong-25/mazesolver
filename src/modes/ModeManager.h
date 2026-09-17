#pragma once
#include <Arduino.h>

// Owns the top-level operating-state machine (FIRMWARE_SPECS.md §49-50) and
// wires DipSwitch, Button, SettingMode, StandbyMode, Safety, and
// Bluetooth's bench-testing MODE override together. Does NOT drive any run
// mode's own logic yet -- Explorer/FastRun/Diagnostic don't exist, so
// RUNNING just waits. See TODOs in the .cpp.
namespace ModeManager {
    enum class SystemState : uint8_t {
        BOOT,  // main.cpp's setup() is still running driver begin()s
        INIT,  // reserved for a future self-check step (spec §20.14)
        LOCKED,
        SETTING,
        STANDBY,
        RUNNING,
        FINISHED,
        ERROR
    };

    // Matches Bluetooth's MODE <n> numbering and UserConfig's gesture
    // table indices -- keep all three in sync if this ever changes.
    enum class RunMode : uint8_t {
        EXPLORE = 0,
        FAST_RUN = 1,
        DIAGNOSTIC = 2,
        DEBUG_LOG = 3,
        MOTION_TEST = 4,
        NONE = 255  // nothing locked in yet this power-cycle
    };

    // Call once, after every driver's own begin() (Safety::begin()
    // included) -- see SETUP_NOTES.md boot order. Enters LOCKED directly;
    // BOOT/INIT are covered by main.cpp's own setup() sequence for now.
    void begin();

    // Call every loop tick, unconditionally.
    void update(uint32_t nowMs);

    SystemState getState();
    RunMode getLockedMode();
}