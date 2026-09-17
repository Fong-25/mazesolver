#pragma once
#include <Arduino.h>

// Centralized RGB status indicator (FIRMWARE_SPECS.md section 17). The
// only module allowed to touch RGB directly -- reads ModeManager /
// SettingMode / StandbyMode and translates that into color + blink so
// nothing else has to reach into the driver itself.
namespace RgbStatus {
    void begin();

    // Call every loop tick, unconditionally -- same pull-model pattern
    // SettingMode/StandbyMode use for their own inputs. Call AFTER
    // ModeManager::update() so it reads this tick's state, not last tick's.
    void update(uint32_t nowMs);
}