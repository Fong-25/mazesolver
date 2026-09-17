#pragma once
#include <Arduino.h>

namespace StandbyMode {
    // Call once, exactly when STANDBY begins.
    void begin();

    void update(uint32_t nowMs);

    // True once a valid cover-then-release sequence has completed —
    // the signal for ModeManager to actually start the selected run mode.
    bool isRunTriggered();
}