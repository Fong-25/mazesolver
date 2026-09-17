#pragma once
#include <Arduino.h>

namespace StandbyMode {
    // Call once, exactly when STANDBY begins.
    void begin();

    void update(uint32_t nowMs);

    // True once a valid cover-then-release sequence has completed —
    // the signal for ModeManager to actually start the selected run mode.
    bool isRunTriggered();

    // True from the moment a valid cover registers, straight through the
    // release event and isRunTriggered() firing, until the next begin()
    // resets it for a fresh STANDBY session. RgbStatus uses this to flip
    // from blinking to solid the instant the cover lands, not just once
    // you've already released.
    bool isCoverConfirmed();
}