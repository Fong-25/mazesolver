#pragma once
#include <Arduino.h>

namespace DipSwitch {
    enum class Event : uint8_t {
        NONE,
        ENTERED_SETTING,  // OFF -> ON
        LOCKED            // ON -> OFF
    };

    void begin();
    void update(uint32_t nowMs);

    // Consumes the transition event, if any, since the last call.
    Event getEvent();

    // Current debounced state: true = ON (in Setting Mode territory).
    bool isSettingModeActive();
}