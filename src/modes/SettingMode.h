#pragma once
#include <Arduino.h>

namespace SettingMode {
    void begin();

    // Call every loop tick while DipSwitch::isSettingModeActive() is true.
    void update(uint32_t nowMs);

    // Call once exactly when SETTING becomes active, to start clean.
    void reset();

    // -1 = nothing selected yet this SETTING session.
    int8_t getSelectedModeIndex();
}