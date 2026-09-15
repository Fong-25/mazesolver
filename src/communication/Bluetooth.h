#pragma once
#include <Arduino.h>

namespace Bluetooth {
    void begin();
    void update();  // non-blocking: drains Serial, dispatches complete lines,
                    // services LOG streaming

    // One-shot, consumed by ModeManager once it exists.
    bool consumeModeRequest(uint8_t& outModeIndex);
    bool consumeTestRequest(char* outBuffer, uint8_t bufferSize);
}