#pragma once
#include <Arduino.h>

namespace Bluetooth {
    void begin();
    void update();  // non-blocking: drains Serial, dispatches complete lines,
                    // services LOG streaming

    // One-shot, consumed by ModeManager once it exists.
    bool consumeModeRequest(uint8_t& outModeIndex);
    bool consumeTestRequest(char* outBuffer, uint8_t bufferSize);

    // MOTOR command is only ever queued here, never applied directly --
    // Bluetooth can't safely gate it on ModeManager's state (that would
    // be a circular dependency, ModeManager already depends on
    // Bluetooth), so it just asks. Diagnostic is the only consumer, and
    // only applies it while genuinely in DIAGNOSTIC's own RUNNING
    // session.
    enum class MotorAction : uint8_t { SET_LEFT, SET_RIGHT, BRAKE, STOP };
    bool consumeMotorRequest(MotorAction& outAction, int16_t& outPwm);
}