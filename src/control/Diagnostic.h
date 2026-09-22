#pragma once
#include <Arduino.h>

// Executes DIAGNOSTIC mode's raw motor commands
// (MODES_AND_BLUETOOTH_PROTOCOL.md's Mode 2 table). ModeManager calls
// begin() when RUNNING starts for this mode, then update() every tick.
// Consumes Bluetooth's queued MOTOR requests and applies them directly
// to MotorDriver, deliberately bypassing MotorControl/Motion --
// DIAGNOSTIC is for raw PWM/polarity bring-up, not closed-loop motion.
//
// This is also the fix for a real gap: before this existed, Bluetooth's
// MOTOR command called MotorDriver directly with no gating at all, so it
// worked from ANY ModeManager state (LOCKED, STANDBY, mid-EXPLORE...).
// Now a MOTOR request is only ever queued by Bluetooth, and only
// actually applied here, while genuinely inside DIAGNOSTIC's own
// RUNNING session.
namespace Diagnostic {
    void begin();
    void update(uint32_t nowMs);

    // DIAGNOSTIC never finishes on its own -- it's an open bring-up
    // session for as long as the mode stays active, same open question
    // as DEBUG_LOG (see MODES_AND_BLUETOOTH_PROTOCOL.md's open items:
    // no exit mechanism back to LOCKED is defined yet for either).
    // Always returns false for now.
    bool isDone();
}