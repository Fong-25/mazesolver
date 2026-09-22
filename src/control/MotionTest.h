#pragma once
#include <Arduino.h>

// Drives Motion's primitives for MOTION_TEST mode
// (MODES_AND_BLUETOOTH_PROTOCOL.md's Mode 4 table). ModeManager calls
// begin() when RUNNING starts for this mode, then update() every tick.
// Waits idle for a Bluetooth::consumeTestRequest() command, runs exactly
// one requested test (or the full TEST ALL sequence) to completion, then
// reports done -- same "one run, then FINISHED" shape as Explorer/
// FastRun, not an indefinite loop, so leaving the mode always goes
// through the existing FINISHED -> PRESS -> LOCKED path. Run another
// test by cycling back through STANDBY.
//
// TODO: Motion has no distinct accel/cruise/decel velocity-profile
// phases yet -- ACCEL/CRUISE/DECEL all currently run the same
// moveForwardCell() primitive. Revisit once Motion gains real profile
// shaping; for now the wheel PID's own step response IS the
// accel/decel behavior, just not separately isolatable per phase.
namespace MotionTest {
    void begin();
    void update(uint32_t nowMs);
    bool isDone();
}