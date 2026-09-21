#pragma once
#include <Arduino.h>

#include "../config/ControlConfig.h"

// Non-blocking motion primitives (FIRMWARE_SPECS.md section 34). Explorer/
// FastRun (once they exist) issue ONE of these at a time and poll
// isBusy(), same pull-model as SettingMode/StandbyMode -- nothing here
// blocks; each call just arms a state machine that update() drives.
//
// Forward motion holds heading via IMU yaw (ControlConfig::HEADING_*);
// turns rotate in place tracked by IMU yaw delta. PoseEstimator (x/y) is
// deliberately NOT used here -- a turn's accuracy depends on gyro
// feedback, not encoder-only odometry (spec section 35), and forward
// distance is tracked directly off encoder counts, the same conversion
// MotorControl already uses internally for its own measured speed.
//
// Does not touch MotorControl::enable()/disable() -- that's the caller's
// call (Explorer / ModeManager's RUNNING dispatch, once it exists).
// setTargetSpeeds() is safe to call regardless: MotorControl no-ops
// while disabled.
namespace Motion {
    enum class Primitive : uint8_t {
        NONE,
        FORWARD_CELL,
        TURN_LEFT_90,
        TURN_RIGHT_90,
        TURN_180,
        STOP
    };

    void begin();

    // Arms a primitive. No-op if one is already running -- caller must
    // wait for isBusy() to clear first, same one-at-a-time contract as
    // the rest of this codebase's state-machine modules.
    void moveForwardCell(
        float speedMmsS = ControlConfig::FORWARD_BASE_SPEED_MM_S);
    void turnLeft90();
    void turnRight90();
    void turn180();

    // Commands an immediate zero speed and completes right away -- a
    // primitive for "come to rest here", not an emergency abort (that's
    // Safety::triggerUserAbort()).
    void stop();

    // Call at the control period, same cadence as MotorControl::update().
    void update(uint32_t nowUs);

    bool isBusy();
    Primitive current();
}