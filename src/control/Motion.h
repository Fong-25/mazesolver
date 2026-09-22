#pragma once
#include <Arduino.h>

#include "../config/ControlConfig.h"
#include "../config/RobotConfig.h"

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
    //
    // moveForwardCell()'s speed defaults to the EXPLORE baseline; FastRun
    // passes ControlConfig::FAST_RUN_SPEED_MM_S instead. distanceMm
    // defaults to one cell, but a caller can pass a multiple of
    // RobotConfig::CELL_SIZE_MM to cover several consecutive cells in one
    // primitive without stopping between them (FastRun's straight-run
    // batching), or RobotConfig::FIRST_MOVE_DISTANCE_MM for the very
    // first move of a run (see that constant's comment -- corrects for
    // the robot not starting exactly cell-centered). Heading hold and the
    // settle tail are unaffected by either parameter.
    void moveForwardCell(
        float speedMmS = ControlConfig::FORWARD_BASE_SPEED_MM_S,
        float distanceMm = RobotConfig::CELL_SIZE_MM);
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