#include "Motion.h"

#include <math.h>

#include "../config/ControlConfig.h"
#include "../config/RobotConfig.h"
#include "../drivers/Encoder.h"
#include "../drivers/IMU.h"
#include "MotorControl.h"
#include "PID.h"

namespace {
    // Same formula MotorControl.cpp uses internally for its own measured
    // speed -- duplicated here (not exposed by MotorControl) because
    // Motion needs accumulated distance, not instantaneous speed.
    const float MM_PER_COUNT_LEFT = (PI * RobotConfig::LEFT_WHEEL_DIAMETER_MM) /
                                    RobotConfig::ENCODER_COUNTS_PER_OUTPUT_REV;
    const float MM_PER_COUNT_RIGHT =
        (PI * RobotConfig::RIGHT_WHEEL_DIAMETER_MM) /
        RobotConfig::ENCODER_COUNTS_PER_OUTPUT_REV;

    const float deg_to_rad = PI / 180.0f;

    Motion::Primitive active = Motion::Primitive::NONE;

    PID headingHoldPid(ControlConfig::HEADING_KP, ControlConfig::HEADING_KI,
                       ControlConfig::HEADING_KD,
                       -ControlConfig::HEADING_CORRECTION_LIMIT_MM_S,
                       ControlConfig::HEADING_CORRECTION_LIMIT_MM_S,
                       -ControlConfig::HEADING_CORRECTION_LIMIT_MM_S,
                       ControlConfig::HEADING_CORRECTION_LIMIT_MM_S);

    // FORWARD_CELL state
    int32_t startLeftCount = 0, startRightCount = 0;
    float forwardStartYawDeg = 0.0f;
    uint32_t forwardLastUpdateUs = 0;
    float forwardSpeedMmS = ControlConfig::FORWARD_BASE_SPEED_MM_S;

    // TURN_* state
    float turnStartYawDeg = 0.0f;
    float turnTargetDeltaDeg = 0.0f;

    // Shared "hold zero, then done" tail used by both forward and turn --
    // lets the wheel PID actually settle instead of the next primitive
    // getting armed on top of leftover momentum.
    bool settling = false;
    uint32_t settleStartMs = 0;

    void beginSettle() {
        settling = true;
        settleStartMs = millis();
        MotorControl::setTargetSpeeds(0.0f, 0.0f);
    }

    bool settleDone() {
        return (millis() - settleStartMs) >= ControlConfig::MOTION_SETTLE_MS;
    }

    void armForward(float speedMmS) {
        active = Motion::Primitive::FORWARD_CELL;
        startLeftCount = Encoder::LEFT_ENCODER_COUNT();
        startRightCount = Encoder::RIGHT_ENCODER_COUNT();
        forwardStartYawDeg = IMU::getYawDeg();
        forwardLastUpdateUs = micros();
        forwardSpeedMmS = speedMmS;
        headingHoldPid.reset();
        settling = false;
    }

    void armTurn(Motion::Primitive p, float targetDeltaDeg) {
        active = p;
        turnStartYawDeg = IMU::getYawDeg();
        turnTargetDeltaDeg = targetDeltaDeg;
        settling = false;
    }

    void updateForward(uint32_t nowUs) {
        if (settling) {
            if (settleDone()) active = Motion::Primitive::NONE;
            return;
        }

        int32_t leftDelta = Encoder::LEFT_ENCODER_COUNT() - startLeftCount;
        int32_t rightDelta = Encoder::RIGHT_ENCODER_COUNT() - startRightCount;
        float distMm = ((leftDelta * MM_PER_COUNT_LEFT) +
                        (rightDelta * MM_PER_COUNT_RIGHT)) /
                       2.0f;

        if (distMm >= RobotConfig::CELL_SIZE_MM) {
            beginSettle();
            return;
        }

        float dtSec = (nowUs - forwardLastUpdateUs) / 1000000.0f;
        forwardLastUpdateUs = nowUs;

        // Section 36: hold heading via differential correction rather
        // than commanding equal PWM outright. PID::update() itself no-ops
        // (returns 0) on a non-positive dt, so a stray zero-dt tick just
        // means "no correction this tick", not a divide-by-zero.
        float correction =
            headingHoldPid.update(forwardStartYawDeg, IMU::getYawDeg(), dtSec);

        MotorControl::setTargetSpeeds(forwardSpeedMmS - correction,
                                      forwardSpeedMmS + correction);
    }

    void updateTurn(uint32_t nowUs) {
        (void)nowUs;
        if (settling) {
            if (settleDone()) active = Motion::Primitive::NONE;
            return;
        }

        float deltaSoFar = IMU::getYawDeg() - turnStartYawDeg;
        if (fabsf(turnTargetDeltaDeg - deltaSoFar) <=
            ControlConfig::TURN_ANGLE_TOLERANCE_DEG) {
            beginSettle();
            return;
        }

        float dir = (turnTargetDeltaDeg >= 0.0f) ? 1.0f : -1.0f;
        float wheelSpeed = ControlConfig::TURN_SPEED_DEG_S * deg_to_rad *
                           (RobotConfig::WHEEL_BASE_MM / 2.0f);
        // Positive yaw (per RobotConfig::IMU_YAW_SIGN) = left/CCW: left
        // wheel back, right wheel forward.
        MotorControl::setTargetSpeeds(-dir * wheelSpeed, dir * wheelSpeed);
    }
}

namespace Motion {
    void begin() { active = Primitive::NONE; }

    void moveForwardCell(float speedMmS) {
        if (active != Primitive::NONE) return;
        armForward(speedMmS);
    }

    void turnLeft90() {
        if (active != Primitive::NONE) return;
        armTurn(Primitive::TURN_LEFT_90, 90.0f);
    }

    void turnRight90() {
        if (active != Primitive::NONE) return;
        armTurn(Primitive::TURN_RIGHT_90, -90.0f);
    }

    void turn180() {
        if (active != Primitive::NONE) return;
        armTurn(Primitive::TURN_180, 180.0f);
    }

    void stop() {
        MotorControl::setTargetSpeeds(0.0f, 0.0f);
        active = Primitive::NONE;
        settling = false;
    }

    void update(uint32_t nowUs) {
        switch (active) {
            case Primitive::NONE:
            case Primitive::STOP:
                break;
            case Primitive::FORWARD_CELL:
                updateForward(nowUs);
                break;
            case Primitive::TURN_LEFT_90:
            case Primitive::TURN_RIGHT_90:
            case Primitive::TURN_180:
                updateTurn(nowUs);
                break;
        }
    }

    bool isBusy() { return active != Primitive::NONE; }
    Primitive current() { return active; }
}