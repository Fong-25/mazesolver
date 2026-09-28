#include "Motion.h"

#include <math.h>

#include "../config/ControlConfig.h"
#include "../config/RobotConfig.h"
#include "../drivers/Encoder.h"
#include "../drivers/IMU.h"
#include "../drivers/ToFManager.h"
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

    // Same shape as headingHoldPid -- see combinedCorrection() for how its
    // target/measurement are set up to make the sign agree with it.
    // Compiled out entirely when the feature is off -- see
    // MAZESOLVER_COMPILE_TOF_WALL_CENTERING's comment in ControlConfig.h.
#if MAZESOLVER_COMPILE_TOF_WALL_CENTERING
    PID wallCenteringPid(ControlConfig::WALL_KP, ControlConfig::WALL_KI,
                         ControlConfig::WALL_KD,
                         -ControlConfig::WALL_CORRECTION_LIMIT_MM_S,
                         ControlConfig::WALL_CORRECTION_LIMIT_MM_S,
                         -ControlConfig::WALL_CORRECTION_LIMIT_MM_S,
                         ControlConfig::WALL_CORRECTION_LIMIT_MM_S);
#endif

    // FORWARD_CELL state
    int32_t startLeftCount = 0, startRightCount = 0;
    float forwardStartYawDeg = 0.0f;
    uint32_t forwardLastUpdateUs = 0;
    float forwardSpeedMmS = ControlConfig::FORWARD_BASE_SPEED_MM_S;
    float forwardTargetDistanceMm = RobotConfig::CELL_SIZE_MM;
    // Trapezoidal (or, for a short move, triangular) velocity profile --
    // spec section 31's "aggressive but controlled acceleration /
    // deceleration". This is the commanded speed for THIS tick; it ramps
    // toward forwardSpeedMmS at the top and is capped low enough, as
    // distance runs out, to still be able to brake to ~0 by
    // forwardTargetDistanceMm at the same accel limit. A short move (below
    // one accel-then-decel's worth of distance) never reaches
    // forwardSpeedMmS at all -- the brake cap kicks in before the ramp-up
    // does, which is exactly a triangular profile, with no separate case
    // needed for it.
    float forwardCurrentSpeedMmS = 0.0f;
    bool forwardAllowWallCentering = false;

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

    void armForward(float speedMmS, float distanceMm, bool allowWallCentering) {
        active = Motion::Primitive::FORWARD_CELL;
        startLeftCount = Encoder::LEFT_ENCODER_COUNT();
        startRightCount = Encoder::RIGHT_ENCODER_COUNT();
        forwardStartYawDeg = IMU::getYawDeg();
        forwardLastUpdateUs = micros();
        forwardSpeedMmS = speedMmS;
        forwardTargetDistanceMm = distanceMm;
        forwardCurrentSpeedMmS = 0.0f;
        forwardAllowWallCentering = allowWallCentering;
        headingHoldPid.reset();
#if MAZESOLVER_COMPILE_TOF_WALL_CENTERING
        wallCenteringPid.reset();
#endif
        settling = false;
    }

    // Sums whichever correction sources are enabled (spec section 36/37:
    // heading error and wall error both feed one differential correction).
    // Each term is independently gated and independently clamped
    // (ControlConfig::*_CORRECTION_LIMIT_MM_S) before summing, so an
    // untuned term can't swamp a working one.
    float combinedCorrection(float dtSec) {
        float correction = 0.0f;

        if (ControlConfig::ENABLE_IMU_HEADING_HOLD) {
            correction += headingHoldPid.update(forwardStartYawDeg,
                                                IMU::getYawDeg(), dtSec);
        }

        if (ControlConfig::ENABLE_TOF_WALL_CENTERING &&
            forwardAllowWallCentering) {
#if MAZESOLVER_COMPILE_TOF_WALL_CENTERING
            using ToFManager::SensorRole;
            // Spec section 37: an invalid/very-large reading (open side,
            // no wall) must NOT be treated as a huge angular error --
            // only correct when BOTH sides are confirmed valid; otherwise
            // skip this source entirely for the tick and fall back to
            // whatever else is enabled (IMU/encoders).
            bool leftValid = ToFManager::isValid(SensorRole::DIAGONAL_LEFT);
            bool rightValid = ToFManager::isValid(SensorRole::DIAGONAL_RIGHT);
            if (leftValid && rightValid) {
                float leftMm =
                    (float)ToFManager::getDistanceMm(SensorRole::DIAGONAL_LEFT);
                float rightMm = (float)ToFManager::getDistanceMm(
                    SensorRole::DIAGONAL_RIGHT);
                // target/measurement set up so PID's (target-measurement)
                // comes out to (leftMm - rightMm) -- positive when closer
                // to the right wall than the left, matching
                // headingHoldPid's sign convention (positive correction =
                // right wheel faster). Sign TBD, verify physically same
                // as every other signed convention in this codebase.
                correction +=
                    wallCenteringPid.update(0.0f, rightMm - leftMm, dtSec);
            }
#endif
        }

        return correction;
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

        if (distMm >= forwardTargetDistanceMm) {
            beginSettle();
            return;
        }

        float dtSec = (nowUs - forwardLastUpdateUs) / 1000000.0f;
        forwardLastUpdateUs = nowUs;
        // A non-positive dt (stray zero-dt tick, or a clock rollover) must
        // not move forwardCurrentSpeedMmS -- treat it as "hold speed, no
        // correction this tick" rather than let a bad dt compute a bogus
        // ramp step.
        if (dtSec <= 0.0f) dtSec = 0.0f;

        // Deceleration cap: the fastest we can be going right now and
        // still stop (v=0) in the distance that's left, at
        // MAX_LINEAR_ACCEL_MM_S2. v^2 = 2*a*d -> v = sqrt(2*a*d).
        float remainingMm = forwardTargetDistanceMm - distMm;
        if (remainingMm < 0.0f) remainingMm = 0.0f;
        float brakeCapMmS =
            sqrtf(2.0f * RobotConfig::MAX_LINEAR_ACCEL_MM_S2 * remainingMm);

        float rampTargetMmS = forwardSpeedMmS;
        if (brakeCapMmS < rampTargetMmS) rampTargetMmS = brakeCapMmS;

        // Slew forwardCurrentSpeedMmS toward rampTargetMmS at the accel
        // limit -- same constant for speeding up and braking, per spec
        // section 31 (no separate decel constant defined). Applied as a
        // step so it can move in either direction without a sign branch.
        float maxStepMmS = RobotConfig::MAX_LINEAR_ACCEL_MM_S2 * dtSec;
        float delta = rampTargetMmS - forwardCurrentSpeedMmS;
        if (delta > maxStepMmS) delta = maxStepMmS;
        if (delta < -maxStepMmS) delta = -maxStepMmS;
        forwardCurrentSpeedMmS += delta;

        // Section 36/37: hold heading and/or center in the corridor via
        // differential correction, rather than commanding equal PWM
        // outright. Each PID no-ops (returns 0) on a non-positive dt.
        float correction = combinedCorrection(dtSec);

        MotorControl::setTargetSpeeds(forwardCurrentSpeedMmS - correction,
                                      forwardCurrentSpeedMmS + correction);
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

    void moveForwardCell(float speedMmS, float distanceMm,
                         bool allowWallCentering) {
        if (active != Primitive::NONE) return;
        armForward(speedMmS, distanceMm, allowWallCentering);
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