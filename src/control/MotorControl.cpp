#include "MotorControl.h"

#include <math.h>

#include "../config/BoardConfig.h"
#include "../config/ControlConfig.h"
#include "../config/RobotConfig.h"
#include "../drivers/Encoder.h"
#include "../drivers/MotorDriver.h"
#include "PID.h"

namespace {
    PID leftPid(ControlConfig::LEFT_KP, ControlConfig::LEFT_KI,
                ControlConfig::LEFT_KD, -(float)Board::PWM_MAX,
                (float)Board::PWM_MAX, -ControlConfig::WHEEL_PID_INTEGRAL_LIMIT,
                ControlConfig::WHEEL_PID_INTEGRAL_LIMIT,
                ControlConfig::WHEEL_PID_DERIVATIVE_FILTER_ALPHA);

    PID rightPid(ControlConfig::RIGHT_KP, ControlConfig::RIGHT_KI,
                 ControlConfig::RIGHT_KD, -(float)Board::PWM_MAX,
                 (float)Board::PWM_MAX,
                 -ControlConfig::WHEEL_PID_INTEGRAL_LIMIT,
                 ControlConfig::WHEEL_PID_INTEGRAL_LIMIT,
                 ControlConfig::WHEEL_PID_DERIVATIVE_FILTER_ALPHA);

    const float MM_PER_COUNT = (PI * RobotConfig::WHEEL_DIAMETER_MM) /
                               RobotConfig::ENCODER_COUNTS_PER_OUTPUT_REV;

    float targetLeftMmS = 0.0f, targetRightMmS = 0.0f;
    float measuredLeftMmS = 0.0f, measuredRightMmS = 0.0f;

    int32_t lastLeftCount = 0, lastRightCount = 0;
    uint32_t lastUpdateUs = 0;

    int16_t lastLeftPwm = 0, lastRightPwm = 0;
    bool enabled = true;

    int16_t applyDeadband(float pwmFloat, float targetSpeed) {
        int16_t pwm = (int16_t)pwmFloat;

        // Only compensate when we actually want motion (nonzero target).
        // Gating on "PID output happens to be nonzero" instead would force
        // a PWM floor almost constantly (float output is essentially never
        // exactly 0.0), making the robot dither/hum at rest instead of
        // settling. Gating on target is what section 10.2 actually means.
        if (targetSpeed != 0.0f) {
            if (pwm > 0 && pwm < ControlConfig::MOTOR_DEADBAND_PWM)
                pwm = ControlConfig::MOTOR_DEADBAND_PWM;
            else if (pwm < 0 && pwm > -ControlConfig::MOTOR_DEADBAND_PWM)
                pwm = -ControlConfig::MOTOR_DEADBAND_PWM;
        }

        if (pwm > Board::PWM_MAX) pwm = Board::PWM_MAX;
        if (pwm < -Board::PWM_MAX) pwm = -Board::PWM_MAX;

        return pwm;
    }
}

namespace MotorControl {
    void begin() {
        lastLeftCount = Encoder::LEFT_ENCODER_COUNT();
        lastRightCount = Encoder::RIGHT_ENCODER_COUNT();
        lastUpdateUs = micros();
        enabled = true;
    }

    void setTargetSpeeds(float leftMmS, float rightMmS) {
        targetLeftMmS = leftMmS;
        targetRightMmS = rightMmS;
    }

    void update(uint32_t nowUs) {
        uint32_t dtUs = nowUs - lastUpdateUs;
        if (dtUs == 0) return;
        lastUpdateUs = nowUs;
        float dtSec = dtUs / 1000000.0f;

        int32_t leftCount = Encoder::LEFT_ENCODER_COUNT();
        int32_t rightCount = Encoder::RIGHT_ENCODER_COUNT();
        int32_t leftDelta = leftCount - lastLeftCount;
        int32_t rightDelta = rightCount - lastRightCount;
        lastLeftCount = leftCount;
        lastRightCount = rightCount;

        measuredLeftMmS = (leftDelta * MM_PER_COUNT) / dtSec;
        measuredRightMmS = (rightDelta * MM_PER_COUNT) / dtSec;

        if (!enabled) {
            lastLeftPwm = lastRightPwm = 0;
            MotorDriver::stopAll();
            return;
        }

        float leftOutput =
            leftPid.update(targetLeftMmS, measuredLeftMmS, dtSec);
        float rightOutput =
            rightPid.update(targetRightMmS, measuredRightMmS, dtSec);

        lastLeftPwm = applyDeadband(leftOutput, targetLeftMmS);
        lastRightPwm = applyDeadband(rightOutput, targetRightMmS);

        MotorDriver::setLeftPWM(lastLeftPwm);
        MotorDriver::setRightPWM(lastRightPwm);
    }

    void enable() {
        enabled = true;
        leftPid.reset();
        rightPid.reset();
    }

    void disable() {
        enabled = false;
        leftPid.reset();
        rightPid.reset();
        MotorDriver::stopAll();
    }

    float getMeasuredLeftSpeedMmS() { return measuredLeftMmS; }
    float getMeasuredRightSpeedMmS() { return measuredRightMmS; }
    int16_t getLastLeftPwm() { return lastLeftPwm; }
    int16_t getLastRightPwm() { return lastRightPwm; }
}