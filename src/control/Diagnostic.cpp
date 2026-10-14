#include "Diagnostic.h"

#include "../communication/Bluetooth.h"
#include "../config/ControlConfig.h"
#include "../drivers/MotorDriver.h"
#include "../system/Diagnostics.h"
#include "../system/Safety.h"

namespace {
    // Updated on every consumed MOTOR request; see update()'s timeout
    // check below.
    uint32_t lastMotorCommandMs = 0;
}

namespace Diagnostic {
    void begin() {
        // Nothing to arm -- DIAGNOSTIC drives MotorDriver directly, not
        // through MotorControl, so there's no enable()/disable() pairing
        // needed the way Explorer/FastRun/MotionTest have.
        //
        // Starts the timeout clock fresh: a session that's entered but
        // never sent a single MOTOR command still trips on schedule
        // rather than sitting silently un-armed.
        lastMotorCommandMs = millis();
    }

    void update(uint32_t nowMs) {
        Bluetooth::MotorAction action;
        int16_t pwm;
        if (Bluetooth::consumeMotorRequest(action, pwm)) {
            lastMotorCommandMs = nowMs;

            switch (action) {
                case Bluetooth::MotorAction::SET_LEFT:
                    MotorDriver::setLeftPWM(pwm);
                    break;
                case Bluetooth::MotorAction::SET_RIGHT:
                    MotorDriver::setRightPWM(pwm);
                    break;
                case Bluetooth::MotorAction::BRAKE:
                    MotorDriver::brakeLeft();
                    MotorDriver::brakeRight();
                    break;
                case Bluetooth::MotorAction::STOP:
                    MotorDriver::stopAll();
                    break;
            }
        }

        // No command (of ANY kind -- including STOP/BRAKE, which is fine,
        // they're idle states anyway) for DIAGNOSTIC_MOTOR_TIMEOUT_MS ->
        // stop driving and trip a real fault rather than silently coast.
        // Safety::trip() is idempotent (first cause wins), so this is
        // safe to call every tick once past the deadline.
        if ((nowMs - lastMotorCommandMs) >=
            ControlConfig::DIAGNOSTIC_MOTOR_TIMEOUT_MS) {
            MotorDriver::stopAll();
            Safety::triggerFault(Diagnostics::ErrorCode::MOTOR_TIMEOUT);
        }
    }

    bool isDone() { return false; }

    void abort() { MotorDriver::stopAll(); }
}