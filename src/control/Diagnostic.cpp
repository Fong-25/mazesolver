#include "Diagnostic.h"

#include "../communication/Bluetooth.h"
#include "../drivers/MotorDriver.h"

namespace Diagnostic {
    void begin() {
        // Nothing to arm -- DIAGNOSTIC drives MotorDriver directly, not
        // through MotorControl, so there's no enable()/disable() pairing
        // needed the way Explorer/FastRun/MotionTest have.
    }

    void update(uint32_t nowMs) {
        (void)nowMs;
        Bluetooth::MotorAction action;
        int16_t pwm;
        if (!Bluetooth::consumeMotorRequest(action, pwm)) return;

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

    bool isDone() { return false; }
}