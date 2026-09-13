#pragma once
#include <Arduino.h>

namespace MotorDriver {
    void begin();

    // Signed PWM: positive = robot-forward, negative = robot-reverse,
    // 0 = coast. Motor polarity (RobotConfig::*_MOTOR_REVERSED) is
    // resolved internally — callers never think about wiring direction.
    void setLeftPWM(int16_t pwm);
    void setRightPWM(int16_t pwm);

    void brakeLeft();
    void brakeRight();

    void coastLeft();
    void coastRight();

    // Safety default — used by Safety module and the BT STOP command.
    void stopAll();
}