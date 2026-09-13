#include "MotorDriver.h"

#include "../config/BoardConfig.h"
#include "../config/RobotConfig.h"

namespace {
    int16_t clampPwm(int16_t pwm) {
        if (pwm > Board::PWM_MAX) return Board::PWM_MAX;
        if (pwm < -Board::PWM_MAX) return -Board::PWM_MAX;
        return pwm;
    }

    // Standard DRV8833 IN/IN drive: one pin PWM, other pin LOW for a given
    // direction; both LOW = coast; both HIGH = active brake.
    void drive(uint8_t in1, uint8_t in2, int16_t pwm) {
        pwm = clampPwm(pwm);

        if (pwm > 0) {
            analogWrite(in1, pwm);
            digitalWrite(in2, LOW);
        } else if (pwm < 0) {
            digitalWrite(in1, LOW);
            analogWrite(in2, -pwm);
        } else {
            digitalWrite(in1, LOW);
            digitalWrite(in2, LOW);  // coast
        }
    }

    void brake(uint8_t in1, uint8_t in2) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, HIGH);
    }
}

namespace MotorDriver {
    void begin() {
        pinMode(Board::PIN_LEFT_IN1, OUTPUT);
        pinMode(Board::PIN_LEFT_IN2, OUTPUT);
        pinMode(Board::PIN_RIGHT_IN1, OUTPUT);
        pinMode(Board::PIN_RIGHT_IN2, OUTPUT);

        stopAll();  // never boot with motors in an undefined state
    }

    void setLeftPWM(int16_t pwm) {
        if (RobotConfig::LEFT_MOTOR_REVERSED) pwm = -pwm;
        drive(Board::PIN_LEFT_IN1, Board::PIN_LEFT_IN2, pwm);
    }

    void setRightPWM(int16_t pwm) {
        if (RobotConfig::RIGHT_MOTOR_REVERSED) pwm = -pwm;
        drive(Board::PIN_RIGHT_IN1, Board::PIN_RIGHT_IN2, pwm);
    }

    void brakeLeft() { brake(Board::PIN_LEFT_IN1, Board::PIN_LEFT_IN2); }
    void brakeRight() { brake(Board::PIN_RIGHT_IN1, Board::PIN_RIGHT_IN2); }

    void coastLeft() { drive(Board::PIN_LEFT_IN1, Board::PIN_LEFT_IN2, 0); }
    void coastRight() { drive(Board::PIN_RIGHT_IN1, Board::PIN_RIGHT_IN2, 0); }

    void stopAll() {
        coastLeft();
        coastRight();
    }
}