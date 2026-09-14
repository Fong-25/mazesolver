#pragma once
#include <Arduino.h>

namespace MotorControl {
    void begin();

    // Target wheel speeds in mm/s. Positive = forward for that wheel.
    void setTargetSpeeds(float leftMmS, float rightMmS);

    // Call at the control period. Takes micros() — see timing note above.
    void update(uint32_t nowUs);

    void enable();
    void disable();  // stops motors immediately, resets both PIDs

    float getMeasuredLeftSpeedMmS();
    float getMeasuredRightSpeedMmS();

    int16_t getLastLeftPwm();
    int16_t getLastRightPwm();
}