#pragma once
#include <Arduino.h>

// Only use Encoder for estimate here
namespace PoseEstimator {
    void begin();

    // Call at the control period, same cadence as MotorControl::update().
    void update(uint32_t nowUs);

    float getXMm();
    float getYMm();
    float getThetaRad();
    float getThetaDeg();

    void resetPose(float xMm = 0.0f, float yMm = 0.0f, float thetaRad = 0.0f);
}