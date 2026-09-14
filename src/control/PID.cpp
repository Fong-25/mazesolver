#include "PID.h"

PID::PID(float kp, float ki, float kd, float outputMin, float outputMax,
         float integralMin, float integralMax, float derivativeFilterAlpha)
    : kp_(kp),
      ki_(ki),
      kd_(kd),
      outputMin_(outputMin),
      outputMax_(outputMax),
      integralMin_(integralMin),
      integralMax_(integralMax),
      derivativeFilterAlpha_(derivativeFilterAlpha) {}

void PID::setGains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PID::setEnabled(bool enabled) {
    enabled_ = enabled;
    if (!enabled_) reset();
}

void PID::reset() {
    integral_ = 0.0f;
    filteredDerivative_ = 0.0f;
    hasLastMeasurement_ = false;
}

float PID::update(float target, float measurement, float dt) {
    if (!enabled_ || dt <= 0.0f) {
        return 0.0f;
    }

    float error = target - measurement;

    integral_ += error * dt;
    if (integral_ > integralMax_) integral_ = integralMax_;
    if (integral_ < integralMin_) integral_ = integralMin_;

    // Derivative-on-measurement, not on error — avoids "derivative kick"
    // when the target itself changes abruptly (a new speed command).
    float rawDerivative = 0.0f;
    if (hasLastMeasurement_) {
        rawDerivative = -(measurement - lastMeasurement_) / dt;
    }
    lastMeasurement_ = measurement;
    hasLastMeasurement_ = true;

    filteredDerivative_ = derivativeFilterAlpha_ * rawDerivative +
                          (1.0f - derivativeFilterAlpha_) * filteredDerivative_;

    float output = kp_ * error + ki_ * integral_ + kd_ * filteredDerivative_;

    if (output > outputMax_) output = outputMax_;
    if (output < outputMin_) output = outputMin_;

    return output;
}