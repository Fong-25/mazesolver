#pragma once

class PID {
   public:
    PID(float kp, float ki, float kd, float outputMin, float outputMax,
        float integralMin, float integralMax,
        float derivativeFilterAlpha = 1.0f);  // 1.0 = no filtering

    void setGains(float kp, float ki, float kd);
    void setEnabled(bool enabled);
    void reset();

    // dt in seconds. Sign-safe: works identically for negative
    // targets/measurements (reverse motion) since it's plain signed
    // arithmetic throughout, no abs()-clamped shortcuts.
    float update(float target, float measurement, float dt);

   private:
    float kp_, ki_, kd_;
    float outputMin_, outputMax_;
    float integralMin_, integralMax_;
    float derivativeFilterAlpha_;

    float integral_ = 0.0f;
    float filteredDerivative_ = 0.0f;
    float lastMeasurement_ = 0.0f;
    bool hasLastMeasurement_ = false;
    bool enabled_ = true;
};