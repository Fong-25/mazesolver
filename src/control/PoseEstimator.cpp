#include "PoseEstimator.h"

#include <math.h>

#include "../config/RobotConfig.h"
#include "../drivers/Encoder.h"

namespace {
    const float MM_PER_COUNT_LEFT = (PI * RobotConfig::LEFT_WHEEL_DIAMETER_MM) /
                                    RobotConfig::ENCODER_COUNTS_PER_OUTPUT_REV;
    const float MM_PER_COUNT_RIGHT =
        (PI * RobotConfig::RIGHT_WHEEL_DIAMETER_MM) /
        RobotConfig::ENCODER_COUNTS_PER_OUTPUT_REV;

    int32_t lastLeftCount = 0, lastRightCount = 0;
    uint32_t lastUpdateUs = 0;

    float xMm = 0.0f, yMm = 0.0f, thetaRad = 0.0f;

    float wrapAngle(float angle) {
        while (angle > PI) angle -= 2.0f * PI;
        while (angle < -PI) angle += 2.0f * PI;
        return angle;
    }
}

namespace PoseEstimator {
    void begin() {
        lastLeftCount = Encoder::LEFT_ENCODER_COUNT();
        lastRightCount = Encoder::RIGHT_ENCODER_COUNT();
        lastUpdateUs = micros();
        xMm = yMm = thetaRad = 0.0f;
    }

    void update(uint32_t nowUs) {
        if (nowUs == lastUpdateUs) return;  // called twice in the same instant
        lastUpdateUs = nowUs;

        int32_t leftCount = Encoder::LEFT_ENCODER_COUNT();
        int32_t rightCount = Encoder::RIGHT_ENCODER_COUNT();
        int32_t leftDelta = leftCount - lastLeftCount;
        int32_t rightDelta = rightCount - lastRightCount;
        lastLeftCount = leftCount;
        lastRightCount = rightCount;

        float dL = leftDelta * MM_PER_COUNT_LEFT;
        float dR = rightDelta * MM_PER_COUNT_RIGHT;

        float dCenter = (dR + dL) / 2.0f;
        float dTheta = (dR - dL) / RobotConfig::WHEEL_BASE_MM;

        // Use the midpoint heading during this tiny step, per spec section 11 —
        // more accurate than using the pre- or post-step heading alone.
        xMm += dCenter * cosf(thetaRad + dTheta / 2.0f);
        yMm += dCenter * sinf(thetaRad + dTheta / 2.0f);
        thetaRad = wrapAngle(thetaRad + dTheta);
    }

    float getXMm() { return xMm; }
    float getYMm() { return yMm; }
    float getThetaRad() { return thetaRad; }
    float getThetaDeg() { return thetaRad * 180.0f / PI; }

    void resetPose(float x, float y, float theta) {
        xMm = x;
        yMm = y;
        thetaRad = wrapAngle(theta);
    }
}