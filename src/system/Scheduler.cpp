#include "Scheduler.h"

void Scheduler::begin(uint32_t controlPeriodUs, uint32_t imuPeriodUs) {
    controlPeriodUs_ = controlPeriodUs;
    imuPeriodUs_ = imuPeriodUs;

    uint32_t now = micros();
    nextControlUs_ = now + controlPeriodUs_;
    nextImuUs_ = now + imuPeriodUs_;
}

bool Scheduler::controlReady(uint32_t nowUs) {
    if (controlPeriodUs_ == 0) return false;

    // Signed-diff comparison handles micros() wraparound correctly without
    // special-casing it.
    if ((int32_t)(nowUs - nextControlUs_) >= 0) {
        // Advance by exactly one period, not "now + period" — keeps this
        // phase-locked to real elapsed time instead of drifting later
        // every time a call happens to run a bit late.
        nextControlUs_ += controlPeriodUs_;
        return true;
    }
    return false;
}

bool Scheduler::imuReady(uint32_t nowUs) {
    if (imuPeriodUs_ == 0) return false;

    if ((int32_t)(nowUs - nextImuUs_) >= 0) {
        nextImuUs_ += imuPeriodUs_;
        return true;
    }
    return false;
}