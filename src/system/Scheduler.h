#pragma once
#include <Arduino.h>

class Scheduler {
   public:
    // Periods in microseconds. 0 = that channel is disabled (never ready).
    void begin(uint32_t controlPeriodUs, uint32_t imuPeriodUs);

    bool controlReady(uint32_t nowUs);
    bool imuReady(uint32_t nowUs);

   private:
    uint32_t controlPeriodUs_ = 0;
    uint32_t imuPeriodUs_ = 0;

    uint32_t nextControlUs_ = 0;
    uint32_t nextImuUs_ = 0;
};