#include "BatteryMonitor.h"

#include "../config/BoardConfig.h"
#include "../config/RobotConfig.h"
#include "../config/UserConfig.h"

namespace {
    float filteredVoltage = 0.0f;
    uint32_t lastSampleMs = 0;
    bool initialized = false;
}

namespace BatteryMonitor {
    void begin() {
        pinMode(Board::PIN_BATTERY_ADC, INPUT);
        // Seed the filter with a real reading so it doesn't ramp up from 0
        // and falsely trip a low-battery flag at boot.
        int raw = analogRead(Board::PIN_BATTERY_ADC);
        float adcVoltage = raw * Board::ADC_REFERENCE_V / Board::ADC_MAX_COUNTS;
        filteredVoltage =
            adcVoltage *
            (Board::BATTERY_DIVIDER_R1_OHM + Board::BATTERY_DIVIDER_R2_OHM) /
            Board::BATTERY_DIVIDER_R2_OHM;
        initialized = true;
        lastSampleMs = millis();
    }

    void update(uint32_t nowMs) {
        if (!initialized ||
            (nowMs - lastSampleMs) < UserConfig::BATTERY_SAMPLE_INTERVAL_MS) {
            return;
        }
        lastSampleMs = nowMs;

        int raw = analogRead(Board::PIN_BATTERY_ADC);
        float adcVoltage = raw * Board::ADC_REFERENCE_V / Board::ADC_MAX_COUNTS;
        float rawBatteryVoltage =
            adcVoltage *
            (Board::BATTERY_DIVIDER_R1_OHM + Board::BATTERY_DIVIDER_R2_OHM) /
            Board::BATTERY_DIVIDER_R2_OHM;

        // Exponential filter — matches spec's suggested strategy
        filteredVoltage =
            RobotConfig::BATTERY_FILTER_ALPHA * rawBatteryVoltage +
            (1.0f - RobotConfig::BATTERY_FILTER_ALPHA) * filteredVoltage;
    }

    float getVoltage() { return filteredVoltage; }

    bool isLow() { return filteredVoltage <= RobotConfig::BATTERY_LOW_VOLTAGE; }

    bool isCritical() {
        return filteredVoltage <= RobotConfig::BATTERY_CRITICAL_VOLTAGE;
    }
}