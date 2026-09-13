#include "IMU.h"

#include <Wire.h>

#include "../config/BoardConfig.h"
#include "../config/RobotConfig.h"
#include "../config/SensorConfig.h"

namespace {
    constexpr uint8_t REG_WHO_AM_I = 0x75;
    constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
    constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
    constexpr uint8_t REG_GYRO_XOUT_H = 0x43;
    constexpr uint8_t REG_GYRO_YOUT_H = 0x45;
    constexpr uint8_t REG_GYRO_ZOUT_H = 0x47;

    float biasDegPerSec = 0.0f;
    float yawRateDegPerSec = 0.0f;
    float yawDeg = 0.0f;
    uint32_t lastUpdateMs = 0;

    void writeReg(uint8_t reg, uint8_t value) {
        Wire.beginTransmission(SensorConfig::IMU_I2C_ADDRESS);
        Wire.write(reg);
        Wire.write(value);
        Wire.endTransmission();
    }

    uint8_t readReg(uint8_t reg) {
        Wire.beginTransmission(SensorConfig::IMU_I2C_ADDRESS);
        Wire.write(reg);
        Wire.endTransmission(false);
        Wire.requestFrom(SensorConfig::IMU_I2C_ADDRESS, (uint8_t)1);
        return Wire.read();
    }

    int16_t readAxisRaw(uint8_t highRegAddr) {
        Wire.beginTransmission(SensorConfig::IMU_I2C_ADDRESS);
        Wire.write(highRegAddr);
        Wire.endTransmission(false);
        Wire.requestFrom(SensorConfig::IMU_I2C_ADDRESS, (uint8_t)2);
        int16_t hi = Wire.read();
        int16_t lo = Wire.read();
        return (hi << 8) | lo;
    }

    uint8_t yawAxisRegister() {
        switch (RobotConfig::IMU_YAW_AXIS) {
            case RobotConfig::ImuAxis::X:
                return REG_GYRO_XOUT_H;
            case RobotConfig::ImuAxis::Y:
                return REG_GYRO_YOUT_H;
            case RobotConfig::ImuAxis::Z:
            default:
                return REG_GYRO_ZOUT_H;
        }
    }

    float readYawRateRawDegPerSec() {
        int16_t raw = readAxisRaw(yawAxisRegister());
        float dps = raw / SensorConfig::IMU_GYRO_SENSITIVITY_LSB_PER_DPS;
        return dps * RobotConfig::IMU_YAW_SIGN;
    }
}

namespace IMU {
    bool begin() {
        writeReg(REG_PWR_MGMT_1, 0x00);  // wake the device from sleep
        delay(50);                       // let it settle after wake
        writeReg(REG_GYRO_CONFIG, SensorConfig::IMU_GYRO_FS_SEL);

        lastUpdateMs = millis();
        return true;  // not gated on WHO_AM_I — see header note
    }

    void calibrateBias() {
        // Blocking is acceptable here: this runs once at boot, before the
        // main loop starts, not during normal operation (different from
        // the "don't block the main loop" rule elsewhere in the spec).
        constexpr uint16_t SAMPLES = 200;
        float sum = 0.0f;

        for (uint16_t i = 0; i < SAMPLES; i++) {
            sum += readYawRateRawDegPerSec();
            delay(2);
        }

        biasDegPerSec = sum / SAMPLES;
    }

    void resetYawDeg() { yawDeg = 0.0f; }

    void update(uint32_t nowMs) {
        float dtSec = (nowMs - lastUpdateMs) / 1000.0f;
        lastUpdateMs = nowMs;

        yawRateDegPerSec = readYawRateRawDegPerSec() - biasDegPerSec;
        yawDeg += yawRateDegPerSec * dtSec;
    }

    float getYawRateDegPerSec() { return yawRateDegPerSec; }
    float getYawDeg() { return yawDeg; }
    uint8_t getWhoAmI() { return readReg(REG_WHO_AM_I); }
}