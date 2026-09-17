#pragma once
#include <stdint.h>
// SENSOR CONSTANTS
namespace SensorConfig {
    constexpr uint8_t TOF_DEFAULT_ADDRESS = 0x29;

    constexpr uint8_t TOF_ADDR_1 = 0x30;
    constexpr uint8_t TOF_ADDR_2 = 0x31;
    constexpr uint8_t TOF_ADDR_3 = 0x32;
    constexpr uint8_t TOF_ADDR_4 = 0x33;

    // "covered" = very close, per spec section 24
    constexpr uint16_t TOF_COVER_THRESHOLD_MM = 30;

    constexpr uint8_t IMU_I2C_ADDRESS = 0x68;

    // Gyro full-scale select register value + its matching sensitivity.
    // Keep these two paired — changing one without the other silently corrupts
    // every downstream deg/s reading.
    constexpr uint8_t IMU_GYRO_FS_SEL = 0x08;  // ±500 dps
    // matches FS_SEL above
    constexpr float IMU_GYRO_SENSITIVITY_LSB_PER_DPS = 65.5f;

    constexpr uint32_t I2C_CLOCK_HZ = 100000;

    constexpr uint16_t TOF_MIN_VALID_MM = 20;
    constexpr uint16_t TOF_MAX_VALID_MM = 2000;

    // Wall thresholds — MUST be calibrated on the real robot
    constexpr uint16_t FRONT_WALL_MM = 90;
    constexpr uint16_t SIDE_WALL_MAX_MM = 120;
}