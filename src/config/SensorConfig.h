#pragma once
#include <stdint.h>
// SENSOR CONSTANTS
namespace SensorConfig {
    constexpr uint8_t TOF_DEFAULT_ADDRESS = 0x29;

    constexpr uint8_t TOF_ADDR_1 = 0x30;
    constexpr uint8_t TOF_ADDR_2 = 0x31;
    constexpr uint8_t TOF_ADDR_3 = 0x32;
    constexpr uint8_t TOF_ADDR_4 = 0x33;

    constexpr uint32_t I2C_CLOCK_HZ = 100000;

    constexpr uint16_t TOF_MIN_VALID_MM = 20;
    constexpr uint16_t TOF_MAX_VALID_MM = 2000;

    // Wall thresholds — MUST be calibrated on the real robot
    constexpr uint16_t FRONT_WALL_MM = 90;
    constexpr uint16_t SIDE_WALL_MAX_MM = 120;
}