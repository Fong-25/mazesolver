#pragma once
#include <stdint.h>
// SENSOR CONSTANTS
namespace SensorConfig {
    constexpr uint8_t TOF_DEFAULT_ADDRESS = 0x29;

    constexpr uint8_t TOF_ADDR_1 = 0x30;
    constexpr uint8_t TOF_ADDR_2 = 0x31;
    constexpr uint8_t TOF_ADDR_3 = 0x32;
    constexpr uint8_t TOF_ADDR_4 = 0x33;

    // Per-physical-sensor distance correction, indexed identically to
    // ToFManager's XSHUT_PINS/TARGET_ADDR arrays above (physical sensor
    // 0-3, NOT logical role -- the bias belongs to that specific sensor
    // unit, not to wherever it happens to be mounted). Added to every raw
    // reading before anything else -- validity check, wall thresholds,
    // everything downstream -- ever sees it, so the fix lives here once
    // instead of getting compensated for all over Explorer/FloodFill.
    //
    // TODO: derive from testing -- hold all 4 sensors at the same known
    // distance, note each raw reading, pick one of them (or their
    // average) as the reference, then offset[i] = reference - raw[i].
    // All zero until measured -- confirmed non-uniform on the bench
    // (e.g. two agreeing, one under-reading, one over-reading by ~10mm).
    constexpr int16_t TOF_OFFSET_MM_1 = 0;
    constexpr int16_t TOF_OFFSET_MM_2 = 0;
    constexpr int16_t TOF_OFFSET_MM_3 = 0;
    constexpr int16_t TOF_OFFSET_MM_4 = 0;

    // Diagonal sensors are mounted at an angle to the robot's
    // forward-travel axis, not pointing straight sideways -- their raw
    // reading is a SLANT range along that angled beam, not the true
    // perpendicular distance to a side wall. Converting once here means
    // SIDE_WALL_MAX_MM (and anything else reading DIAGONAL_LEFT/RIGHT)
    // can be tuned against an intuitive "actual distance to the wall"
    // number instead of an angle-dependent slant range that gets longer
    // just because the mount angle is shallower.
    //
    // perpendicularMm = slantRangeMm * sin(mountAngleDeg), where
    // mountAngleDeg is measured from the forward-travel axis (0deg =
    // pointing straight ahead, 90deg = pointing straight sideways).
    // Applied only to the two DIAGONAL_* roles -- FRONT_LEFT/FRONT_RIGHT
    // stay raw, since they're only ever used as presence/absence
    // thresholds, not as a physically-meaningful distance.
    //
    // TODO: confirm against the actual PCB/mechanical mounting angle --
    // 45deg is a common default assumption for a "diagonal" sensor, not
    // a measurement of this specific board.
    constexpr float TOF_DIAGONAL_MOUNT_ANGLE_DEG = 45.0f;

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