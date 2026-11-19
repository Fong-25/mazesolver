#include "ToFManager.h"

#include <VL53L0X.h>
#include <Wire.h>
#include <math.h>

#include "../config/BoardConfig.h"
#include "../config/RobotConfig.h"
#include "../config/SensorConfig.h"
#include "../config/UserConfig.h"

namespace {
    constexpr uint8_t MAX_SENSORS = 4;

    // Physical indices that carry the diagonal slant->perpendicular
    // correction below -- must match RobotConfig::TOF_ROLE_INDEX_DIAGONAL_*.
    constexpr uint8_t DIAGONAL_LEFT_IDX =
        RobotConfig::TOF_ROLE_INDEX_DIAGONAL_LEFT;
    constexpr uint8_t DIAGONAL_RIGHT_IDX =
        RobotConfig::TOF_ROLE_INDEX_DIAGONAL_RIGHT;
    // sin() isn't constexpr in every toolchain -- computed once at
    // startup instead of per-reading.
    const float DIAGONAL_SIN_MOUNT_ANGLE =
        sinf(SensorConfig::TOF_DIAGONAL_MOUNT_ANGLE_DEG * PI / 180.0f);

    const uint8_t XSHUT_PINS[MAX_SENSORS] = {
        Board::PIN_TOF_XSHUT_1, Board::PIN_TOF_XSHUT_2, Board::PIN_TOF_XSHUT_3,
        Board::PIN_TOF_XSHUT_4};

    const uint8_t TARGET_ADDR[MAX_SENSORS] = {
        SensorConfig::TOF_ADDR_1, SensorConfig::TOF_ADDR_2,
        SensorConfig::TOF_ADDR_3, SensorConfig::TOF_ADDR_4};

    const int16_t TOF_OFFSET_MM[MAX_SENSORS] = {
        SensorConfig::TOF_OFFSET_MM_1, SensorConfig::TOF_OFFSET_MM_2,
        SensorConfig::TOF_OFFSET_MM_3, SensorConfig::TOF_OFFSET_MM_4};

    VL53L0X sensors[MAX_SENSORS];
    bool sensorOk[MAX_SENSORS] = {false, false, false, false};
    uint16_t lastDistanceMm[MAX_SENSORS] = {0, 0, 0, 0};
    bool lastValid[MAX_SENSORS] = {false, false, false, false};
    uint8_t sampleCount[MAX_SENSORS] = {0, 0, 0, 0};
    uint8_t consecutiveTimeouts[MAX_SENSORS] = {0, 0, 0, 0};
    uint16_t sampleMs[MAX_SENSORS] = {0, 0, 0, 0};  // low 16 bits of millis()

    // Runtime read timeout. A blocking read in update() stalls the WHOLE
    // loop for up to this long, and the watchdog is 500 ms (Safety.cpp):
    // the 500 ms used during init would turn one dead sensor into a
    // watchdog reset mid-run. Continuous mode delivers a result about every
    // 33 ms and each sensor is polled every 4 * TOF_POLL_INTERVAL_MS, so
    // data is normally already waiting and this never costs anything.
    constexpr uint16_t RUNTIME_TIMEOUT_MS = 30;
    constexpr uint8_t RUNTIME_TIMEOUT_LIMIT = 3;  // consecutive -> invalid

    uint8_t pollCursor = 0;
    uint32_t lastPollMs = 0;

    uint8_t roleToIndex(ToFManager::SensorRole role) {
        switch (role) {
            case ToFManager::SensorRole::FRONT_LEFT:
                return RobotConfig::TOF_ROLE_INDEX_FRONT_LEFT;
            case ToFManager::SensorRole::FRONT_RIGHT:
                return RobotConfig::TOF_ROLE_INDEX_FRONT_RIGHT;
            case ToFManager::SensorRole::DIAGONAL_LEFT:
                return RobotConfig::TOF_ROLE_INDEX_DIAGONAL_LEFT;
            case ToFManager::SensorRole::DIAGONAL_RIGHT:
                return RobotConfig::TOF_ROLE_INDEX_DIAGONAL_RIGHT;
            default:
                return 0xFF;
        }
    }

    void pollOneSensor(uint8_t i) {
        if (!sensorOk[i]) return;

        uint16_t rawMm = sensors[i].readRangeContinuousMillimeters();

        if (sensors[i].timeoutOccurred()) {
            // One bad I2C cycle -> keep last known good value rather than
            // feeding a garbage reading into navigation. But a sensor that
            // keeps timing out is dead, and its frozen last value would
            // otherwise be trusted forever -- after a few in a row, report
            // it invalid.
            if (consecutiveTimeouts[i] < 255) consecutiveTimeouts[i]++;
            if (consecutiveTimeouts[i] >= RUNTIME_TIMEOUT_LIMIT) {
                lastValid[i] = false;
            }
            return;
        }
        consecutiveTimeouts[i] = 0;

        // Per-sensor hardware bias correction (SensorConfig::TOF_OFFSET_MM_*)
        // -- applied once, here, so every consumer above this driver
        // always sees an already-corrected distance and never has to
        // know these four units don't agree with each other raw.
        int32_t corrected = (int32_t)rawMm + TOF_OFFSET_MM[i];
        if (corrected < 0) corrected = 0;

        // Diagonal-only: slant range -> true perpendicular wall distance
        // (see SensorConfig::TOF_DIAGONAL_MOUNT_ANGLE_DEG's comment).
        // Applied after the offset correction, on the corrected value.
        if (i == DIAGONAL_LEFT_IDX || i == DIAGONAL_RIGHT_IDX) {
            corrected = (int32_t)((float)corrected * DIAGONAL_SIN_MOUNT_ANGLE);
        }

        lastDistanceMm[i] = (uint16_t)corrected;
        // Validity is judged on the RAW reading, before the per-sensor
        // offset: the offset can legitimately pull a good close-range
        // reading below TOF_MIN_VALID_MM (raw 38 mm with a -20 mm offset is
        // a perfectly good 18 mm), and a front sensor at the cell center
        // with a wall ahead reads exactly that low.
        lastValid[i] = (rawMm >= SensorConfig::TOF_MIN_VALID_MM &&
                        rawMm <= SensorConfig::TOF_MAX_VALID_MM);

        sampleCount[i]++;
        sampleMs[i] = (uint16_t)lastPollMs;  // set by update() just before
    }
}

namespace ToFManager {
    bool begin() {
        // Wire.begin();
        // IMU is added on the same bus, Wire.begin() must only be
        // called ONCE for the whole firmware. Move this call up to main
        // setup() at that point — don't leave two drivers both owning it.
        Wire.setClock(SensorConfig::I2C_CLOCK_HZ);

        // Steps 1-2 (spec 13.1): hold every sensor in reset, no exceptions,
        // including ones we're not going to install this build.
        for (uint8_t i = 0; i < MAX_SENSORS; i++) {
            pinMode(XSHUT_PINS[i], OUTPUT);
            digitalWrite(XSHUT_PINS[i], LOW);
        }
        delay(10);  // Step 3: let them fully power down before anyone boots

        bool allOk = true;

        for (uint8_t i = 0; i < RobotConfig::TOF_INSTALLED_COUNT; i++) {
            // Step 4-6: enable ONLY this sensor. Every other sensor is still
            // held LOW, so this one is guaranteed to be the only device
            // answering at the default 0x29 address right now.
            digitalWrite(XSHUT_PINS[i], HIGH);
            delay(10);  // boot time before it'll answer on I2C

            sensors[i].setTimeout(500);

            if (!sensors[i].init()) {
                // Don't let one dead sensor block bringing up the rest.
                sensorOk[i] = false;
                allOk = false;
                continue;
            }

            sensors[i].setAddress(
                TARGET_ADDR[i]);  // Step 6/9: claim unique address
            sensors[i].startContinuous();
            sensors[i].setTimeout(RUNTIME_TIMEOUT_MS);  // 500
            sensorOk[i] = true;
        }

        // Anything beyond TOF_INSTALLED_COUNT stays held in reset forever —
        // it's simply not part of this build (spec: 4th sensor optional).
        return allOk;
    }

    void update(uint32_t nowMs) {
        if ((nowMs - lastPollMs) < UserConfig::TOF_POLL_INTERVAL_MS) {
            return;
        }
        lastPollMs = nowMs;

        pollOneSensor(pollCursor);
        pollCursor = (pollCursor + 1) % RobotConfig::TOF_INSTALLED_COUNT;
    }

    uint16_t getDistanceMm(SensorRole role) {
        uint8_t idx = roleToIndex(role);
        if (idx >= MAX_SENSORS || !sensorOk[idx]) {
            return SensorConfig::TOF_MAX_VALID_MM;  // "clear" — no wall
                                                    // detected
        }
        return lastDistanceMm[idx];
    }

    bool isValid(SensorRole role) {
        uint8_t idx = roleToIndex(role);
        if (idx >= MAX_SENSORS || !sensorOk[idx]) return false;
        return lastValid[idx];
    }

    uint8_t getSampleCount(SensorRole role) {
        uint8_t idx = roleToIndex(role);
        if (idx >= MAX_SENSORS || !sensorOk[idx]) return 0;
        return sampleCount[idx];
    }

    uint16_t getSampleAgeMs(SensorRole role, uint32_t nowMs) {
        uint8_t idx = roleToIndex(role);
        if (idx >= MAX_SENSORS || !sensorOk[idx]) return 0xFFFF;
        return (uint16_t)((uint16_t)nowMs - sampleMs[idx]);
    }
}