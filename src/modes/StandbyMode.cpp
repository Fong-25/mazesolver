#include "StandbyMode.h"

#include "../config/SensorConfig.h"
#include "../config/UserConfig.h"
#include "../drivers/ToFManager.h"

namespace {
    enum class SubState { WAIT_FOR_COVER, COVERED };

    SubState state = SubState::WAIT_FOR_COVER;
    bool hasSeenOpen = false;
    bool runTriggered = false;

    ToFManager::SensorRole startSensorRole() {
        switch (UserConfig::START_SENSOR_ROLE_INDEX) {
            case 0:
                return ToFManager::SensorRole::FRONT_LEFT;
            case 1:
                return ToFManager::SensorRole::FRONT_RIGHT;
            case 2:
                return ToFManager::SensorRole::DIAGONAL_LEFT;
            case 3:
                return ToFManager::SensorRole::DIAGONAL_RIGHT;
            default:
                return ToFManager::SensorRole::FRONT_LEFT;
        }
    }

    bool isCoveredNow() {
        // Deliberately raw getDistanceMm(), NOT isValid() — see rationale
        // above. A genuine cover event is well below the normal validity
        // floor by design.
        uint16_t mm = ToFManager::getDistanceMm(startSensorRole());
        return mm <= SensorConfig::TOF_COVER_THRESHOLD_MM;
    }
}

namespace StandbyMode {
    void begin() {
        state = SubState::WAIT_FOR_COVER;
        // Must observe "open" before a cover counts — guards against the
        // sensor already being covered the instant STANDBY begins.
        hasSeenOpen = false;
        runTriggered = false;
    }

    void update(uint32_t nowMs) {
        (void)nowMs;  // pure level/edge logic, no timing needed yet
        if (runTriggered) return;

        bool covered = isCoveredNow();

        switch (state) {
            case SubState::WAIT_FOR_COVER:
                if (!covered) {
                    hasSeenOpen = true;
                } else if (hasSeenOpen) {
                    state = SubState::COVERED;
                }
                break;

            case SubState::COVERED:
                if (!covered) {
                    runTriggered = true;  // valid cover-then-release complete
                }
                break;
        }
    }

    bool isRunTriggered() { return runTriggered; }

    bool isCoverConfirmed() { return state == SubState::COVERED; }
}