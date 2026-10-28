#include "SettingMode.h"

#include "../config/RobotConfig.h"
#include "../config/UserConfig.h"
#include "../drivers/Encoder.h"

namespace {

    // uint32_t lastGestureMs = 0;
    int32_t referenceCount =
        0;  // wheel count at the start of the current stroke
    int32_t lastSeenCount = 0;  // last count at which the wheel was seen moving
    uint32_t lastMoveMs = 0;    // last time the wheel moved

    UserConfig::WheelGesture buffer[UserConfig::MODE_SEQUENCE_LENGTH];
    uint8_t bufferLength = 0;

    int8_t selectedModeIndex = -1;

    int32_t currentWheelCount() {
        return RobotConfig::MODE_SELECT_USE_LEFT_WHEEL
                   ? Encoder::LEFT_ENCODER_COUNT()
                   : Encoder::RIGHT_ENCODER_COUNT();
    }

    bool matchesPrefix(const UserConfig::ModeSequenceEntry& entry) {
        for (uint8_t i = 0; i < bufferLength; i++) {
            if (entry.sequence[i] != buffer[i]) return false;
        }
        return true;
    }

    void checkForMatch() {
        bool anyPrefixStillValid = false;

        for (uint8_t s = 0; s < UserConfig::MODE_SEQUENCE_COUNT; s++) {
            const auto& entry = UserConfig::MODE_SEQUENCES[s];
            if (matchesPrefix(entry)) {
                anyPrefixStillValid = true;
                if (bufferLength == UserConfig::MODE_SEQUENCE_LENGTH) {
                    selectedModeIndex =
                        entry
                            .modeIndex;  // update to the newest confirmed match
                }
            }
        }

        // Reset the buffer whenever we've either fully matched something OR
        // ruled out every candidate — either way, start fresh. This is what
        // lets you change your mind: selectedModeIndex keeps the last
        // CONFIRMED selection (so RGB stays showing it), but the gesture
        // buffer itself always goes back to listening immediately, instead of
        // permanently freezing after the first successful selection.
        if (!anyPrefixStillValid ||
            bufferLength == UserConfig::MODE_SEQUENCE_LENGTH) {
            bufferLength = 0;
        }
    }
}

namespace SettingMode {
    void begin() { reset(); }

    void reset() {
        referenceCount = lastSeenCount = currentWheelCount();
        bufferLength = 0;
        lastMoveMs = millis();
        selectedModeIndex = -1;
    }

    // A gesture is one STROKE: the wheel moves, then pauses. Direction and
    // size are judged when it pauses, not the instant a threshold is
    // crossed, so overshoot, a half turn, or a sloppy quarter turn all
    // count as exactly one gesture and nothing carries into the next one.
    void update(uint32_t nowMs) {
        int32_t current = currentWheelCount();

        int32_t moved = current - lastSeenCount;
        if (moved > (int32_t)UserConfig::MODE_GESTURE_MOTION_EPS_COUNTS ||
            moved < -(int32_t)UserConfig::MODE_GESTURE_MOTION_EPS_COUNTS) {
            lastSeenCount = current;
            lastMoveMs = nowMs;
        }

        uint32_t idleMs = nowMs - lastMoveMs;
        if (idleMs < UserConfig::MODE_GESTURE_SETTLE_MS) return;  // mid-stroke

        int32_t delta = lastSeenCount - referenceCount;
        int32_t minCounts = (int32_t)RobotConfig::MODE_GESTURE_MIN_COUNTS;
        if (delta >= minCounts || delta <= -minCounts) {
            UserConfig::WheelGesture gesture =
                (delta > 0) ? UserConfig::WheelGesture::FORWARD
                            : UserConfig::WheelGesture::BACKWARD;
            if (bufferLength < UserConfig::MODE_SEQUENCE_LENGTH) {
                buffer[bufferLength++] = gesture;
            }
            checkForMatch();
        }
        referenceCount = lastSeenCount;  // next stroke measured fresh

        if (bufferLength > 0 && idleMs > UserConfig::MODE_GESTURE_TIMEOUT_MS) {
            bufferLength = 0;
        }
    }

    int8_t getSelectedModeIndex() { return selectedModeIndex; }
}