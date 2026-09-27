#include "RgbStatus.h"

#include "../config/UserConfig.h"
#include "../drivers/RGB.h"
#include "ModeManager.h"
#include "SettingMode.h"
#include "StandbyMode.h"

namespace {
    struct Color {
        uint8_t r, g, b;
    };

    Color modeColor(ModeManager::RunMode mode) {
        switch (mode) {
            case ModeManager::RunMode::EXPLORE:
                return {UserConfig::RGB_EXPLORE_R, UserConfig::RGB_EXPLORE_G,
                        UserConfig::RGB_EXPLORE_B};
            case ModeManager::RunMode::FAST_RUN:
                return {UserConfig::RGB_FAST_RUN_R, UserConfig::RGB_FAST_RUN_G,
                        UserConfig::RGB_FAST_RUN_B};
            case ModeManager::RunMode::DIAGNOSTIC:
                return {UserConfig::RGB_DIAGNOSTIC_R,
                        UserConfig::RGB_DIAGNOSTIC_G,
                        UserConfig::RGB_DIAGNOSTIC_B};
            case ModeManager::RunMode::DEBUG_LOG:
                return {UserConfig::RGB_DEBUG_LOG_R,
                        UserConfig::RGB_DEBUG_LOG_G,
                        UserConfig::RGB_DEBUG_LOG_B};
            case ModeManager::RunMode::MOTION_TEST:
                return {UserConfig::RGB_MOTION_TEST_R,
                        UserConfig::RGB_MOTION_TEST_G,
                        UserConfig::RGB_MOTION_TEST_B};
            default:
                return {0, 0, 0};  // NONE -- nothing locked yet
        }
    }

    bool blinkOn = false;
    uint32_t lastBlinkMs = 0;

    // BOOT/INIT color-wheel state. Purely cosmetic (spec section 17's
    // "short startup animation") -- no meaning attached to position or
    // speed, just something visibly alive on the one pixel we have while
    // self-check runs, distinct from every locked-mode color so it can't
    // be mistaken for one.
    uint8_t bootWheelPos = 0;
    uint32_t lastBootStepMs = 0;

    void showSolid(Color c) { RGB::setColor(c.r, c.g, c.b); }

    void showBlinking(Color c, uint32_t nowMs) {
        if ((nowMs - lastBlinkMs) >= UserConfig::STANDBY_BLINK_INTERVAL_MS) {
            lastBlinkMs = nowMs;
            blinkOn = !blinkOn;
        }
        if (blinkOn) {
            showSolid(c);
        } else {
            RGB::off();
        }
    }

    // Classic 0-255 position -> RGB rainbow (red -> green -> blue -> red).
    // No floating point, no HSV math -- three linear ramps.
    Color wheel(uint8_t pos) {
        pos = 255 - pos;
        if (pos < 85) {
            return {(uint8_t)(255 - pos * 3), 0, (uint8_t)(pos * 3)};
        }
        if (pos < 170) {
            pos -= 85;
            return {0, (uint8_t)(pos * 3), (uint8_t)(255 - pos * 3)};
        }
        pos -= 170;
        return {(uint8_t)(pos * 3), (uint8_t)(255 - pos * 3), 0};
    }

    void showBootAnimation(uint32_t nowMs) {
        if ((nowMs - lastBootStepMs) >= UserConfig::BOOT_ANIMATION_STEP_MS) {
            lastBootStepMs = nowMs;
            bootWheelPos++;  // uint8_t: wraps 255 -> 0, loop is automatic
        }
        showSolid(wheel(bootWheelPos));
    }
}

namespace RgbStatus {
    void begin() {
        blinkOn = false;
        lastBlinkMs = millis();
        bootWheelPos = 0;
        lastBootStepMs = millis();
        RGB::off();
    }

    void update(uint32_t nowMs) {
        using SystemState = ModeManager::SystemState;
        SystemState state = ModeManager::getState();

        switch (state) {
            case SystemState::BOOT:
            case SystemState::INIT:
                showBootAnimation(nowMs);
                break;

            case SystemState::SETTING: {
                // Live feedback DURING setting, independent of ModeManager's
                // committed lockedMode -- spec 21.3: "leave the
                // corresponding mode color continuously ON" as soon as a
                // sequence matches this session, before DIP even flips off.
                int8_t sel = SettingMode::getSelectedModeIndex();
                if (sel >= 0) {
                    showSolid(modeColor((ModeManager::RunMode)sel));
                } else {
                    RGB::off();  // nothing matched yet this session
                }
                break;
            }

            case SystemState::LOCKED:
                showSolid(modeColor(ModeManager::getLockedMode()));
                break;

            case SystemState::STANDBY: {
                Color mc = modeColor(ModeManager::getLockedMode());
                // NEW: solid the instant a valid cover registers, not just
                // on release -- lets the user tell a good gesture from a
                // bad one before they've already committed to releasing.
                if (StandbyMode::isCoverConfirmed()) {
                    showSolid(mc);
                } else {
                    showBlinking(mc, nowMs);
                }
                break;
            }

            case SystemState::RUNNING:
                // Same color as LOCKED/STANDBY -- spec §62's "dedicated run
                // color" IS the mode color, no separate constant needed.
                showSolid(modeColor(ModeManager::getLockedMode()));
                break;

            case SystemState::FINISHED:
                showSolid({UserConfig::RGB_FINISHED_R,
                           UserConfig::RGB_FINISHED_G,
                           UserConfig::RGB_FINISHED_B});
                break;

            case SystemState::ERROR:
                showSolid({UserConfig::RGB_ERROR_R, UserConfig::RGB_ERROR_G,
                           UserConfig::RGB_ERROR_B});
                break;
        }
    }
}