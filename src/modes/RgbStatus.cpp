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
}

namespace RgbStatus {
    void begin() {
        blinkOn = false;
        lastBlinkMs = millis();
        RGB::off();
    }

    void update(uint32_t nowMs) {
        using SystemState = ModeManager::SystemState;
        SystemState state = ModeManager::getState();

        switch (state) {
            case SystemState::BOOT:
            case SystemState::INIT:
                // TODO: short startup animation (spec's "Booting" row).
                RGB::off();
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