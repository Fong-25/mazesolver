#include "ModeManager.h"

#include "../communication/Bluetooth.h"
#include "../drivers/Button.h"
#include "../drivers/DipSwitch.h"
#include "../navigation/Explorer.h"
#include "../navigation/FastRun.h"
#include "../system/Safety.h"
#include "SettingMode.h"
#include "StandbyMode.h"

namespace {
    ModeManager::SystemState state = ModeManager::SystemState::BOOT;
    ModeManager::RunMode lockedMode = ModeManager::RunMode::NONE;

    void enterSetting() {
        state = ModeManager::SystemState::SETTING;
        SettingMode::begin();
    }

    void enterLocked() { state = ModeManager::SystemState::LOCKED; }

    void enterStandby() {
        state = ModeManager::SystemState::STANDBY;
        StandbyMode::begin();
    }

    // Bench-testing shortcut (MODES_AND_BLUETOOTH_PROTOCOL.md): BT's
    // MODE <n> sets the locked mode directly, bypassing the wheel-gesture
    // dance. Only honored while nothing is armed/running -- same "must not
    // move automatically" principle SettingMode/StandbyMode already
    // follow. Silently dropped otherwise: a bench tester will notice via
    // STATUS/RGB that it didn't take, and ModeManager has no business
    // reaching into Bluetooth's Serial output to explain why.
    void applyBluetoothModeOverride() {
        uint8_t idx;
        if (!Bluetooth::consumeModeRequest(idx)) return;

        bool canOverrideNow = state == ModeManager::SystemState::LOCKED ||
                              state == ModeManager::SystemState::SETTING;

        if (canOverrideNow &&
            idx <= (uint8_t)ModeManager::RunMode::MOTION_TEST) {
            lockedMode = (ModeManager::RunMode)idx;
        }
    }
}

namespace ModeManager {
    void begin() {
        lockedMode = RunMode::NONE;
        enterLocked();
    }

    void update(uint32_t nowMs) {
        // Highest priority, every tick, from any state but ERROR itself:
        // a Safety trip always wins the tick it happens on. Motors are
        // already disabled by Safety::trip() -- this just makes the
        // top-level state machine agree with reality.
        if (state != SystemState::ERROR && Safety::isTripped()) {
            state = SystemState::ERROR;
        }

        // Drained once per tick regardless of state, so an event that
        // arrives while we're not listening for it doesn't linger and get
        // misread on some later tick after a transition.
        DipSwitch::Event dipEvent = DipSwitch::getEvent();
        Button::Event btnEvent = Button::getEvent();

        applyBluetoothModeOverride();

        switch (state) {
            case SystemState::BOOT:
            case SystemState::INIT:
                // main.cpp's setup() sequence IS boot/init right now --
                // nothing to do until a self-check step actually lands
                // here.
                break;

            case SystemState::LOCKED:
                if (dipEvent == DipSwitch::Event::ENTERED_SETTING) {
                    enterSetting();
                } else if (btnEvent == Button::Event::LONG_PRESS &&
                           lockedMode != RunMode::NONE) {
                    enterStandby();
                }
                break;

            case SystemState::SETTING:
                SettingMode::update(nowMs);
                if (dipEvent == DipSwitch::Event::LOCKED) {
                    int8_t selected = SettingMode::getSelectedModeIndex();
                    // -1 means the DIP flipped off mid-gesture with
                    // nothing confirmed this session -- keep whatever was
                    // locked before instead of clearing it.
                    if (selected >= 0) lockedMode = (RunMode)selected;
                    enterLocked();
                }
                break;

            case SystemState::STANDBY:
                StandbyMode::update(nowMs);
                if (StandbyMode::isRunTriggered()) {
                    state = SystemState::RUNNING;
                    if (lockedMode == RunMode::EXPLORE) {
                        Explorer::begin();
                    }
                    // TODO: FAST_RUN/DIAGNOSTIC/DEBUG_LOG/MOTION_TEST dispatch
                    // once those modules exist -- RUNNING just sits idle for
                    // them for now, same as before.
                }
                break;

            case SystemState::RUNNING:
                if (lockedMode == RunMode::EXPLORE) {
                    Explorer::update(nowMs);
                    if (Explorer::isDone()) {
                        state = SystemState::FINISHED;
                    }
                } else if (lockedMode == RunMode::FAST_RUN) {
                    FastRun::update(nowMs);
                    if (FastRun::isDone()) {
                        state = SystemState::FINISHED;
                    }
                }
                // TODO: same dispatch gap as STANDBY's above for the other four
                // modes -- nothing to delegate update() to yet.
                break;

            case SystemState::FINISHED:
                // Spec: "-> LOCKED after acknowledgement/power-cycle."
                // Picking a short PRESS as that acknowledgement --
                // deliberately NOT the 3s LONG_PRESS STANDBY uses, so
                // acking a finished run can't double as re-arming one.
                // Easy to swap later.
                if (btnEvent == Button::Event::PRESS) enterLocked();
                break;

            case SystemState::ERROR:
                // No automatic recovery -- Safety::clearTrip() is an
                // explicit action by its own header's contract. Until
                // something calls it, isTripped() keeps returning true
                // and we just sit here.
                if (!Safety::isTripped()) enterLocked();
                break;
        }
    }

    SystemState getState() { return state; }
    RunMode getLockedMode() { return lockedMode; }
}