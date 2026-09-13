#include "DipSwitch.h"

#include "../config/BoardConfig.h"
#include "../config/UserConfig.h"

namespace {
    bool rawState = false;
    bool debouncedState = false;
    uint32_t lastChangeMs = 0;

    DipSwitch::Event pendingEvent = DipSwitch::Event::NONE;
}

namespace DipSwitch {
    void begin() {
        pinMode(Board::PIN_SETTING_SWITCH, INPUT_PULLUP);
        rawState = debouncedState =
            (digitalRead(Board::PIN_SETTING_SWITCH) == LOW);
        lastChangeMs = millis();
    }

    void update(uint32_t nowMs) {
        bool reading = (digitalRead(Board::PIN_SETTING_SWITCH) == LOW);

        if (reading != rawState) {
            rawState = reading;
            lastChangeMs = nowMs;
        }

        if ((nowMs - lastChangeMs) >= UserConfig::DIP_DEBOUNCE_MS &&
            debouncedState != rawState) {
            debouncedState = rawState;
            pendingEvent =
                debouncedState ? Event::ENTERED_SETTING : Event::LOCKED;
        }
    }

    Event getEvent() {
        Event e = pendingEvent;
        pendingEvent = Event::NONE;
        return e;
    }

    bool isSettingModeActive() { return debouncedState; }
}