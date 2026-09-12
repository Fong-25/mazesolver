#include "Button.h"

#include "../config/BoardConfig.h"
#include "../config/UserConfig.h"

namespace {
    bool rawState = false;  // true = press
    bool debouncedState = false;
    bool lastDebounced = false;
    uint32_t lastChangeMs = 0;

    bool longPressFired = false;
    uint32_t pressStartMs = 0;

    Button::Event pendingEvent = Button::Event::NONE;
}

namespace Button {
    void begin() {
        pinMode(Board::PIN_BUTTON, INPUT_PULLUP);
        rawState = debouncedState = lastDebounced =
            (digitalRead(Board::PIN_BUTTON) == LOW);
        lastChangeMs = millis();
    }

    void update(uint32_t nowMs) {
        bool reading = (digitalRead(Board::PIN_BUTTON) == LOW);  // active-low

        if (reading != rawState) {
            rawState = reading;
            lastChangeMs = nowMs;
        }

        if ((nowMs - lastChangeMs) >= UserConfig::BUTTON_DEBOUNCE_MS) {
            debouncedState = rawState;
        }

        // Edge detection on the debounced signal
        if (debouncedState != lastDebounced) {
            if (debouncedState) {
                // just pressed
                pressStartMs = nowMs;
                longPressFired = false;
                pendingEvent = Event::PRESS;
            } else {
                // just released
                pendingEvent = Event::RELEASE;
            }
            lastDebounced = debouncedState;
        } else if (debouncedState && !longPressFired) {
            if ((nowMs - pressStartMs) >= UserConfig::BUTTON_LONG_PRESS_MS) {
                longPressFired = true;
                pendingEvent = Event::LONG_PRESS;
            }
        }
    }

    Event getEvent() {
        Event e = pendingEvent;
        pendingEvent = Event::NONE;
        return e;
    }

    bool isPressed() { return debouncedState; }
}