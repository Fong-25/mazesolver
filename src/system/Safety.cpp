#include "Safety.h"

#include <avr/wdt.h>

#include "../control/MotorControl.h"
#include "../drivers/BatteryMonitor.h"
#include "../drivers/Encoder.h"
#include "Diagnostics.h"

namespace {
    bool tripped = false;

    void trip(Diagnostics::ErrorCode code) {
        if (tripped)
            return;  // first cause wins — don't overwrite the real reason

        tripped = true;
        Diagnostics::setError(code);
        MotorControl::disable();  // step 1 of the spec's safety-stop procedure

        // RGB / Bluetooth reporting (steps 3-4) deliberately NOT done here.
        // RgbStatus (later) and Bluetooth (later) each poll
        // Safety::isTripped() + Diagnostics::getError() themselves — one
        // writer per output channel, not Safety pushing into two others.
    }
}

namespace Safety {
    void begin() {
        tripped = false;
        wdt_enable(WDTO_500MS);  // generous margin over CONTROL_PERIOD_US — see
                                 // SETUP_NOTES.md
    }

    void update(uint32_t nowUs) {
        (void)nowUs;  // unused for now, kept for signature consistency

        if (tripped) return;

        if (BatteryMonitor::isCritical()) {
            trip(Diagnostics::ErrorCode::BATTERY_CRITICAL);
            return;
        }

        // This is exactly what Encoder's glitch flag (built several
        // modules back) was for — spec 43's "impossible pose/speed
        // condition" IS this check, not a separate one to invent here.
        if (Encoder::consumeGlitchFlag()) {
            trip(Diagnostics::ErrorCode::ENCODER_INVALID);
            return;
        }
    }

    void kickWatchdog() { wdt_reset(); }

    void triggerUserAbort() { trip(Diagnostics::ErrorCode::USER_ABORT); }
    void triggerFault(Diagnostics::ErrorCode code) { trip(code); }

    bool isTripped() { return tripped; }

    void clearTrip() {
        tripped = false;
        Diagnostics::clearError();
        // MotorControl stays disabled — re-enabling drive is a deliberate
        // decision for ModeManager to make, not automatic on clearTrip().
    }
}