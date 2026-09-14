#pragma once
#include <Arduino.h>

#include "Diagnostics.h"

namespace Safety {
    // Enables the hardware watchdog. CRITICAL PRECONDITION: main.cpp must
    // clear MCUSR and call wdt_disable() as the very FIRST lines of
    // setup(), before this or anything else runs — see SETUP_NOTES.md,
    // this is a well-known AVR/Optiboot gotcha that can brick the upload
    // path if skipped.
    void begin();

    // Call every loop tick, unconditionally — same pattern as
    // Encoder::service().
    void update(uint32_t nowUs);

    // Call ONLY when the caller has confirmed the system is healthy this
    // cycle — never unconditionally. See spec section 44's explicit warning
    // against kicking from "one deeply nested subsystem" blindly.
    void kickWatchdog();

    void triggerUserAbort();
    void triggerFault(Diagnostics::ErrorCode code);

    bool isTripped();
    void clearTrip();  // explicit recovery action only
}