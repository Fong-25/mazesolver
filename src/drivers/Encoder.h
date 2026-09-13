#pragma once
#include <Arduino.h>

namespace Encoder {
    void begin();

    // Call every loop tick. Cheap — does the noise cross-check from spec
    // section 9.2 (compare measured speed against what's physically possible).
    void service(uint32_t nowMs);

    // The only two functions anything outside this module should ever call.
    int32_t LEFT_ENCODER_COUNT();
    int32_t RIGHT_ENCODER_COUNT();

    // One-shot: true if the last service() detected a count-rate exceeding
    // physically-possible speed (EMI/PWM noise corrupting a read, most
    // likely). Consuming it clears it. Intended to feed Safety/Diagnostics'
    // ErrorCode::ENCODER_INVALID later.
    bool consumeGlitchFlag();
}