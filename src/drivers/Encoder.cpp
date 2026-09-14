#include "Encoder.h"

#include <math.h>

#include "../config/BoardConfig.h"
#include "../config/RobotConfig.h"

namespace {
    volatile int32_t leftCount = 0;
    volatile int32_t rightCount = 0;

    volatile uint8_t leftState = 0;  // 2-bit (A<<1 | B)
    volatile uint8_t rightState = 0;

    int32_t lastLeftForCheck = 0;
    int32_t lastRightForCheck = 0;
    uint32_t lastServiceMs = 0;
    bool glitchFlag = false;

    // Standard x4 quadrature decode table. Index = (oldState<<2 | newState).
    // Entries of 0 correspond to either "no change" or a Gray-code-impossible
    // jump — both are safely ignored rather than miscounted. This is spec
    // 9.2 item #1/#2 ("reject impossible quadrature transitions") built
    // directly into the decode step, not bolted on after.
    const int8_t QUAD_TABLE[16] = {0,  -1, +1, 0,  +1, 0,  0,  -1,
                                   -1, 0,  0,  +1, 0,  +1, -1, 0};

    inline void decodeLeft() {
        uint8_t a = digitalRead(Board::PIN_LEFT_ENC_A);
        uint8_t b = digitalRead(Board::PIN_LEFT_ENC_B);
        uint8_t newState = (a << 1) | b;
        int8_t delta = QUAD_TABLE[(leftState << 2) | newState];
        leftCount += RobotConfig::LEFT_ENCODER_REVERSED ? -delta : delta;
        leftState = newState;
    }

    inline void decodeRight() {
        uint8_t a = digitalRead(Board::PIN_RIGHT_ENC_A);
        uint8_t b = digitalRead(Board::PIN_RIGHT_ENC_B);
        uint8_t newState = (a << 1) | b;
        int8_t delta = QUAD_TABLE[(rightState << 2) | newState];
        rightCount += RobotConfig::RIGHT_ENCODER_REVERSED ? -delta : delta;
        rightState = newState;
    }

    void leftISR() {
        decodeLeft();  // no float, no I2C, no printing — spec 9.1 compliant
    }
}

// Right encoder shares the PCINT2 vector (PORTD, D0-D7) since D4/D7 have no
// dedicated external-interrupt lines. PCMSK2 masks it down to ONLY these
// two pins, so UART activity on D0/D1 never spuriously fires this.
ISR(PCINT2_vect) { decodeRight(); }

namespace Encoder {
    void begin() {
        pinMode(Board::PIN_LEFT_ENC_A, INPUT_PULLUP);
        pinMode(Board::PIN_LEFT_ENC_B, INPUT_PULLUP);
        pinMode(Board::PIN_RIGHT_ENC_A, INPUT_PULLUP);
        pinMode(Board::PIN_RIGHT_ENC_B, INPUT_PULLUP);

        leftState = (digitalRead(Board::PIN_LEFT_ENC_A) << 1) |
                    digitalRead(Board::PIN_LEFT_ENC_B);
        rightState = (digitalRead(Board::PIN_RIGHT_ENC_A) << 1) |
                     digitalRead(Board::PIN_RIGHT_ENC_B);

        attachInterrupt(digitalPinToInterrupt(Board::PIN_LEFT_ENC_A), leftISR,
                        CHANGE);
        attachInterrupt(digitalPinToInterrupt(Board::PIN_LEFT_ENC_B), leftISR,
                        CHANGE);

        PCMSK2 |= (1 << PCINT20) | (1 << PCINT23);  // D4, D7 only
        PCICR |= (1 << PCIE2);

        lastServiceMs = millis();
    }

    void service(uint32_t nowMs) {
        uint32_t dt = nowMs - lastServiceMs;
        if (dt == 0) return;
        lastServiceMs = nowMs;

        int32_t l = LEFT_ENCODER_COUNT();
        int32_t r = RIGHT_ENCODER_COUNT();
        int32_t leftDelta = l - lastLeftForCheck;
        int32_t rightDelta = r - lastRightForCheck;
        lastLeftForCheck = l;
        lastRightForCheck = r;

        float leftMmPerCount = (PI * RobotConfig::LEFT_WHEEL_DIAMETER_MM) /
                               RobotConfig::ENCODER_COUNTS_PER_OUTPUT_REV;
        float rightMmPerCount = (PI * RobotConfig::RIGHT_WHEEL_DIAMETER_MM) /
                                RobotConfig::ENCODER_COUNTS_PER_OUTPUT_REV;
        float dtSec = dt / 1000.0f;
        float leftSpeedMmS = (leftDelta * leftMmPerCount) / dtSec;
        float rightSpeedMmS = (rightDelta * rightMmPerCount) / dtSec;

        float limit = RobotConfig::MAX_LINEAR_SPEED_MM_S *
                      RobotConfig::ENCODER_GLITCH_SAFETY_MARGIN;
        if (fabs(leftSpeedMmS) > limit || fabs(rightSpeedMmS) > limit) {
            glitchFlag = true;
        }
    }

    int32_t LEFT_ENCODER_COUNT() {
        noInterrupts();
        int32_t c = leftCount;
        interrupts();
        return c;
    }

    int32_t RIGHT_ENCODER_COUNT() {
        noInterrupts();
        int32_t c = rightCount;
        interrupts();
        return c;
    }

    bool consumeGlitchFlag() {
        bool f = glitchFlag;
        glitchFlag = false;
        return f;
    }
}