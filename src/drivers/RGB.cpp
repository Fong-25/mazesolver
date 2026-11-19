#include "RGB.h"

#include <avr/io.h>
#include <util/atomic.h>

#include "../config/BoardConfig.h"
#include "../config/UserConfig.h"

// Minimal WS2812/SK6812 (GRB, 800 kHz) bit-bang driver, replacing
// Adafruit_NeoPixel. Flash optimization: the library plus the malloc()/free()
// it drags in cost ~1.8 KB, which is a lot for one LED. The asm below is
// hard-wired to this board; the static_asserts fail the build if either
// assumption changes instead of silently producing wrong timing.
static_assert(F_CPU == 16000000UL,
              "RGB bit-bang timing assumes a 16 MHz clock");
static_assert(Board::PIN_RGB == 11,
              "RGB bit-bang assumes the LED is on D11 = PORTB bit 3");

namespace {
    uint8_t lastR = 0, lastG = 0, lastB = 0;
    bool initialized = false;
    uint32_t lastShowUs = 0;

    // Same scaling as Adafruit_NeoPixel::setBrightness(): (c * (b + 1)) >> 8,
    // and no scaling at all at 255.
    inline uint8_t scale(uint8_t c) {
        return UserConfig::RGB_BRIGHTNESS == 255
                   ? c
                   : (uint8_t)(((uint16_t)c *
                                (UserConfig::RGB_BRIGHTNESS + 1)) >>
                               8);
    }

    // One byte, MSB first, 16 MHz. Cycle budget per bit (sbi/cbi write the pin
    // on their 2nd cycle):
    //   0-bit: high 5 cycles (312 ns), period 21 cycles (1.31 us)
    //   1-bit: high 12 cycles (750 ns), period 20 cycles (1.25 us)
    // Both are inside the WS2812B/SK6812 tolerances. Interrupts must be off.
    inline void sendByte(uint8_t v) {
        uint8_t n = 8;
        asm volatile(
            "1:                      \n\t"
            "sbi  %[port], %[bit]    \n\t"  // pin high
            "nop                     \n\t"
            "nop                     \n\t"
            "sbrs %[v], 7            \n\t"  // 1-bit: skip the early clear
            "cbi  %[port], %[bit]    \n\t"  // 0-bit: pin low (short pulse)
            "lsl  %[v]               \n\t"
            "nop \n\t nop \n\t nop \n\t nop \n\t nop \n\t"
            "cbi  %[port], %[bit]    \n\t"  // 1-bit: pin low (long pulse)
            "nop \n\t nop \n\t nop   \n\t"
            "dec  %[n]               \n\t"
            "brne 1b                 \n\t"
            : [v] "+r"(v), [n] "+r"(n)
            : [port] "I"(_SFR_IO_ADDR(PORTB)), [bit] "I"(PB3));
    }

    void show(uint8_t r, uint8_t g, uint8_t b) {
        // WS2812 latches after >50 us low, SK6812 after >80 us. Back-to-back
        // shows would merge into one frame, so wait out the latch first.
        while ((uint32_t)(micros() - lastShowUs) < 80) {
        }
        r = scale(r);
        g = scale(g);
        b = scale(b);
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            for (uint8_t i = 0; i < Board::RGB_LED_COUNT; i++) {
                sendByte(g);
                sendByte(r);
                sendByte(b);
            }
        }
        lastShowUs = micros();
    }
}

namespace RGB {
    void begin() {
        DDRB |= _BV(PB3);
        PORTB &= ~_BV(PB3);
        show(0, 0, 0);
        lastR = lastG = lastB = 0;
        initialized = true;
    }

    void setColor(uint8_t r, uint8_t g, uint8_t b) {
        // Skip the interrupt-disabling show() call entirely if nothing changed.
        // This is the difference between "a blackout every mode transition"
        // and "a blackout every single loop tick."
        if (initialized && r == lastR && g == lastG && b == lastB) {
            return;
        }

        show(r, g, b);

        lastR = r;
        lastG = g;
        lastB = b;
    }

    void off() { setColor(0, 0, 0); }
}