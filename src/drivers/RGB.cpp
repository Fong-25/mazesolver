#include "RGB.h"

#include <Adafruit_NeoPixel.h>

#include "../config/BoardConfig.h"

namespace {
    Adafruit_NeoPixel strip(Board::RGB_LED_COUNT, Board::PIN_RGB,
                            NEO_GRB + NEO_KHZ800);

    uint8_t lastR = 0, lastG = 0, lastB = 0;
    bool initialized = false;
}

namespace RGB {
    void begin() {
        strip.begin();
        strip.show();
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

        for (uint16_t i = 0; i < strip.numPixels(); i++) {
            strip.setPixelColor(i, strip.Color(r, g, b));
        }
        strip.show();

        lastR = r;
        lastG = g;
        lastB = b;
    }

    void off() { setColor(0, 0, 0); }
}