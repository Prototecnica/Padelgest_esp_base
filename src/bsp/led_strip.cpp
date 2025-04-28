#include <bsp/led_strip.h>
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define NUM_LEDS 36

static Adafruit_NeoPixel strip(NUM_LEDS, rgb_led_pin, NEO_GRB + NEO_KHZ800);

LedStrip::LedStrip(uint8_t pin) : pin(pin) {
    // strip.updateType(NEO_GRB + NEO_KHZ800);
    // strip.setPin(pin);
}

bool LedStrip::init() {
    strip.begin();
    strip.setBrightness(127);
    turnOff();
    return true;
}

void LedStrip::turnOn(uint8_t r, uint8_t g, uint8_t b) {
    uint32_t color_value;
    strip.setBrightness(255);
    color_value = strip.Color(r, g, b);
    strip.fill(color_value);
    strip.show();
}

void LedStrip::turnOff() {
    strip.fill(strip.Color(0, 0, 0));
    strip.show();
}

void LedStrip::effectStatic(uint8_t r, uint8_t g, uint8_t b) {
    turnOn(r, g, b);
}

void LedStrip::effectSnake(uint8_t r, uint8_t g, uint8_t b, int step, int length) {
    turnOff();
    const int snakeLength = length;
    for (int i = 0; i < snakeLength; ++i) {
        int index = (step + i) % NUM_LEDS;
        strip.setPixelColor(index, strip.Color(r, g, b));
    }
    strip.show();
}

// led_strip.cpp

void LedStrip::fillOneByOne(uint8_t r, uint8_t g, uint8_t b) {
    if (!fillEffectRunning || r != currentR || g != currentG || b != currentB) {
        // Starting fresh
        currentR = r;
        currentG = g;
        currentB = b;
        fillIndex = 0;
        fillingPhase = true;
        fillEffectRunning = true;
    }
    if (fillingPhase) {
        if (fillIndex < NUM_LEDS) {
            strip.setPixelColor(fillIndex, strip.Color(currentR, currentG, currentB));
            strip.show();
            fillIndex++;
        } else {
            // Start emptying
            fillingPhase = false;
            fillIndex = 0;
        }
    } else {
        if (fillIndex < NUM_LEDS) {
            strip.setPixelColor(fillIndex, strip.Color(0, 0, 0));
            strip.show();
            fillIndex++;
        } else {
            // Restart the fill cycle
            fillingPhase = true;
            fillIndex = 0;
        }
    }
}

void LedStrip::resetFillOneByOne() {
    fillEffectRunning = false;
    fillIndex = 0;
    fillingPhase = true;
}


