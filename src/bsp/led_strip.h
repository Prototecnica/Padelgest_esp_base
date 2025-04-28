#pragma once
#include <Arduino.h>
#include <globals.h>

class LedStrip {
    private:
        uint8_t pin;
        uint8_t currentR, currentG, currentB;
        int fillIndex;
        bool fillingPhase;
        bool fillEffectRunning;
    public:
        LedStrip(uint8_t pin);
        bool init();;
        void turnOn(uint8_t r, uint8_t g, uint8_t b);
        void turnOff();
        void effectStatic(uint8_t r, uint8_t g, uint8_t b); 
        void effectSnake(uint8_t r, uint8_t g, uint8_t b, int step, int length);
        void fillOneByOne(uint8_t r, uint8_t g, uint8_t b);
        void resetFillOneByOne();  // resets internal state
};
