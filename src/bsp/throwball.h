#pragma once
#include <Arduino.h>
#include <s3servo.h>

class Throwball {
public:
    Throwball(uint8_t servoPin, uint8_t motorEnable);
    void init();
    void execute(); // Runs the whole routine once (blocking)

private:
    uint8_t servoPin;
    uint8_t motorEnable;
    s3servo servo;
};