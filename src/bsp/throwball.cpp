#include "throwball.h"

Throwball::Throwball(uint8_t servoPin, uint8_t motorEnable)
: servoPin(servoPin), motorEnable(motorEnable) {}

void Throwball::init() {
    pinMode(motorEnable, OUTPUT);
    // ESP32Servo requires setting up PWM channels before attach
    // Optional: Servo.attach(pin, minUs, maxUs) to match your servo's range
    servo.attach(servoPin, 500, 2400);
    digitalWrite(motorEnable, HIGH); // HIGH = motor off
    servo.write(130); // Default to closed position
}

void Throwball::execute() {
    // Start motor
    Serial.println("activating");
    digitalWrite(motorEnable, LOW);  // LOW = motor on
    delay(3500); // Spin up for 3.5s
    // Open servo
    servo.write(60); // Adjust to your "open" angle
    delay(650);
    // Close servo
    servo.write(130); // Adjust to your "closed" angle
    delay(400);
    // Stop motor
    Serial.println("deactivating");
    digitalWrite(motorEnable, HIGH);
}