#include "throwball.h"

Throwball::Throwball(uint8_t servoPin, uint8_t motorEnable)
: servoPin(servoPin), motorEnable(motorEnable) {}

void Throwball::init() {
    pinMode(motorEnable, OUTPUT);
    digitalWrite(motorEnable, HIGH); // Motor off
    servo.attach(servoPin);   // PWM pin
    servo.write(180-130); // Default to closed position
}

void Throwball::execute() {
    Serial.println("activating");
    digitalWrite(motorEnable, LOW);
    delay(5000);
    // Open servo
    servo.write(180-60); // Adjust to your "open" angle
    delay(2000);
    // Close servo
    servo.write(180-130); // Adjust to your "closed" angle
    delay(3000);
    // Stop motor
    Serial.println("deactivating");
    digitalWrite(motorEnable, HIGH);
}
