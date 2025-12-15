#include <s3servo.h>

s3servo myServo;

void setup() {
  Serial.begin(115200);
  Serial.println("Servo Test");
  myServo.attach(41);   // PWM pin
  myServo.write(180-130); // Default to closed position
}

void loop() {
  Serial.println("Moving to 60 degrees");
  myServo.write(180-60);    // Move to 90 degrees
  delay(5000);
  Serial.println("Moving to 130 degrees");
  myServo.write(180-130);    // Move to 90 degrees
  delay(5000);
}