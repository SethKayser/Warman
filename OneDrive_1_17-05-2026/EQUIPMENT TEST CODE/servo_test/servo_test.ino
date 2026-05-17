#include <Servo.h>

Servo myServo;

const int SERVO_PIN = 5;
const int BUTTON_PIN = 22;

void setup() {
  myServo.attach(SERVO_PIN);
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Use internal pull-up resistor
  myServo.write(0);                   // Start at 0 degrees
}

void loop() {
  if (digitalRead(BUTTON_PIN) == HIGH) {  // LOW = pressed (with pull-up)
    myServo.write(180);
    Serial.write(digitalRead(BUTTON_PIN));  // Spin to 180 degrees
  } else {
    myServo.write(0);    // Return to 0 degrees when released
  }
}