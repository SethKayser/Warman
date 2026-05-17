

#include "AccelStepper.h"
#include <SpeedEncoder.h> //wheel encoder driver
#include <TB6612.h> //motor driver
#include <Servo.h>
#include <PID_v1.h>
#include <Encoder.h>

// Drivetrain motor
SpeedEncoder myEnc(19,18);
TB6612 MotorDriver = TB6612();
#define M_D   33   //Drive motor direction
#define M_S   46   //Drive motor 6 PWM go/stop

// Lazy susan stepper
#define Z_ENABLE_PIN       34
#define Z_STEP_PIN         35
#define Z_DIR_PIN          36
#define mrz 300.0 // max range in mm
#define s360z 600 //steps per 360

// Pitch stepper
#define V_ENABLE_PIN       42
#define V_STEP_PIN         43
#define V_DIR_PIN          57
#define mrv 300.0 // max range in mm
#define s360v 600 //steps per 360

// Arm extender stepper
#define U_ENABLE_PIN       16
#define U_STEP_PIN         17
#define U_DIR_PIN          37
#define mru 300.0 // max range in mm
#define s360u 600 //steps per 360

// Scoop servo
Servo myservo;  // create servo object to control a servo
#define servo_pin 5
#define max_range_mm 300.0

// Start Button
const int buttonPin = 22;

// LED (pin 12)
const int ledPin = 12;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
