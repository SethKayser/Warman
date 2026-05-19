// ============================================================
//  Robot Arm Control — Arduino Mega 2560 + Motor Shield
// ============================================================

// Instructions Given:
//1. Program starts once the button (pin 22) is pressed. This will also toggle on the LED on pin 12
//2. The servo will rotate 110 degrees one way.
//3. The V stepper motor will rotate a certain amount
//4. The U stepper will rotate a certain amount
//5. The V stepper will rotate a little bit the other direction, and (at the same time) the servo will rotate the same direction as before a small amount - all while the U stepper keeps rotating the same way
//6. The U stepper will reverse a little bit.
//7. The V stepper will rotate in its other direction as the servo slowly reverses its direction as well.
//8. The Z stepper will rotate a certain amount.
//9. The DC motor will drive forward a certain amount of rotations using the encoder
//10. The U stepper will rotate its original direction
//11. Have a section here where small movements can be made in the Z stepper, V stepper and servo. Servo will need to rotate its opposite direction about 60 degrees or so
//12. The U stepper will now rotate its opposite direction as the DC motor reverses as well a certain amount of rotations tracked by its encoder
//13. The V stepper will now rotate opposite to its original direction a certain amount.
//14. After this final task is done, the LED should switch off

#include "AccelStepper.h"
#include <SpeedEncoder.h>   // wheel encoder driver
#include <TB6612.h>         // motor driver
#include <Servo.h>
#include <PID_v1.h>
#include <Encoder.h>

// ------------------------------------------------------------
//  Drive motor (DC + encoder)
// ------------------------------------------------------------
Encoder myEnc(67, 66);
TB6612 MotorDriver = TB6612();
#define M_D  31    // Drive motor direction pin
#define M_S  44    // Drive motor PWM (go/stop)

// Drive speed (0–255 PWM) and encoder targets
// TUNE: adjust PWM level and encoder counts to match your robot
#define DRIVE_SPEED      180
#define DRIVE_FWD_COUNTS 1000   // encoder counts for step 9 (forward)
#define DRIVE_REV_COUNTS 900    // encoder counts for step 12 (reverse)

// ------------------------------------------------------------
//  Lazy-susan (Z) stepper
// ------------------------------------------------------------
#define Z_ENABLE_PIN  34
#define Z_STEP_PIN    35
#define Z_DIR_PIN     36
AccelStepper stepperZ(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);

// TUNE: steps for each Z move
#define Z_STEP_8    200    // step 8  — main rotation
#define Z_STEP_11a   30    // step 11 — small trim move

// ------------------------------------------------------------
//  Pitch (V) stepper
// ------------------------------------------------------------
#define V_ENABLE_PIN  42
#define V_STEP_PIN    43
#define V_DIR_PIN     57
AccelStepper stepperV(AccelStepper::DRIVER, V_STEP_PIN, V_DIR_PIN);

// TUNE: steps for each V move
#define V_STEP_3     400    // step 3  — main pitch up
#define V_STEP_5     -80    // step 5  — small reverse (simultaneous with servo/U)
#define V_STEP_7     300    // step 7  — pitch back (opposite original)
#define V_STEP_11b    50    // step 11 — small trim move
#define V_STEP_13    350    // step 13 — final opposite rotation

// ------------------------------------------------------------
//  Arm extender (U) stepper
// ------------------------------------------------------------
#define U_ENABLE_PIN  16
#define U_STEP_PIN    17
#define U_DIR_PIN     37
AccelStepper stepperU(AccelStepper::DRIVER, U_STEP_PIN, U_DIR_PIN);

// TUNE: steps for each U move
#define U_STEP_4     600    // step 4  — extend
#define U_STEP_6    -100    // step 6  — small reverse
#define U_STEP_10    600    // step 10 — re-extend
#define U_STEP_12   -500    // step 12 — retract (same time as DC reverse)

// ------------------------------------------------------------
//  Stepper speeds / accelerations
// ------------------------------------------------------------
// TUNE: adjust to taste
#define STEPPER_MAX_SPEED  800.0
#define STEPPER_ACCEL      400.0

// ------------------------------------------------------------
//  Scoop servo
// ------------------------------------------------------------
Servo myServo;
#define SERVO_PIN         5
#define SERVO_HOME        0     // resting angle
#define SERVO_STEP_2    110     // step 2  — initial 110-degree rotation
#define SERVO_STEP_5     20     // step 5  — small further rotation (same direction as step 2)
#define SERVO_STEP_7_END  0     // step 7  — slowly reverse back toward home
#define SERVO_STEP_11   -60     // step 11 — trim: rotate ~60 deg opposite to step-2 direction
//  (Negative is relative — actual angle calculated at runtime.)

// Servo sweep speed: delay (ms) between each 1-degree step
#define SERVO_SLOW_DELAY  15    // slow sweep
#define SERVO_FAST_DELAY   5    // normal sweep

// ------------------------------------------------------------
//  Button & LED
// ------------------------------------------------------------
const int buttonPin = 22;
const int ledPin    = 12;

// ============================================================
//  Helper functions
// ============================================================

// Block until a stepper reaches its target
void runToTarget(AccelStepper &stepper) {
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
}

// Move a stepper a relative number of steps and wait
void moveStepperRelative(AccelStepper &stepper, long steps) {
  stepper.move(steps);
  runToTarget(stepper);
}

// Sweep servo smoothly to a target angle
void sweepServo(int fromAngle, int toAngle, int delayMs) {
  if (fromAngle < toAngle) {
    for (int pos = fromAngle; pos <= toAngle; pos++) {
      myServo.write(pos);
      delay(delayMs);
    }
  } else {
    for (int pos = fromAngle; pos >= toAngle; pos--) {
      myServo.write(pos);
      delay(delayMs);
    }
  }
}

// Drive DC motor forward and wait for encoder count
void driveForward(int pwmSpeed, long encoderCounts) {
  long startCount = myEnc.read();
  digitalWrite(M_D, HIGH);
  analogWrite(M_S, pwmSpeed);

  while (abs(myEnc.read() - startCount) < encoderCounts) {
    // wait
  }

  analogWrite(M_S, 0);
}

void driveReverse(int pwmSpeed, long encoderCounts) {
  long startCount = myEnc.read();
  digitalWrite(M_D, LOW);
  analogWrite(M_S, pwmSpeed);

  while (abs(myEnc.read() - startCount) < encoderCounts) {
    // wait
  }

  analogWrite(M_S, 0);
}

// ============================================================
//  setup()
// ============================================================
void setup() {
  Serial.begin(115200);

  // --- Button & LED ---
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // --- Servo ---
  myServo.attach(SERVO_PIN);
  myServo.write(SERVO_HOME);

  // --- Stepper enable pins (active LOW) ---
  pinMode(Z_ENABLE_PIN, OUTPUT); digitalWrite(Z_ENABLE_PIN, LOW);
  pinMode(V_ENABLE_PIN, OUTPUT); digitalWrite(V_ENABLE_PIN, LOW);
  pinMode(U_ENABLE_PIN, OUTPUT); digitalWrite(U_ENABLE_PIN, LOW);

  // --- Stepper config ---
  stepperZ.setMaxSpeed(STEPPER_MAX_SPEED);
  stepperZ.setAcceleration(STEPPER_ACCEL);

  stepperV.setMaxSpeed(STEPPER_MAX_SPEED);
  stepperV.setAcceleration(STEPPER_ACCEL);

  stepperU.setMaxSpeed(STEPPER_MAX_SPEED);
  stepperU.setAcceleration(STEPPER_ACCEL);

  // --- DC motor pins ---
  pinMode(M_D, OUTPUT);
  pinMode(M_S, OUTPUT);
  analogWrite(M_S, 0);  // ensure motor is stopped

  Serial.println("Ready — waiting for button press.");
}

// ============================================================
//  loop()
// ============================================================
void loop() {
  // ---- STEP 1: Wait for button press ----
  // Wait for button press (active LOW with INPUT_PULLUP)
  while (digitalRead(buttonPin) == HIGH) {
    // do nothing — just wait
  }

  delay(50); // debounce

  // Wait for release before starting, so the press-and-release
  // is fully complete before any motion begins
  while (digitalRead(buttonPin) == LOW) {
    delay(10);
  }

  digitalWrite(ledPin, HIGH);
  runSequence();
  digitalWrite(ledPin, LOW);

  // Loop now does nothing — robot is done.
  // Power cycle or reset to run again.
  while (true) { }
}


// ============================================================
//  Main robot sequence
// ============================================================
void runSequence() {

  int servoAngle = SERVO_HOME;  // track current servo position

  // ----------------------------------------------------------
  //  STEP 2 — Servo rotates 110 degrees
  // ----------------------------------------------------------
  Serial.println("Step 2: Servo → 110°");
  sweepServo(servoAngle, servoAngle + SERVO_STEP_2, SERVO_FAST_DELAY);
  servoAngle += SERVO_STEP_2;   // now at 110

  // ----------------------------------------------------------
  //  STEP 3 — V stepper rotates a certain amount
  // ----------------------------------------------------------
  Serial.println("Step 3: V stepper forward");
  moveStepperRelative(stepperV, V_STEP_3);

  // ----------------------------------------------------------
  //  STEP 4 — U stepper rotates a certain amount
  // ----------------------------------------------------------
  Serial.println("Step 4: U stepper extend");
  moveStepperRelative(stepperU, U_STEP_4);

  // ----------------------------------------------------------
  //  STEP 5 — V reverses a little, servo continues same
  //           direction a small amount, U keeps rotating
  //  These three run simultaneously.
  // ----------------------------------------------------------
  Serial.println("Step 5: V small reverse + servo nudge + U continues (simultaneous)");

  int servoTarget5 = servoAngle + SERVO_STEP_5;  // servo nudge same direction

  // Queue all three moves
  stepperV.move(V_STEP_5);      // small reverse (negative)
  stepperU.move(U_STEP_4);      // U continues same direction

  // Interleave all three while moving
  int servoPos = servoAngle;
  bool servoDone = false;

  while (stepperV.distanceToGo() != 0 ||
         stepperU.distanceToGo() != 0 ||
         !servoDone) {

    stepperV.run();
    stepperU.run();

    // Advance servo one step at a time toward target
    if (!servoDone) {
      if (servoPos < servoTarget5) {
        servoPos++;
        myServo.write(servoPos);
        delay(SERVO_SLOW_DELAY);
      } else {
        servoDone = true;
      }
    }
  }
  servoAngle = servoPos;

  // ----------------------------------------------------------
  //  STEP 6 — U stepper reverses a little bit
  // ----------------------------------------------------------
  Serial.println("Step 6: U small reverse");
  moveStepperRelative(stepperU, U_STEP_6);

  // ----------------------------------------------------------
  //  STEP 7 — V rotates its other direction; servo slowly
  //           reverses direction simultaneously
  // ----------------------------------------------------------
  Serial.println("Step 7: V forward + servo slow reverse (simultaneous)");

  stepperV.move(V_STEP_7);

  int servoTarget7 = SERVO_STEP_7_END;
  servoPos = servoAngle;
  servoDone = false;

  while (stepperV.distanceToGo() != 0 || !servoDone) {
    stepperV.run();

    if (!servoDone) {
      if (servoPos > servoTarget7) {
        servoPos--;
        myServo.write(servoPos);
        delay(SERVO_SLOW_DELAY);
      } else {
        servoDone = true;
      }
    }
  }
  servoAngle = servoPos;

  // ----------------------------------------------------------
  //  STEP 8 — Z (lazy susan) stepper rotates
  // ----------------------------------------------------------
  Serial.println("Step 8: Z stepper rotate");
  moveStepperRelative(stepperZ, Z_STEP_8);

  // ----------------------------------------------------------
  //  STEP 9 — DC motor drives forward a set encoder distance
  // ----------------------------------------------------------
  Serial.println("Step 9: DC motor forward");
  driveForward(DRIVE_SPEED, DRIVE_FWD_COUNTS);

  // ----------------------------------------------------------
  //  STEP 10 — U stepper rotates its original direction
  // ----------------------------------------------------------
  Serial.println("Step 10: U stepper extend again");
  moveStepperRelative(stepperU, U_STEP_10);

  // ----------------------------------------------------------
  //  STEP 11 — Small trim moves: Z, V, and servo
  //            Servo rotates ~60° in the OPPOSITE direction
  //            to step 2 (i.e., it goes back toward home)
  // ----------------------------------------------------------
  Serial.println("Step 11: Trim — Z nudge, V nudge, servo reverse ~60°");

  int servoTarget11 = servoAngle + SERVO_STEP_11; // SERVO_STEP_11 is negative (-60)
  if (servoTarget11 < 0) servoTarget11 = 0;        // clamp to valid range

  stepperZ.move(Z_STEP_11a);
  stepperV.move(V_STEP_11b);

  servoPos = servoAngle;
  servoDone = false;

  while (stepperZ.distanceToGo() != 0 ||
         stepperV.distanceToGo() != 0 ||
         !servoDone) {

    stepperZ.run();
    stepperV.run();

    if (!servoDone) {
      if (servoPos > servoTarget11) {
        servoPos--;
        myServo.write(servoPos);
        delay(SERVO_SLOW_DELAY);
      } else {
        servoDone = true;
      }
    }
  }
  servoAngle = servoPos;

  // ----------------------------------------------------------
  //  STEP 12 — U stepper reverses; DC motor also reverses
  //            simultaneously, tracked by encoder
  // ----------------------------------------------------------
  Serial.println("Step 12: U reverse + DC motor reverse (simultaneous)");

  // Step 12
  long encStart = myEnc.read();
  digitalWrite(M_D, LOW);
  analogWrite(M_S, DRIVE_SPEED);

  stepperU.move(U_STEP_12);

  while (stepperU.distanceToGo() != 0 ||
        abs(myEnc.read() - encStart) < DRIVE_REV_COUNTS) {
    stepperU.run();
  }

  analogWrite(M_S, 0);  // stop DC motor

  // ----------------------------------------------------------
  //  STEP 13 — V stepper rotates opposite to its original
  //            direction a certain amount
  // ----------------------------------------------------------
  Serial.println("Step 13: V stepper opposite direction");
  moveStepperRelative(stepperV, -V_STEP_13);

  // ----------------------------------------------------------
  //  STEP 14 — LED is switched off (handled in loop())
  // ----------------------------------------------------------
  Serial.println("Step 14: Sequence done — LED off.");
  // LED is turned off by the caller (loop) after this returns.
}
