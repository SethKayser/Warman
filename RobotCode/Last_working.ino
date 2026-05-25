// ============================================================
//  Robot Arm Control — Arduino Mega 2560 + Motor Shield
//  v6: V and Z steppers use AccelStepper. Extension motor
//      uses raw pulse/delay drive with software hard-stop
//      tracking — simpler and more direct for a pure
//      extend/retract axis. Every step is fully sequential
//      — only one motor/servo runs at a time.
//      Steps 4-6 revised: servo goes limp during extension,
//      then reattaches and nudges +10 deg.
//
//  Extension axis must be at its physical minimum before
//  power-on — that position is treated as zero.
// ============================================================

// Sequence (triggered by a single tap of D22):
//  1.  D22 tapped → LED on.
//  2.  Servo sweeps to 100 deg.
//  3.  V stepper pitches forward.
//  4a. Servo goes limp (detach).
//  4b. Extension motor extends.
//  5.  Servo reattaches and nudges +10 deg from current position.
//  5a. V stepper small reverse.
//  5b. Extension continues out.
//  6.  Extension small retract.
//  7a. V stepper pitches back.
//  7b. Servo slowly reverses.
//  8.  Z (lazy susan) stepper rotates.
//  9.  DC motor drives forward (encoder tracked).
// 10.  Extension re-extends.
// 11a. Z stepper nudge.
// 11b. V stepper nudge.
// 11c. Servo ~60° reverse trim.
// 12a. Extension retracts.
// 12b. DC motor reverses (encoder tracked).
// 13.  V stepper opposite direction.
// 14.  LED off.

#include <AccelStepper.h>
#include <Servo.h>
#include <Encoder.h>

// ============================================================
//  Extension motor — raw pulse/delay drive
// ============================================================
#define EXTENSION_ENABLE_PIN 16
#define EXTENSION_STEP_PIN   17
#define EXTENSION_DIR_PIN    37

// Swap LOW/HIGH here if the motor runs the wrong way.
const int EXTENSION_OUT_DIR = LOW;
const int EXTENSION_IN_DIR  = HIGH;

// Maximum travel from physical zero (steps). Never commanded past this.
// TUNE: measure steps to your desired maximum extension.
const long EXTENSION_HARD_STOP_POS = 5000;

// Step delays (µs) — larger = slower.
// TUNE: match to your driver's microstepping and motor torque.
int EXTENSION_OUT_STEP_DELAY_US = 1000;
int EXTENSION_IN_STEP_DELAY_US  = 1000;

// Step counter — starts at 0 (physical minimum) at power-on.
long extensionStepPosition = 0;

// Sequence move sizes (steps). Positive = extend, negative = retract.
// TUNE: adjust to match your robot's geometry.
#define EXT_STEP_4          5000   // step 4  — main extension
#define EXT_STEP_5C          800   // step 5c — additional extend
#define EXT_STEP_6          -100   // step 6  — small retract
#define EXT_STEP_10          600   // step 10 — re-extend
#define EXT_STEP_12A        -500   // step 12a — retract

// ============================================================
//  V stepper (NEMA 23) — AccelStepper (DRIVER mode)
// ============================================================
#define V_ENABLE_PIN 42
#define V_STEP_PIN   43
#define V_DIR_PIN    57

AccelStepper stepperV(AccelStepper::DRIVER, V_STEP_PIN, V_DIR_PIN);

// If the axis runs backwards, flip this to true.
#define V_INVERT_DIR false

// TUNE: the NEMA 23 needs a gentle acceleration to avoid
// brownout; start low and increase if motion is too slow.
#define V_MAX_SPEED  150.0
#define V_ACCEL       60.0

// Sequence move sizes (steps). Positive = AccelStepper forward.
// TUNE: adjust to match your arm's geometry.
#define V_STEPS_3    -200   // step 3   — main pitch
#define V_STEPS_5A    -40   // step 5a  — small reverse
#define V_STEPS_7A    250   // step 7a  — pitch back
#define V_STEPS_11B    20   // step 11b — small trim
#define V_STEPS_13     50   // step 13  — final opposite rotation

// ============================================================
//  Z (lazy susan) stepper — AccelStepper (DRIVER mode)
// ============================================================
#define Z_ENABLE_PIN 34
#define Z_STEP_PIN   35
#define Z_DIR_PIN    36

AccelStepper stepperZ(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);

#define Z_MAX_SPEED  500.0
#define Z_ACCEL      120.0

#define Z_STEP_8    -570   // step 8   — main rotation
#define Z_STEP_11A  -100   // step 11a — small trim

// ============================================================
//  Scoop servo
// ============================================================
Servo myServo;
#define SERVO_PIN        5
#define SERVO_HOME       0
#define SERVO_STEP_2   100   // step 2   — initial swing (absolute target)
#define SERVO_STEP_7B   70   // step 7b  — slow reverse (absolute target)
#define SERVO_STEP_11C -60   // step 11c — trim (relative, negative = reverse)
#define SERVO_STEP_11D 90   

#define SERVO_SLOW_DELAY_MS 15   // ms per degree — slow sweeps
#define SERVO_FAST_DELAY_MS  5   // ms per degree — normal sweeps

// ============================================================
//  DC drive motor + encoder
// ============================================================
Encoder myEnc(67, 66);
#define M_D 31   // direction pin
#define M_S 44   // PWM speed pin

#define DRIVE_SPEED       180
#define DRIVE_FWD_COUNTS 200   // encoder counts — step 9
#define DRIVE_REV_COUNTS 200   // encoder counts — step 12b

// ============================================================
//  Button & LED
// ============================================================
const int buttonPin = 22;
const int ledPin    = 12;

// ============================================================
//  Servo helpers
// ============================================================

// servoAngle tracks the current position across the sequence.
int servoAngle = SERVO_HOME;

// Sweep servo from its current position to an absolute target angle.
void sweepServoTo(int targetAngle, int delayMs) {
  if (servoAngle == targetAngle) return;
  int step = (targetAngle > servoAngle) ? 1 : -1;
  while (servoAngle != targetAngle) {
    servoAngle += step;
    myServo.write(servoAngle);
    delay(delayMs);
  }
}

// Sweep servo by a relative amount from its current position.
void sweepServoBy(int delta, int delayMs) {
  int target = servoAngle + delta;
  if (target < 0)   target = 0;
  if (target > 180) target = 180;
  sweepServoTo(target, delayMs);
}

// ============================================================
//  Stepper helpers
// ============================================================

// Block until a stepper finishes its move.
void runToStop(AccelStepper &stepper) {
  while (stepper.distanceToGo() != 0) stepper.run();
}

// Send one step pulse in the given direction (1 = out, -1 = in).
// Enforces the hard stop; returns false if the limit was hit.
bool stepExtension(int direction) {
  if (direction > 0 && extensionStepPosition >= EXTENSION_HARD_STOP_POS) {
    Serial.print("EXTENSION HARD STOP @ ");
    Serial.println(extensionStepPosition);
    return false;
  }
  if (direction < 0 && extensionStepPosition <= 0) {
    extensionStepPosition = 0;
    return false;
  }

  int delayUs = (direction > 0) ? EXTENSION_OUT_STEP_DELAY_US
                                 : EXTENSION_IN_STEP_DELAY_US;
  digitalWrite(EXTENSION_STEP_PIN, HIGH);
  delayMicroseconds(8);
  digitalWrite(EXTENSION_STEP_PIN, LOW);
  delayMicroseconds(delayUs);

  extensionStepPosition += direction;
  return true;
}

// Blocking move by a fixed number of steps, enforcing hard stop.
void moveExtension(long steps) {
  if (steps == 0) return;
  int dir = (steps > 0) ? 1 : -1;
  digitalWrite(EXTENSION_DIR_PIN, (dir > 0) ? EXTENSION_OUT_DIR : EXTENSION_IN_DIR);
  long remaining = abs(steps);
  while (remaining > 0) {
    if (!stepExtension(dir)) break;
    remaining--;
  }
}

// Move V stepper by steps.
void moveV(long steps) {
  if (steps == 0) return;
  stepperV.move(steps);
  runToStop(stepperV);
}

// Move Z stepper by steps.
void moveZ(long steps) {
  if (steps == 0) return;
  stepperZ.move(steps);
  runToStop(stepperZ);
}

// ============================================================
//  DC motor helpers
// ============================================================

void driveForward(int pwmSpeed, long encoderCounts) {
  long start = myEnc.read();
  digitalWrite(M_D, HIGH);
  analogWrite(M_S, pwmSpeed);
  while (abs(myEnc.read() - start) < encoderCounts) {}
  analogWrite(M_S, 0);
}

void driveReverse(int pwmSpeed, long encoderCounts) {
  long start = myEnc.read();
  digitalWrite(M_D, LOW);
  analogWrite(M_S, pwmSpeed);
  while (abs(myEnc.read() - start) < encoderCounts) {}
  analogWrite(M_S, 0);
}

// ============================================================
//  setup()
// ============================================================
void setup() {
  Serial.begin(115200);

  // Button & LED
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin,    OUTPUT);
  digitalWrite(ledPin, LOW);

  // Servo — attached in runSequence() only, so it doesn't snap on power-on.

  // Z stepper
  pinMode(Z_ENABLE_PIN, OUTPUT);
  digitalWrite(Z_ENABLE_PIN, LOW);
  stepperZ.setMaxSpeed(Z_MAX_SPEED);
  stepperZ.setAcceleration(Z_ACCEL);

  // V stepper (NEMA 23)
  pinMode(V_ENABLE_PIN, OUTPUT);
  digitalWrite(V_ENABLE_PIN, LOW);
  stepperV.setMaxSpeed(V_MAX_SPEED);
  stepperV.setAcceleration(V_ACCEL);
  stepperV.setPinsInverted(V_INVERT_DIR, false, false);

  // Extension motor (raw pulse drive)
  pinMode(EXTENSION_ENABLE_PIN, OUTPUT);
  digitalWrite(EXTENSION_ENABLE_PIN, LOW);
  pinMode(EXTENSION_STEP_PIN,   OUTPUT);
  digitalWrite(EXTENSION_STEP_PIN,   LOW);
  pinMode(EXTENSION_DIR_PIN,    OUTPUT);
  extensionStepPosition = 0;   // physical minimum = zero

  // DC motor
  pinMode(M_D, OUTPUT);
  pinMode(M_S, OUTPUT);
  analogWrite(M_S, 0);

  Serial.println("Ready — ensure extension is at physical minimum, then press D22.");
}

// ============================================================
//  loop()
// ============================================================
void loop() {
  // Edge-detect on D22 — tap only, no hold required.
  if (digitalRead(buttonPin) == HIGH) return;
  delay(50);                                        // debounce
  while (digitalRead(buttonPin) == LOW) delay(10);  // wait for release

  digitalWrite(ledPin, HIGH);
  runSequence();
  digitalWrite(ledPin, LOW);

  Serial.println("Done. Reset or power-cycle to run again.");
  while (true) {}
}

// ============================================================
//  Main robot sequence — one device at a time
// ============================================================
void runSequence() {

  // Attach servo here so it doesn't snap on power-on.
  myServo.attach(SERVO_PIN);
  servoAngle = SERVO_HOME;
  myServo.write(servoAngle);
  delay(300);

  // Ensure all stepper drivers are enabled (active LOW).
  // If a motor still doesn't move, try flipping its enable pin HIGH —
  // some drivers use active-HIGH enable instead.
  digitalWrite(Z_ENABLE_PIN,         LOW);
  digitalWrite(V_ENABLE_PIN,         LOW);
  digitalWrite(EXTENSION_ENABLE_PIN, LOW);
  delay(200);  // let drivers settle before first pulse

  // --------------------------------------------------------
  //  STEP 2 — Servo sweeps to 100 deg
  // --------------------------------------------------------
  Serial.println("Step 2: Servo -> 100 deg");
  sweepServoTo(SERVO_STEP_2, SERVO_FAST_DELAY_MS);
  delay(100);

  // --------------------------------------------------------
  //  STEP 3 — V stepper pitches forward
  // --------------------------------------------------------
  Serial.println("Step 3: V stepper forward");
  moveV(V_STEPS_3);
  delay(100);

  // --------------------------------------------------------
  //  STEP 4a — Servo goes limp
  // --------------------------------------------------------
  Serial.println("Step 4a: Servo detach (limp)");
  myServo.detach();
  delay(100);

  // --------------------------------------------------------
  //  STEP 4b — Extension motor extends (main move)
  // --------------------------------------------------------
  Serial.println("Step 4b: Extension out (main)");
  moveExtension(EXT_STEP_4);
  delay(100);

  // --------------------------------------------------------
  //  STEP 5 — Servo reattaches and nudges +10 deg
  // --------------------------------------------------------
  Serial.println("Step 5: Servo reattach + nudge +10 deg");
  myServo.attach(SERVO_PIN);
  delay(100);                        // let driver settle before commanding
  myServo.write(servoAngle);         // lock back to last known position first
  delay(200);                        // give servo time to reach/hold that angle
  sweepServoBy(10, SERVO_SLOW_DELAY_MS);
  delay(100);

  // --------------------------------------------------------
  //  STEP 5a — V stepper small reverse
  // --------------------------------------------------------
  Serial.println("Step 5a: V stepper small reverse");
  moveV(V_STEPS_5A);
  delay(100);

  // --------------------------------------------------------
  //  STEP 5b — Extension continues out
  // --------------------------------------------------------
  Serial.println("Step 5b: Extension continues out");
  moveExtension(EXT_STEP_5C);
  delay(100);

  // --------------------------------------------------------
  //  STEP 6 — Extension small retract
  // --------------------------------------------------------
  Serial.println("Step 6: Extension small retract");
  moveExtension(EXT_STEP_6);
  delay(100);

  // --------------------------------------------------------
  //  STEP 7a — V stepper pitches back
  // --------------------------------------------------------
  Serial.println("Step 7a: V stepper pitch back");
  moveV(V_STEPS_7A);
  delay(100);

  // --------------------------------------------------------
  //  STEP 7b — Servo slowly reverses to target
  // --------------------------------------------------------
  Serial.println("Step 7b: Servo slow reverse");
  sweepServoTo(SERVO_STEP_7B, SERVO_SLOW_DELAY_MS);
  delay(100);

  // --------------------------------------------------------
  //  STEP 8 — Z (lazy susan) rotates
  // --------------------------------------------------------
  Serial.println("Step 8: Z stepper rotate");
  moveZ(Z_STEP_8);
  delay(100);

  // --------------------------------------------------------
  //  STEP 9 — DC motor drives forward (encoder tracked)
  // --------------------------------------------------------
  Serial.println("Step 9: DC motor forward");
  driveForward(DRIVE_SPEED, DRIVE_FWD_COUNTS);
  delay(100);

  // --------------------------------------------------------
  //  STEP 10 — Extension re-extends
  // --------------------------------------------------------
  Serial.println("Step 10: Extension re-extend");
  moveExtension(EXT_STEP_10);
  delay(100);

  // --------------------------------------------------------
  //  STEP 11a — Z stepper nudge trim
  // --------------------------------------------------------
  Serial.println("Step 11a: Z stepper trim nudge");
  moveZ(Z_STEP_11A);
  delay(100);

  // --------------------------------------------------------
  //  STEP 11b — V stepper nudge trim
  // --------------------------------------------------------
  Serial.println("Step 11b: V stepper trim nudge");
  moveV(V_STEPS_11B);
  delay(100);

  // --------------------------------------------------------
  //  STEP 11c — Servo ~60° reverse trim
  // --------------------------------------------------------
  Serial.println("Step 11c: Servo trim reverse ~60 deg");
  sweepServoBy(SERVO_STEP_11C, SERVO_SLOW_DELAY_MS);
  delay(2500);  // hold position for ~2-3 seconds

  // --------------------------------------------------------
  //  STEP 11d — Servo returns to 90 deg
  // --------------------------------------------------------
  Serial.println("Step 11d: Servo return to 90 deg");
  sweepServoTo(SERVO_STEP_11D, SERVO_SLOW_DELAY_MS);
  delay(100);

  // --------------------------------------------------------
  //  STEP 12a — Extension retracts
  // --------------------------------------------------------
  Serial.println("Step 12a: Extension retract");
  moveExtension(EXT_STEP_12A);
  delay(100);

  // --------------------------------------------------------
  //  STEP 12b — DC motor reverses (encoder tracked)
  // --------------------------------------------------------
  Serial.println("Step 12b: DC motor reverse");
  driveReverse(DRIVE_SPEED, DRIVE_REV_COUNTS);
  delay(100);

  // --------------------------------------------------------
  //  STEP 13 — V stepper rotates opposite to step 3
  // --------------------------------------------------------
  Serial.println("Step 13: V stepper opposite direction");
  moveV(V_STEPS_13);
  delay(100);

  // --------------------------------------------------------
  //  STEP 14 — Sequence complete, LED off handled by caller
  // --------------------------------------------------------
  myServo.detach();  // release servo signal so it doesn't jitter at rest
  Serial.println("Step 14: Sequence done.");
}
