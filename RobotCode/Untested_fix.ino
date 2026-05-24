// ============================================================
//  Robot Arm Control — Arduino Mega 2560 + Motor Shield
//  v3: No limit switch. Extension axis is physically positioned
//      at its minimum before power-on — that position is treated
//      as zero. The arm extends only up to EXTENSION_HARD_STOP_POS.
// ============================================================

// Sequence (triggered by a single tap of D22):
// 1.  D22 tapped → LED on.
// 2.  Servo rotates 110 degrees.
// 3.  V stepper (NEMA 23) rotates forward.
// 4.  Extension motor extends.
// 5.  V small reverse + servo nudge forward + extension continues (interleaved).
// 6.  Extension retracts a little.
// 7.  V rotates back + servo slowly reverses (interleaved).
// 8.  Z (lazy susan) stepper rotates.
// 9.  DC motor drives forward (encoder tracked).
// 10. Extension re-extends.
// 11. Trim: Z nudge + V nudge + servo ~60° back (interleaved).
// 12. Extension retracts + DC motor reverses (interleaved, encoder tracked).
// 13. V rotates opposite to original direction.
// 14. LED switches off.

#include "AccelStepper.h"
#include <Servo.h>
#include <Encoder.h>

// ============================================================
//  Extension motor
// ============================================================
#define EXTENSION_ENABLE_PIN 16
#define EXTENSION_STEP_PIN   17
#define EXTENSION_DIR_PIN    37

// Swap LOW/HIGH here if the motor runs the wrong way
const int EXTENSION_OUT_DIR = LOW;
const int EXTENSION_IN_DIR  = HIGH;

// Maximum travel from the physical zero position (steps).
// The arm will never be commanded past this point.
// TUNE: measure how many steps reach your desired maximum extension.
const long EXTENSION_HARD_STOP_POS = 3600;

// Step delays (µs) — larger = slower.
// TUNE: match to your driver's microstepping and motor torque.
int EXTENSION_OUT_STEP_DELAY_US = 700;
int EXTENSION_IN_STEP_DELAY_US  = 700;

// Sequence move sizes (steps). Positive = extend, negative = retract.
// TUNE: adjust to match your robot's geometry.
#define EXT_STEP_4          5000    // step 4  — main extension
#define EXT_STEP_5_CONTINUE 800    // step 5  — continues during interleaved block
#define EXT_STEP_6         -100    // step 6  — small retract
#define EXT_STEP_10         600    // step 10 — re-extend
#define EXT_STEP_12        -500    // step 12 — retract while DC reverses

// Step counter — starts at 0 (physical minimum) at power-on.
long extensionStepPosition = 0;

// ============================================================
//  NEMA 23 V stepper  (pulse/delay — no AccelStepper)
// ============================================================
#define V_ENABLE_PIN 42
#define V_STEP_PIN   43
#define V_DIR_PIN    57

// Swap HIGH/LOW if the axis runs backwards
const int V_POSITIVE_DIR = HIGH;
const int V_NEGATIVE_DIR = LOW;

// TUNE: step timing for your NEMA 23 + driver combo
int V_STEP_PULSE_WIDTH_US = 20;
int V_FAST_STEP_DELAY_US  = 1000;
int V_SLOW_STEP_DELAY_US  = 5000;

// Sequence move sizes (steps). Positive = V_POSITIVE_DIR.
// TUNE: adjust to match your arm's geometry.
#define V_STEPS_3   2800    // step 3  — main pitch
#define V_STEPS_5    400    // step 5  — small reverse (interleaved)
#define V_STEPS_7   -3500    // step 7  — pitch back
#define V_STEPS_11b  -300    // step 11 — small trim
#define V_STEPS_13  -500    // step 13 — final opposite rotation

// ============================================================
//  Z (lazy-susan) stepper
// ============================================================
#define Z_ENABLE_PIN 34
#define Z_STEP_PIN   35
#define Z_DIR_PIN    36
AccelStepper stepperZ(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);

#define Z_MAX_SPEED 500.0
#define Z_ACCEL     120.0

#define Z_STEP_8   -550    // step 8  — main rotation
#define Z_STEP_11a  -100    // step 11 — small trim

// ============================================================
//  Scoop servo
// ============================================================
Servo myServo;
#define SERVO_PIN      5
#define SERVO_HOME     0
#define SERVO_STEP_2 100    // step 2  — initial swing
#define SERVO_STEP_5  60    // step 5  — small nudge (same direction)
#define SERVO_STEP_7_END 70  // step 7  — slow reverse target
#define SERVO_STEP_11 -60   // step 11 — trim (~60° back, applied at runtime)

#define SERVO_SLOW_DELAY 15  // ms per degree (slow sweep)
#define SERVO_FAST_DELAY  5  // ms per degree (normal sweep)

// ============================================================
//  DC drive motor + encoder
// ============================================================
Encoder myEnc(67, 66);
#define M_D 31    // direction pin
#define M_S 44    // PWM pin

#define DRIVE_SPEED      180
#define DRIVE_FWD_COUNTS 1000   // encoder counts — step 9
#define DRIVE_REV_COUNTS  900   // encoder counts — step 12

// ============================================================
//  Button & LED
// ============================================================
const int buttonPin = 22;
const int ledPin    = 12;

// ============================================================
//  Extension motor helpers
// ============================================================

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

// Blocking move by a fixed number of steps.
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

// ============================================================
//  NEMA 23 V stepper helpers
// ============================================================

// Number of steps over which to ramp from V_SLOW_STEP_DELAY_US
// down to the target speed. Increase if brownouts persist.
// TUNE: 80 is a conservative starting point for a NEMA 23.
#define V_RAMP_STEPS 80

// Blocking move with software ramp-up and ramp-down.
// Starts slow, accelerates to targetDelayUs, then decelerates.
// This prevents the inrush current spike that causes brownout resets.
void moveV(long steps, int targetDelayUs = -1) {
  if (steps == 0) return;
  if (targetDelayUs < 0) targetDelayUs = V_FAST_STEP_DELAY_US;

  int dir = (steps > 0) ? 1 : -1;
  digitalWrite(V_DIR_PIN, (dir > 0) ? V_POSITIVE_DIR : V_NEGATIVE_DIR);
  delay(2);  // direction-setup settle — do not shorten

  long total = abs(steps);
  long ramp  = min((long)V_RAMP_STEPS, total / 2);

  for (long i = 0; i < total; i++) {
    int currentDelay;
    if (ramp > 0 && i < ramp) {
      currentDelay = map(i, 0, ramp, V_SLOW_STEP_DELAY_US, targetDelayUs);
    } else if (ramp > 0 && i >= (total - ramp)) {
      currentDelay = map(i, total - ramp, total, targetDelayUs, V_SLOW_STEP_DELAY_US);
    } else {
      currentDelay = targetDelayUs;
    }
    digitalWrite(V_STEP_PIN, HIGH);
    delayMicroseconds(V_STEP_PULSE_WIDTH_US);
    digitalWrite(V_STEP_PIN, LOW);
    delayMicroseconds(currentDelay);
  }
}

// ============================================================
//  Z stepper helpers
// ============================================================

void moveZ(long steps) {
  stepperZ.move(steps);
  while (stepperZ.distanceToGo() != 0) stepperZ.run();
}

// ============================================================
//  Servo helpers
// ============================================================

void sweepServo(int fromAngle, int toAngle, int delayMs) {
  if (fromAngle < toAngle) {
    for (int pos = fromAngle; pos <= toAngle; pos++) {
      myServo.write(pos); delay(delayMs);
    }
  } else {
    for (int pos = fromAngle; pos >= toAngle; pos--) {
      myServo.write(pos); delay(delayMs);
    }
  }
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

// ============================================================
//  setup()
// ============================================================
void setup() {
  Serial.begin(115200);

  // Button & LED
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin,    OUTPUT);
  digitalWrite(ledPin, LOW);

  // Servo — NOT attached here. Attaching in setup() causes the servo
  // to immediately snap to a position on power-on. It is attached only
  // at the moment the sequence needs it (start of runSequence).

  // Z stepper
  pinMode(Z_ENABLE_PIN, OUTPUT); digitalWrite(Z_ENABLE_PIN, LOW);
  stepperZ.setMaxSpeed(Z_MAX_SPEED);
  stepperZ.setAcceleration(Z_ACCEL);

  // V stepper (NEMA 23)
  pinMode(V_ENABLE_PIN, OUTPUT); digitalWrite(V_ENABLE_PIN, LOW);
  pinMode(V_STEP_PIN,   OUTPUT); digitalWrite(V_STEP_PIN,   LOW);
  pinMode(V_DIR_PIN,    OUTPUT); digitalWrite(V_DIR_PIN,    LOW);

  // Extension motor
  pinMode(EXTENSION_ENABLE_PIN, OUTPUT); digitalWrite(EXTENSION_ENABLE_PIN, LOW);
  pinMode(EXTENSION_STEP_PIN,   OUTPUT); digitalWrite(EXTENSION_STEP_PIN,   LOW);
  pinMode(EXTENSION_DIR_PIN,    OUTPUT);

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
  // Edge-detect on D22 — tap only, no hold required
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
//  Main robot sequence
// ============================================================
void runSequence() {

  // Attach servo here — not in setup() — so it doesn't snap to a
  // position on power-on. Give it a moment to settle before use.
  myServo.attach(SERVO_PIN);
  delay(100);
  myServo.write(SERVO_HOME);
  delay(200);

  // Confirm stepper drivers are enabled (active LOW).
  // If any motor still doesn't move, try flipping its enable pin
  // to HIGH — some shields/drivers use active-HIGH enable instead.
  digitalWrite(Z_ENABLE_PIN,         LOW);
  digitalWrite(V_ENABLE_PIN,         LOW);
  digitalWrite(EXTENSION_ENABLE_PIN, LOW);
  delay(200);  // let drivers fully settle before first step pulse

  int servoAngle = SERVO_HOME;

  // --------------------------------------------------------
  //  STEP 2 — Servo rotates 110 degrees
  // --------------------------------------------------------
  Serial.println("Step 2: Servo -> 110 deg");
  sweepServo(servoAngle, servoAngle + SERVO_STEP_2, SERVO_FAST_DELAY);
  servoAngle += SERVO_STEP_2;

  // --------------------------------------------------------
  //  STEP 3 — V stepper rotates forward
  // --------------------------------------------------------
  Serial.println("Step 3: V stepper forward");
  Serial.println("  > enabling V driver, setting direction...");
  Serial.flush();
  moveV(V_STEPS_3, V_FAST_STEP_DELAY_US);
  Serial.println("  > V stepper done");

  // --------------------------------------------------------
  //  STEP 4 — Extension motor extends
  // --------------------------------------------------------
  Serial.println("Step 4: Extension out");
  moveExtension(EXT_STEP_4);

  // --------------------------------------------------------
  //  STEP 5 — V small reverse + servo nudge + extension
  //           continues — interleaved simultaneous
  // --------------------------------------------------------
  Serial.println("Step 5: V reverse + servo nudge + extension continues (interleaved)");

  long vRemaining   = V_STEPS_5;
  long extRemaining = EXT_STEP_5_CONTINUE;
  int  servoTarget5 = servoAngle + SERVO_STEP_5;
  int  servoPos     = servoAngle;
  bool servoDone    = false;

  unsigned long lastVStep   = 0;
  unsigned long lastExtStep = 0;
  unsigned long lastServoMs = 0;

  digitalWrite(V_DIR_PIN,         V_NEGATIVE_DIR);
  digitalWrite(EXTENSION_DIR_PIN, EXTENSION_OUT_DIR);

  while (vRemaining > 0 || extRemaining > 0 || !servoDone) {
    unsigned long nowUs = micros();
    unsigned long nowMs = millis();

    if (vRemaining > 0 && (nowUs - lastVStep) >= (unsigned long)V_FAST_STEP_DELAY_US) {
      digitalWrite(V_STEP_PIN, HIGH);
      delayMicroseconds(V_STEP_PULSE_WIDTH_US);
      digitalWrite(V_STEP_PIN, LOW);
      lastVStep = nowUs;
      vRemaining--;
    }

    if (extRemaining > 0 && (nowUs - lastExtStep) >= (unsigned long)EXTENSION_OUT_STEP_DELAY_US) {
      if (stepExtension(1)) extRemaining--;
      else                   extRemaining = 0;
      lastExtStep = nowUs;
    }

    if (!servoDone && (nowMs - lastServoMs) >= (unsigned long)SERVO_SLOW_DELAY) {
      if (servoPos < servoTarget5) {
        servoPos++;
        myServo.write(servoPos);
      } else {
        servoDone = true;
      }
      lastServoMs = nowMs;
    }
  }
  servoAngle = servoPos;

  // --------------------------------------------------------
  //  STEP 6 — Extension retracts a little
  // --------------------------------------------------------
  Serial.println("Step 6: Extension small retract");
  moveExtension(EXT_STEP_6);

  // --------------------------------------------------------
  //  STEP 7 — V rotates back + servo slowly reverses
  //           interleaved simultaneous
  // --------------------------------------------------------
  Serial.println("Step 7: V forward + servo slow reverse (interleaved)");

  long vRemaining7  = abs(V_STEPS_7);
  int  servoTarget7 = SERVO_STEP_7_END;
  servoPos  = servoAngle;
  servoDone = false;

  unsigned long lastVStep7  = 0;
  lastServoMs = 0;

  digitalWrite(V_DIR_PIN, V_POSITIVE_DIR);

  while (vRemaining7 > 0 || !servoDone) {
    unsigned long nowUs = micros();
    unsigned long nowMs = millis();

    if (vRemaining7 > 0 && (nowUs - lastVStep7) >= (unsigned long)V_FAST_STEP_DELAY_US) {
      digitalWrite(V_STEP_PIN, HIGH);
      delayMicroseconds(V_STEP_PULSE_WIDTH_US);
      digitalWrite(V_STEP_PIN, LOW);
      lastVStep7 = nowUs;
      vRemaining7--;
    }

    if (!servoDone && (nowMs - lastServoMs) >= (unsigned long)SERVO_SLOW_DELAY) {
      if (servoPos > servoTarget7) {
        servoPos--;
        myServo.write(servoPos);
      } else {
        servoDone = true;
      }
      lastServoMs = nowMs;
    }
  }
  servoAngle = servoPos;

  // --------------------------------------------------------
  //  STEP 8 — Z (lazy susan) rotates
  // --------------------------------------------------------
  Serial.println("Step 8: Z stepper rotate");
  moveZ(Z_STEP_8);

  // --------------------------------------------------------
  //  STEP 9 — DC motor drives forward
  // --------------------------------------------------------
  Serial.println("Step 9: DC motor forward");
  driveForward(DRIVE_SPEED, DRIVE_FWD_COUNTS);

  // --------------------------------------------------------
  //  STEP 10 — Extension re-extends
  // --------------------------------------------------------
  Serial.println("Step 10: Extension out again");
  moveExtension(EXT_STEP_10);

  // --------------------------------------------------------
  //  STEP 11 — Trim: Z nudge + V nudge + servo ~60° back
  //            interleaved simultaneous
  // --------------------------------------------------------
  Serial.println("Step 11: Trim - Z, V nudge, servo reverse ~60 deg");

  int servoTarget11 = servoAngle + SERVO_STEP_11;  // SERVO_STEP_11 is -60
  if (servoTarget11 < 0) servoTarget11 = 0;

  long vRemaining11 = abs(V_STEPS_11b);
  servoPos  = servoAngle;
  servoDone = false;

  stepperZ.move(Z_STEP_11a);

  unsigned long lastVStep11 = 0;
  lastServoMs = 0;
  digitalWrite(V_DIR_PIN, V_POSITIVE_DIR);

  while (stepperZ.distanceToGo() != 0 || vRemaining11 > 0 || !servoDone) {
    unsigned long nowUs = micros();
    unsigned long nowMs = millis();

    stepperZ.run();

    if (vRemaining11 > 0 && (nowUs - lastVStep11) >= (unsigned long)V_FAST_STEP_DELAY_US) {
      digitalWrite(V_STEP_PIN, HIGH);
      delayMicroseconds(V_STEP_PULSE_WIDTH_US);
      digitalWrite(V_STEP_PIN, LOW);
      lastVStep11 = nowUs;
      vRemaining11--;
    }

    if (!servoDone && (nowMs - lastServoMs) >= (unsigned long)SERVO_SLOW_DELAY) {
      if (servoPos > servoTarget11) {
        servoPos--;
        myServo.write(servoPos);
      } else {
        servoDone = true;
      }
      lastServoMs = nowMs;
    }
  }
  servoAngle = servoPos;

  // --------------------------------------------------------
  //  STEP 12 — Extension retracts + DC motor reverses
  //            interleaved simultaneous, encoder tracked
  // --------------------------------------------------------
  Serial.println("Step 12: Extension retract + DC motor reverse (interleaved)");

  long encStart    = myEnc.read();
  long extRemain12 = abs(EXT_STEP_12);

  digitalWrite(M_D, LOW);
  analogWrite(M_S, DRIVE_SPEED);
  digitalWrite(EXTENSION_DIR_PIN, EXTENSION_IN_DIR);

  unsigned long lastExtStep12 = 0;

  while (extRemain12 > 0 || abs(myEnc.read() - encStart) < DRIVE_REV_COUNTS) {
    unsigned long nowUs = micros();
    if (extRemain12 > 0 && (nowUs - lastExtStep12) >= (unsigned long)EXTENSION_IN_STEP_DELAY_US) {
      if (stepExtension(-1)) extRemain12--;
      else                    extRemain12 = 0;
      lastExtStep12 = nowUs;
    }
  }
  analogWrite(M_S, 0);

  // --------------------------------------------------------
  //  STEP 13 — V rotates opposite to original direction
  // --------------------------------------------------------
  Serial.println("Step 13: V stepper opposite direction");
  moveV(-V_STEPS_13, V_FAST_STEP_DELAY_US);

  // --------------------------------------------------------
  //  STEP 14 — LED off (handled by caller)
  // --------------------------------------------------------
  myServo.detach();  // release servo signal so it doesn't jitter at rest
  Serial.println("Step 14: Sequence done - LED off.");
}
