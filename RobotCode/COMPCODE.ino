#include <Servo.h>

// ============================================================
// CLICK-TO-ADVANCE SEQUENCE
// ============================================================
// Click  1: Servo to first scoop angle
// Click  2: Extension out
// Click  3: Pitch down
// Click  4: Extension out again
// Click  5: Servo to second scoop angle
// Click  6: Extension out again
// Click  7: Servo off
// Click  8: Extension out after servo off
// Click  9: Servo back on and raised
// Click 10: Pitch up while adjusting servo
// Click 11: Pitch up again, then extend to max
// Click 12: Rotate left
// Click 13: Drive forward
// Click 14: Drop rocks and stop sequence
// ============================================================

// ============================================================
// PIN DEFINITIONS
// ============================================================

// ----- Pitch stepper -----
constexpr uint8_t PITCH_ENABLE_PIN = 42;
constexpr uint8_t PITCH_STEP_PIN   = 43;
constexpr uint8_t PITCH_DIR_PIN    = 57;

// ----- Extension stepper -----
constexpr uint8_t EXTENSION_ENABLE_PIN = 34;
constexpr uint8_t EXTENSION_STEP_PIN   = 35;
constexpr uint8_t EXTENSION_DIR_PIN    = 36;

// ----- Rotate stepper -----
constexpr uint8_t ROTATE_ENABLE_PIN = 16;
constexpr uint8_t ROTATE_STEP_PIN   = 17;
constexpr uint8_t ROTATE_DIR_PIN    = 37;

// ----- Drive motor -----
constexpr uint8_t DRIVE_DIR_PIN = 33;
constexpr uint8_t DRIVE_PWM_PIN = 46;

// ----- Servo -----
constexpr uint8_t SERVO_PIN = 5;

// ----- Button -----
constexpr uint8_t BUTTON_PIN = 22;

// ----- LED -----
constexpr uint8_t LED_PIN = 12;

// ============================================================
// INPUT / OUTPUT LOGIC STATES
// ============================================================

// This assumes the button is wired with an external pulldown resistor.
// If using INPUT_PULLUP, change BUTTON_PRESSED_STATE to LOW and pinMode to INPUT_PULLUP.
constexpr uint8_t BUTTON_PRESSED_STATE = HIGH;

constexpr uint8_t DRIVER_ENABLED_STATE  = LOW;
constexpr uint8_t DRIVER_DISABLED_STATE = HIGH;

// ============================================================
// MOTOR DIRECTIONS
// ============================================================

constexpr uint8_t PITCH_DOWN_DIR    = LOW;
constexpr uint8_t PITCH_UP_DIR      = HIGH;
constexpr uint8_t EXTENSION_OUT_DIR = LOW;
constexpr uint8_t ROTATE_LEFT_DIR   = LOW;
constexpr uint8_t DRIVE_FORWARD_DIR = LOW;

// ============================================================
// STEPPER TIMING
// ============================================================

constexpr unsigned int  PULSE_WIDTH_US         = 40;
constexpr unsigned long PITCH_STEP_DELAY_US    = 5000;
constexpr unsigned long PITCH_UP_STEP_DELAY_US = 5000;
constexpr unsigned long EXTENSION_STEP_DELAY_US = 1000;
constexpr unsigned long ROTATE_STEP_DELAY_US   = 1500;

constexpr unsigned long STEPPER_DIR_SETTLE_MS    = 20;
constexpr unsigned long STEPPER_ENABLE_SETTLE_MS = 100;
constexpr unsigned long EXTENSION_ENABLE_SETTLE_MS = 50;

// ============================================================
// MOVEMENT DISTANCES
// ============================================================

// ----- Pitch movements -----
constexpr unsigned long PITCH_DOWN_STEPS       = 400;
constexpr unsigned long PITCH_UP_STEPS_ONE     = 1200;
constexpr unsigned long PITCH_UP_STEPS_TWO     = 1100;

// ----- Extension movements -----
constexpr unsigned long EXTENSION_START_STEPS           = 700;
constexpr unsigned long EXTENSION_EXTRA_STEPS           = 700;
constexpr unsigned long EXTENSION_FINAL_STEPS           = 400;
constexpr unsigned long EXTENSION_AFTER_SERVO_OFF_STEPS = 350;
constexpr unsigned long EXTENSION_TO_MAX_STEPS          = 950;

// ----- Rotate movement -----
constexpr unsigned long ROTATE_LEFT_STEPS = 2400;

// ============================================================
// SERVO SETTINGS
// ============================================================

constexpr int SERVO_MIN_ANGLE = 0;
constexpr int SERVO_MAX_ANGLE = 180;

constexpr int SERVO_START_ANGLE      = 0;
constexpr int SERVO_FIRST_ANGLE      = 30;
constexpr int SERVO_SECOND_ANGLE     = 140;
constexpr int SERVO_UP_EXTRA_DEGREES = 40;
constexpr int SERVO_DROP_ANGLE       = 20;

// Target angle used during the two pitch-up movements.
constexpr int SERVO_LEVEL_ANGLE_ONE = 100;
constexpr int SERVO_LEVEL_ANGLE_TWO = 100;

// Existing alignment offset preserved from the original sketch.
constexpr int SERVO_ALIGN_OFFSET_DEG = 15;

// Larger value = slower movement.
constexpr int SERVO_SLOW_DELAY_MS = 35;

constexpr unsigned long SERVO_ATTACH_SETTLE_MS = 300;
constexpr unsigned long SERVO_FINAL_SETTLE_MS  = 500;

// ============================================================
// DRIVE MOTOR SETTINGS
// ============================================================

constexpr uint8_t DRIVE_SPEED = 180;
constexpr unsigned long DRIVE_TIME_MS = 3600;

// ============================================================
// BUTTON SETTINGS
// ============================================================

constexpr unsigned long BUTTON_DEBOUNCE_MS = 50;
constexpr unsigned long BUTTON_RELEASE_POLL_MS = 10;

// ============================================================
// TYPES / GLOBAL STATE
// ============================================================

struct StepperMotor {
  uint8_t enablePin;
  uint8_t stepPin;
  uint8_t dirPin;
  unsigned long enableSettleMs;
};

const StepperMotor pitchStepper = {
  PITCH_ENABLE_PIN,
  PITCH_STEP_PIN,
  PITCH_DIR_PIN,
  STEPPER_ENABLE_SETTLE_MS
};

const StepperMotor extensionStepper = {
  EXTENSION_ENABLE_PIN,
  EXTENSION_STEP_PIN,
  EXTENSION_DIR_PIN,
  EXTENSION_ENABLE_SETTLE_MS
};

const StepperMotor rotateStepper = {
  ROTATE_ENABLE_PIN,
  ROTATE_STEP_PIN,
  ROTATE_DIR_PIN,
  STEPPER_ENABLE_SETTLE_MS
};

Servo scoopServo;

uint8_t sequenceStep = 0;
bool sequenceDone = false;
int currentServoAngle = SERVO_START_ANGLE;

// ============================================================
// FUNCTION DECLARATIONS
// ============================================================

void setupStepper(const StepperMotor &motor, uint8_t initialDirection);
void setupDriveMotor();

bool buttonPressed();
void waitForButtonRelease();

void runNextStep();
void printStep(uint8_t stepNumber, const char *description);
void finishSequence();

void moveStepper(const StepperMotor &motor,
                 uint8_t direction,
                 unsigned long steps,
                 unsigned long stepDelayUs,
                 bool disableAfterMove);

void movePitch(uint8_t direction, unsigned long steps, unsigned long stepDelayUs, bool disableAfterMove);
void moveExtension(unsigned long steps);
void rotateLeft(unsigned long steps);

void moveServoSlowToAndLock(int targetAngle, int baseDelayMs);
void lockServoAtCurrentAngle();
void detachServo();
void movePitchUpWithServoLevel(unsigned long steps, int servoEndAngle);

void driveForwardForTime();
void stopDriveMotor();

// ============================================================
// ARDUINO SETUP / LOOP
// ============================================================

void setup() {
  Serial.begin(9600);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  setupStepper(pitchStepper, PITCH_DOWN_DIR);
  setupStepper(extensionStepper, EXTENSION_OUT_DIR);
  setupStepper(rotateStepper, ROTATE_LEFT_DIR);
  setupDriveMotor();

  pinMode(BUTTON_PIN, INPUT);

  detachServo();

  Serial.println("Ready: one movement per button press");
}

void loop() {
  if (sequenceDone || !buttonPressed()) {
    return;
  }

  delay(BUTTON_DEBOUNCE_MS);

  if (buttonPressed()) {
    waitForButtonRelease();
    runNextStep(); // change this to runFullSequence() for all in one or to runNextStep() for one step per button press
  }
}

// ============================================================
// SETUP HELPERS
// ============================================================

void setupStepper(const StepperMotor &motor, uint8_t initialDirection) {
  pinMode(motor.enablePin, OUTPUT);
  pinMode(motor.stepPin, OUTPUT);
  pinMode(motor.dirPin, OUTPUT);

  digitalWrite(motor.stepPin, LOW);
  digitalWrite(motor.dirPin, initialDirection);
  digitalWrite(motor.enablePin, DRIVER_DISABLED_STATE);
}

void setupDriveMotor() {
  pinMode(DRIVE_DIR_PIN, OUTPUT);
  pinMode(DRIVE_PWM_PIN, OUTPUT);

  digitalWrite(DRIVE_DIR_PIN, DRIVE_FORWARD_DIR);
  stopDriveMotor();
}

// ============================================================
// BUTTON HELPERS
// ============================================================

bool buttonPressed() {
  return digitalRead(BUTTON_PIN) == BUTTON_PRESSED_STATE;
}

void waitForButtonRelease() {
  while (buttonPressed()) {
    delay(BUTTON_RELEASE_POLL_MS);
  }
}

// ============================================================
// CLICK SEQUENCE
// ============================================================

void runNextStep() {
  if (sequenceStep == 0) {
    digitalWrite(LED_PIN, HIGH);
  }
  sequenceStep++;

  switch (sequenceStep) {
    case 1:
      printStep(sequenceStep, "Servo slowly goes to first angle and locks");
      moveServoSlowToAndLock(SERVO_FIRST_ANGLE, SERVO_SLOW_DELAY_MS);
      break;

    case 2:
      printStep(sequenceStep, "Extension moves out from start position");
      moveExtension(EXTENSION_START_STEPS);
      break;

    case 3:
      printStep(sequenceStep, "Pitch lowers");
      movePitch(PITCH_DOWN_DIR, PITCH_DOWN_STEPS, PITCH_STEP_DELAY_US, true);
      break;

    case 4:
      printStep(sequenceStep, "Extension moves out again");
      moveExtension(EXTENSION_EXTRA_STEPS);
      break;

    case 5:
      printStep(sequenceStep, "Servo slowly moves to second angle and locks");
      moveServoSlowToAndLock(SERVO_SECOND_ANGLE, SERVO_SLOW_DELAY_MS);
      break;

    case 6:
      printStep(sequenceStep, "Extension moves out again");
      moveExtension(EXTENSION_FINAL_STEPS);
      break;

    case 7:
      printStep(sequenceStep, "Servo switches off");
      detachServo();
      break;

    case 8:
      printStep(sequenceStep, "Extension moves out after servo off");
      moveExtension(EXTENSION_AFTER_SERVO_OFF_STEPS);
      break;

    case 9:
      printStep(sequenceStep, "Servo turns on and raises");
      lockServoAtCurrentAngle();
      moveServoSlowToAndLock(currentServoAngle + SERVO_UP_EXTRA_DEGREES, SERVO_SLOW_DELAY_MS);
      break;

    case 10:
      printStep(sequenceStep, "Pitch up while servo levels");
      movePitchUpWithServoLevel(PITCH_UP_STEPS_ONE, SERVO_LEVEL_ANGLE_ONE);
      break;

    case 11:
      printStep(sequenceStep, "Max pitch while keeping servo level, then extend to max");
      moveExtension(EXTENSION_TO_MAX_STEPS);
      break;

    case 12:
      printStep(sequenceStep, "Rotate left");
      rotateLeft(ROTATE_LEFT_STEPS);
      break;

    case 13:
      printStep(sequenceStep, "Drive forward");
      driveForwardForTime();
      break;

    case 14:
      printStep(sequenceStep, "Drop rocks");
      moveServoSlowToAndLock(SERVO_DROP_ANGLE, SERVO_SLOW_DELAY_MS);
      finishSequence();
      break;

    default:
      finishSequence();
      break;
  }
}

void runFullSequence() {
  while (!sequenceDone) {
    runNextStep();

    // Optional small pause between sections so movements do not chain too violently
    delay(250);
  }
}

void printStep(uint8_t stepNumber, const char *description) {
  Serial.print("RUNNING CLICK ");
  Serial.println(stepNumber);
  Serial.print("Click ");
  Serial.print(stepNumber);
  Serial.print(": ");
  Serial.println(description);
}

void finishSequence() {
  sequenceDone = true;
  stopDriveMotor();

  // Keep pitch enabled so it can hold its position at the end.
  digitalWrite(PITCH_ENABLE_PIN, DRIVER_ENABLED_STATE);
  digitalWrite(LED_PIN, LOW);

  Serial.println("SEQUENCE STOPPED - PITCH MOTOR HOLDING");
}

// ============================================================
// STEPPER HELPERS
// ============================================================

void moveStepper(const StepperMotor &motor,
                 uint8_t direction,
                 unsigned long steps,
                 unsigned long stepDelayUs,
                 bool disableAfterMove) {
  digitalWrite(motor.dirPin, direction);
  delay(STEPPER_DIR_SETTLE_MS);

  digitalWrite(motor.enablePin, DRIVER_ENABLED_STATE);
  delay(motor.enableSettleMs);

  for (unsigned long stepCount = 0; stepCount < steps; stepCount++) {
    digitalWrite(motor.stepPin, HIGH);
    delayMicroseconds(PULSE_WIDTH_US);
    digitalWrite(motor.stepPin, LOW);
    delayMicroseconds(stepDelayUs);
  }

  digitalWrite(motor.stepPin, LOW);

  if (disableAfterMove) {
    digitalWrite(motor.enablePin, DRIVER_DISABLED_STATE);
  }
}

void movePitch(uint8_t direction, unsigned long steps, unsigned long stepDelayUs, bool disableAfterMove) {
  moveStepper(pitchStepper, direction, steps, stepDelayUs, disableAfterMove);
}

void moveExtension(unsigned long steps) {
  moveStepper(extensionStepper, EXTENSION_OUT_DIR, steps, EXTENSION_STEP_DELAY_US, true);
}

void rotateLeft(unsigned long steps) {
  moveStepper(rotateStepper, ROTATE_LEFT_DIR, steps, ROTATE_STEP_DELAY_US, true);
}

// ============================================================
// SERVO HELPERS
// ============================================================

void moveServoSlowToAndLock(int targetAngle, int baseDelayMs) {
  targetAngle = constrain(targetAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

  if (!scoopServo.attached()) {
    scoopServo.attach(SERVO_PIN);
    scoopServo.write(currentServoAngle);
    delay(SERVO_ATTACH_SETTLE_MS);
  }

  if (targetAngle == currentServoAngle) {
    scoopServo.write(currentServoAngle);
    delay(SERVO_FINAL_SETTLE_MS);
    return;
  }

  const int stepDirection = (targetAngle > currentServoAngle) ? 1 : -1;

  while (currentServoAngle != targetAngle) {
    const int remainingDegrees = abs(targetAngle - currentServoAngle);

    currentServoAngle += stepDirection;
    scoopServo.write(currentServoAngle);

    if (remainingDegrees > 30) {
      delay(baseDelayMs);
    } else if (remainingDegrees > 10) {
      delay(baseDelayMs * 2);
    } else {
      delay(baseDelayMs * 4);
    }
  }

  scoopServo.write(currentServoAngle);
  delay(SERVO_FINAL_SETTLE_MS);
}

void lockServoAtCurrentAngle() {
  if (!scoopServo.attached()) {
    scoopServo.attach(SERVO_PIN);
  }

  scoopServo.write(currentServoAngle);
}

void detachServo() {
  scoopServo.detach();
}

void movePitchUpWithServoLevel(unsigned long steps, int servoEndAngle) {
  servoEndAngle = constrain(servoEndAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

  const unsigned long firstThird  = steps / 3;
  const unsigned long secondThird = steps / 3;
  const unsigned long finalThird  = steps - firstThird - secondThird;

  const int servoStartAngle = currentServoAngle;
  int servoFirstAlignAngle  = servoStartAngle + ((servoEndAngle - servoStartAngle) / 3);
  int servoSecondAlignAngle = servoStartAngle + ((2 * (servoEndAngle - servoStartAngle)) / 3);
  int servoThirdAlignAngle = servoEndAngle;

  servoFirstAlignAngle  = constrain(servoFirstAlignAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  servoSecondAlignAngle = constrain(servoSecondAlignAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  servoThirdAlignAngle = constrain(servoThirdAlignAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

  lockServoAtCurrentAngle();
  delay(300);

  Serial.println("Pitch up first third");
  movePitch(PITCH_UP_DIR, firstThird, PITCH_UP_STEP_DELAY_US, false);
  delay(250);

  Serial.println("Servo align first time");
  moveServoSlowToAndLock(servoFirstAlignAngle + SERVO_ALIGN_OFFSET_DEG, SERVO_SLOW_DELAY_MS);
  delay(250);

  Serial.println("Pitch up second third");
  movePitch(PITCH_UP_DIR, secondThird, PITCH_UP_STEP_DELAY_US, false);
  delay(250);

  Serial.println("Servo align second time");
  moveServoSlowToAndLock(servoSecondAlignAngle + SERVO_ALIGN_OFFSET_DEG, SERVO_SLOW_DELAY_MS);
  delay(250);

  Serial.println("Pitch up final third");
  movePitch(PITCH_UP_DIR, finalThird, PITCH_UP_STEP_DELAY_US, false);
  delay(250);

  Serial.println("Servo align third time");
  moveServoSlowToAndLock(servoThirdAlignAngle + SERVO_ALIGN_OFFSET_DEG, SERVO_SLOW_DELAY_MS);
  delay(250);

  // Leave pitch enabled so the motor holds position.
  digitalWrite(PITCH_STEP_PIN, LOW);
  digitalWrite(PITCH_ENABLE_PIN, DRIVER_ENABLED_STATE);
  delay(250);
}

// ============================================================
// DRIVE MOTOR HELPERS
// ============================================================

void driveForwardForTime() {
  digitalWrite(DRIVE_DIR_PIN, DRIVE_FORWARD_DIR);
  analogWrite(DRIVE_PWM_PIN, DRIVE_SPEED);

  delay(DRIVE_TIME_MS);

  stopDriveMotor();
}

void stopDriveMotor() {
  analogWrite(DRIVE_PWM_PIN, 0);
}