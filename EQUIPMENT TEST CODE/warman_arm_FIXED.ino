// Warman arm control - CORRECTED VERSION
// AD0 / A0 = joystick X axis = PITCH left/right
// AD1 / A1 = joystick Y axis = ROTATION
//
// Extension on U slot with D26 button, D27 zero switch, and hard limit
// Pitch NEMA 23 on V STEP/DIR only
// Rotation on Z slot

// =====================================================
// EXTENSION - U SLOT
// =====================================================

const int EXTENSION_ENABLE_PIN = 16;
const int EXTENSION_STEP_PIN   = 17;
const int EXTENSION_DIR_PIN    = 37;

// =====================================================
// PITCH - V SLOT, NEMA 23 EXTERNAL DRIVER
// ONLY STEP AND DIR CONNECTED
// =====================================================

const int PITCH_STEP_PIN = 43;
const int PITCH_DIR_PIN  = 57;

// =====================================================
// ROTATION - Z SLOT
// =====================================================

const int ROTATION_ENABLE_PIN = 34;
const int ROTATION_STEP_PIN   = 35;
const int ROTATION_DIR_PIN    = 36;

// =====================================================
// BUTTONS
// =====================================================

const int EXTENSION_BUTTON_PIN = 26;
const int EXTENSION_ZERO_SWITCH_PIN = 27;

// If D26 button does nothing, change HIGH to LOW
const int BUTTON_PRESSED_STATE = HIGH;

// D27 zero switch uses INPUT_PULLUP
// not pressed = HIGH
// pressed = LOW
const int ZERO_SWITCH_PRESSED_STATE = LOW;

// =====================================================
// JOYSTICK INPUTS
// =====================================================

const int JOY_PITCH_PIN    = A0;  // AD0 / X axis = pitch
const int JOY_ROTATION_PIN = A1;  // AD1 / Y axis = rotation

// =====================================================
// EXTENSION SETTINGS
// =====================================================

const int EXTENSION_OUT_DIR = LOW;
const int EXTENSION_IN_DIR  = HIGH;

int currentExtensionDirection = EXTENSION_OUT_DIR;

// Bigger = slower
int EXTENSION_OUT_STEP_DELAY_US = 1200;
int EXTENSION_IN_STEP_DELAY_US  = 1200;

// Hard limits
const long EXTENSION_MIN_POS = 0;
const long EXTENSION_HARD_STOP_POS = 3600;

// Slow near ends
const long EXTENSION_SLOW_START_POS = 2954;
const long EXTENSION_RETRACT_SLOW_START_POS = 600;

int EXTENSION_OUT_SLOW_END_DELAY_US = 2500;
int EXTENSION_IN_SLOW_END_DELAY_US  = 2500;

// =====================================================
// PITCH SETTINGS - V / NEMA 23
// =====================================================

int PITCH_DEADZONE = 180;

// Change these if pitch direction is backwards
const int PITCH_NEGATIVE_DIR = LOW;
const int PITCH_POSITIVE_DIR = HIGH;

// Your working pitch speed values
int PITCH_POSITIVE_SLOW_STEP_DELAY_US = 800;
int PITCH_POSITIVE_FAST_STEP_DELAY_US = 500;

int PITCH_NEGATIVE_SLOW_STEP_DELAY_US = 800;
int PITCH_NEGATIVE_FAST_STEP_DELAY_US = 500;

int PITCH_STEP_PULSE_WIDTH_US = 20;

bool INVERT_PITCH_JOYSTICK = false;

// =====================================================
// ROTATION SETTINGS - Z
// =====================================================

int ROTATION_DEADZONE = 300;

// Change these if rotation direction is backwards
const int ROTATION_LEFT_DIR  = HIGH;
const int ROTATION_RIGHT_DIR = LOW;

int ROTATION_LEFT_STEP_DELAY_US  = 2000;
int ROTATION_RIGHT_STEP_DELAY_US = 2000;

int ROTATION_STEP_PULSE_WIDTH_US = 8;

bool INVERT_ROTATION_JOYSTICK = false;

// =====================================================
// RUNTIME VARIABLES
// =====================================================

int pitchJoyCenter = 512;
int rotationJoyCenter = 512;

String currentPitchAction = "STOP";
String currentRotationAction = "STOP";
String currentExtensionAction = "STOP";

unsigned long lastPitchStepTime = 0;
unsigned long lastExtensionStepTime = 0;
unsigned long lastRotationStepTime = 0;

long extensionStepPosition = 0;

unsigned long DOUBLE_CLICK_GAP_MS = 500;  // Increased from 350ms for easier use
bool buttonWasPressed = false;
unsigned long lastButtonReleaseTime = 0;

bool zeroSwitchWasPressed = false;
unsigned long lastZeroSwitchPressTime = 0;
const unsigned long ZERO_SWITCH_DEBOUNCE_MS = 250;

// Motor enable state tracking
bool extensionEnabled = false;
bool rotationEnabled = false;

void setup() {
  Serial.begin(9600);

  // Extension
  pinMode(EXTENSION_ENABLE_PIN, OUTPUT);
  pinMode(EXTENSION_STEP_PIN, OUTPUT);
  pinMode(EXTENSION_DIR_PIN, OUTPUT);

  digitalWrite(EXTENSION_ENABLE_PIN, HIGH);  // Start disabled
  digitalWrite(EXTENSION_STEP_PIN, LOW);
  digitalWrite(EXTENSION_DIR_PIN, currentExtensionDirection);

  pinMode(EXTENSION_BUTTON_PIN, INPUT);
  pinMode(EXTENSION_ZERO_SWITCH_PIN, INPUT_PULLUP);

  // Pitch NEMA 23 external driver
  pinMode(PITCH_STEP_PIN, OUTPUT);
  pinMode(PITCH_DIR_PIN, OUTPUT);

  digitalWrite(PITCH_STEP_PIN, LOW);
  digitalWrite(PITCH_DIR_PIN, LOW);

  // Rotation
  pinMode(ROTATION_ENABLE_PIN, OUTPUT);
  pinMode(ROTATION_STEP_PIN, OUTPUT);
  pinMode(ROTATION_DIR_PIN, OUTPUT);

  digitalWrite(ROTATION_ENABLE_PIN, HIGH);  // Start disabled
  digitalWrite(ROTATION_STEP_PIN, LOW);
  digitalWrite(ROTATION_DIR_PIN, LOW);

  calibrateJoysticks();

  Serial.println("========================================");
  Serial.println("WARMAN ARM READY");
  Serial.println("========================================");
  Serial.println("A0 / AD0 = joystick X = pitch");
  Serial.println("A1 / AD1 = joystick Y = rotation");
  Serial.println("Extension = U slot, D26 button, D27 zero switch");
  Serial.println("Pitch NEMA 23 = V STEP/DIR only");
  Serial.println("Rotation = Z slot");
  Serial.println();

  Serial.print("Pitch centre = ");
  Serial.println(pitchJoyCenter);

  Serial.print("Rotation centre = ");
  Serial.println(rotationJoyCenter);

  Serial.print("Extension hard stop = ");
  Serial.println(EXTENSION_HARD_STOP_POS);
  Serial.println("========================================");
  Serial.println();
}

void loop() {
  handleZeroSwitch();
  handleExtensionButton();

  int pitchJoy = analogRead(JOY_PITCH_PIN);
  int rotationJoy = analogRead(JOY_ROTATION_PIN);

  controlPitch(pitchJoy);
  controlRotation(rotationJoy);
}

// =====================================================
// JOYSTICK CALIBRATION
// =====================================================

void calibrateJoysticks() {
  Serial.println("Calibrating joysticks...");
  Serial.println("Keep joystick centred for 2 seconds!");
  
  delay(2000);  // Give user time to centre joystick
  
  long pitchTotal = 0;
  long rotationTotal = 0;

  for (int i = 0; i < 200; i++) {  // More samples for better accuracy
    pitchTotal += analogRead(JOY_PITCH_PIN);
    rotationTotal += analogRead(JOY_ROTATION_PIN);
    delay(5);
  }

  pitchJoyCenter = pitchTotal / 200;
  rotationJoyCenter = rotationTotal / 200;
  
  Serial.println("Calibration complete!");
}

// =====================================================
// EXTENSION ZERO SWITCH
// =====================================================

bool isZeroSwitchPressed() {
  return digitalRead(EXTENSION_ZERO_SWITCH_PIN) == ZERO_SWITCH_PRESSED_STATE;
}

void handleZeroSwitch() {
  bool zeroPressed = isZeroSwitchPressed();
  unsigned long nowMs = millis();

  if (zeroPressed && !zeroSwitchWasPressed) {
    // FIXED: Update timestamp before debounce check
    if (nowMs - lastZeroSwitchPressTime > ZERO_SWITCH_DEBOUNCE_MS) {
      extensionStepPosition = 0;

      Serial.println(">>> ZERO SWITCH HIT <<<");
      Serial.println("Extension position reset to 0");
    }
    lastZeroSwitchPressTime = nowMs;
  }

  zeroSwitchWasPressed = zeroPressed;
}

// =====================================================
// EXTENSION BUTTON CONTROL
// =====================================================

void handleExtensionButton() {
  bool buttonPressed = digitalRead(EXTENSION_BUTTON_PIN) == BUTTON_PRESSED_STATE;
  unsigned long nowMs = millis();

  if (buttonPressed && !buttonWasPressed) {
    if ((nowMs - lastButtonReleaseTime) <= DOUBLE_CLICK_GAP_MS) {
      reverseExtensionDirection();
    }
  }

  if (!buttonPressed && buttonWasPressed) {
    lastButtonReleaseTime = nowMs;
  }

  buttonWasPressed = buttonPressed;

  if (buttonPressed) {
    enableExtension();
    moveExtension();
  } else {
    disableExtension();
  }
}

void enableExtension() {
  if (!extensionEnabled) {
    digitalWrite(EXTENSION_ENABLE_PIN, LOW);  // LOW = enabled on MMDS
    extensionEnabled = true;
  }
}

void disableExtension() {
  if (extensionEnabled) {
    digitalWrite(EXTENSION_ENABLE_PIN, HIGH);  // HIGH = disabled on MMDS
    extensionEnabled = false;
    
    if (currentExtensionAction != "STOP") {
      currentExtensionAction = "STOP";
      Serial.println("EXTENSION STOP");
    }
  }
}

void reverseExtensionDirection() {
  if (currentExtensionDirection == EXTENSION_OUT_DIR) {
    currentExtensionDirection = EXTENSION_IN_DIR;
    Serial.println(">>> Extension direction: IN <<<");
  } else {
    currentExtensionDirection = EXTENSION_OUT_DIR;
    Serial.println(">>> Extension direction: OUT <<<");
  }

  digitalWrite(EXTENSION_DIR_PIN, currentExtensionDirection);
}

void moveExtension() {
  if (currentExtensionDirection == EXTENSION_OUT_DIR) {
    if (extensionStepPosition >= EXTENSION_HARD_STOP_POS) {
      extensionStepPosition = EXTENSION_HARD_STOP_POS;
      
      if (currentExtensionAction != "HARD_STOP_OUT") {
        currentExtensionAction = "HARD_STOP_OUT";
        Serial.println("EXTENSION HARD STOP REACHED");
      }
      return;
    }

    if (currentExtensionAction != "OUT") {
      currentExtensionAction = "OUT";
      Serial.println("EXTENSION OUT");
    }

    digitalWrite(EXTENSION_DIR_PIN, EXTENSION_OUT_DIR);
    stepExtension(1);
  }

  else if (currentExtensionDirection == EXTENSION_IN_DIR) {
    if (extensionStepPosition <= EXTENSION_MIN_POS || isZeroSwitchPressed()) {
      extensionStepPosition = 0;
      
      if (currentExtensionAction != "ZERO_LIMIT") {
        currentExtensionAction = "ZERO_LIMIT";
        if (isZeroSwitchPressed()) {
          Serial.println("EXTENSION AT ZERO SWITCH");
        } else {
          Serial.println("EXTENSION AT MIN POSITION");
        }
      }
      return;
    }

    if (currentExtensionAction != "IN") {
      currentExtensionAction = "IN";
      Serial.println("EXTENSION IN");
    }

    digitalWrite(EXTENSION_DIR_PIN, EXTENSION_IN_DIR);
    stepExtension(-1);
  }
}

void stepExtension(int direction) {
  unsigned long now = micros();

  int effectiveDelayUs;

  if (direction > 0) {
    effectiveDelayUs = EXTENSION_OUT_STEP_DELAY_US;

    if (extensionStepPosition >= EXTENSION_SLOW_START_POS) {
      effectiveDelayUs = map(
        constrain(extensionStepPosition, EXTENSION_SLOW_START_POS, EXTENSION_HARD_STOP_POS),
        EXTENSION_SLOW_START_POS,
        EXTENSION_HARD_STOP_POS,
        EXTENSION_OUT_STEP_DELAY_US,
        EXTENSION_OUT_SLOW_END_DELAY_US
      );
    }
  }

  else {
    effectiveDelayUs = EXTENSION_IN_STEP_DELAY_US;

    if (extensionStepPosition <= EXTENSION_RETRACT_SLOW_START_POS) {
      effectiveDelayUs = map(
        constrain(extensionStepPosition, EXTENSION_MIN_POS, EXTENSION_RETRACT_SLOW_START_POS),
        EXTENSION_RETRACT_SLOW_START_POS,
        EXTENSION_MIN_POS,
        EXTENSION_IN_STEP_DELAY_US,
        EXTENSION_IN_SLOW_END_DELAY_US
      );
    }
  }

  if (now - lastExtensionStepTime >= (unsigned long)effectiveDelayUs) {
    digitalWrite(EXTENSION_STEP_PIN, HIGH);
    delayMicroseconds(8);
    digitalWrite(EXTENSION_STEP_PIN, LOW);

    lastExtensionStepTime = now;

    extensionStepPosition += direction;

    if (extensionStepPosition < EXTENSION_MIN_POS) {
      extensionStepPosition = EXTENSION_MIN_POS;
    }

    if (extensionStepPosition > EXTENSION_HARD_STOP_POS) {
      extensionStepPosition = EXTENSION_HARD_STOP_POS;
    }
  }
}

// =====================================================
// PITCH CONTROL - V STEP/DIR ONLY
// =====================================================

void controlPitch(int joyValue) {
  int offset = joyValue - pitchJoyCenter;

  if (INVERT_PITCH_JOYSTICK) {
    offset = -offset;
  }

  int absOffset = abs(offset);

  if (absOffset < PITCH_DEADZONE) {
    digitalWrite(PITCH_STEP_PIN, LOW);

    if (currentPitchAction != "STOP") {
      currentPitchAction = "STOP";
      Serial.println("PITCH STOP");
    }

    return;
  }

  int stepDelay;
  String newAction;

  // Max offset is ~511 from centre, constrain to 500 for mapping
  int mappedOffset = constrain(absOffset, PITCH_DEADZONE, 500);

  if (offset > 0) {
    digitalWrite(PITCH_DIR_PIN, PITCH_POSITIVE_DIR);
    newAction = "PITCH POSITIVE";

    stepDelay = map(
      mappedOffset,
      PITCH_DEADZONE,
      500,
      PITCH_POSITIVE_SLOW_STEP_DELAY_US,
      PITCH_POSITIVE_FAST_STEP_DELAY_US
    );
  } else {
    digitalWrite(PITCH_DIR_PIN, PITCH_NEGATIVE_DIR);
    newAction = "PITCH NEGATIVE";

    stepDelay = map(
      mappedOffset,
      PITCH_DEADZONE,
      500,
      PITCH_NEGATIVE_SLOW_STEP_DELAY_US,
      PITCH_NEGATIVE_FAST_STEP_DELAY_US
    );
  }

  if (newAction != currentPitchAction) {
    currentPitchAction = newAction;
    Serial.println(currentPitchAction);
  }

  unsigned long now = micros();

  if (now - lastPitchStepTime >= (unsigned long)stepDelay) {
    lastPitchStepTime = now;

    digitalWrite(PITCH_STEP_PIN, HIGH);
    delayMicroseconds(PITCH_STEP_PULSE_WIDTH_US);
    digitalWrite(PITCH_STEP_PIN, LOW);
  }
}

// =====================================================
// ROTATION CONTROL - Z SLOT
// FIXED: Inline timing like pitch, no shared function
// =====================================================

void controlRotation(int joyValue) {
  int offset = joyValue - rotationJoyCenter;

  if (INVERT_ROTATION_JOYSTICK) {
    offset = -offset;
  }

  int absOffset = abs(offset);

  if (absOffset < ROTATION_DEADZONE) {
    digitalWrite(ROTATION_STEP_PIN, LOW);
    
    disableRotation();

    if (currentRotationAction != "STOP") {
      currentRotationAction = "STOP";
      Serial.println("ROTATION STOP");
    }

    return;
  }

  enableRotation();

  int stepDelay;
  String newAction;

  if (offset > ROTATION_DEADZONE) {
    digitalWrite(ROTATION_DIR_PIN, ROTATION_RIGHT_DIR);
    newAction = "ROTATION RIGHT";
    stepDelay = ROTATION_RIGHT_STEP_DELAY_US;
  }

  else if (offset < -ROTATION_DEADZONE) {
    digitalWrite(ROTATION_DIR_PIN, ROTATION_LEFT_DIR);
    newAction = "ROTATION LEFT";
    stepDelay = ROTATION_LEFT_STEP_DELAY_US;
  }

  if (newAction != currentRotationAction) {
    currentRotationAction = newAction;
    Serial.println(currentRotationAction);
  }

  // FIXED: Inline timing check instead of broken reference function
  unsigned long now = micros();

  if (now - lastRotationStepTime >= (unsigned long)stepDelay) {
    lastRotationStepTime = now;

    digitalWrite(ROTATION_STEP_PIN, HIGH);
    delayMicroseconds(ROTATION_STEP_PULSE_WIDTH_US);
    digitalWrite(ROTATION_STEP_PIN, LOW);
  }
}

void enableRotation() {
  if (!rotationEnabled) {
    digitalWrite(ROTATION_ENABLE_PIN, LOW);  // LOW = enabled on MMDS
    rotationEnabled = true;
  }
}

void disableRotation() {
  if (rotationEnabled) {
    digitalWrite(ROTATION_ENABLE_PIN, HIGH);  // HIGH = disabled on MMDS
    rotationEnabled = false;
  }
}
