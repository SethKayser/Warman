// =======================
// JOYSTICK AXIS
// =======================

#define JOY_AXIS_PIN A0

// =======================
// MAIN VARIABLES TO CHANGE
// =======================

// Smaller = more sensitive
int JOY_DEADZONE = 80;

// Direction names (for serial debug)
const char* NEGATIVE_NAME = "MOTOR REVERSE";
const char* POSITIVE_NAME = "MOTOR FORWARD";

// If joystick direction feels backwards, change to true
bool INVERT_JOYSTICK = false;

// =======================
// PIN DEFINITIONS
// =======================

const int BUTTON_PIN = 22;  // Unused, kept for reference
const int DIR_PIN    = 31;
const int GO_PIN     = 44;
const int ENC_A      = 67;  // Interrupt pin
const int ENC_B      = 66;

// =======================
// ENCODER
// =======================

volatile long encoderCount = 0;

void encoderISR() {
  if (digitalRead(ENC_B) == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

// =======================
// INTERNAL
// =======================

int JOY_CENTER = 0;
String currentAction = "STOP";
unsigned long lastPrintTime = 0;

void setup() {
  Serial.begin(9600);

  pinMode(DIR_PIN, OUTPUT);
  pinMode(GO_PIN, OUTPUT);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  digitalWrite(DIR_PIN, LOW);
  digitalWrite(GO_PIN, LOW);  // Motor off initially

  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, RISING);

  // Auto-calibrate joystick centre
  long total = 0;
  for (int i = 0; i < 100; i++) {
    total += analogRead(JOY_AXIS_PIN);
    delay(5);
  }
  JOY_CENTER = total / 100;

  Serial.println("Joystick H-bridge control ready");
  Serial.print("Joystick centre = ");
  Serial.println(JOY_CENTER);
}

void loop() {
  int joyValue = analogRead(JOY_AXIS_PIN);
  controlMotor(joyValue);

  // Print encoder position every 200ms
  unsigned long now = millis();
  if (now - lastPrintTime >= 200) {
    lastPrintTime = now;
    Serial.print("Encoder: ");
    Serial.println(encoderCount);
  }
}

void controlMotor(int joyValue) {
  int offset = joyValue - JOY_CENTER;
  int absOffset = abs(offset);

  // Stop when joystick is centred
  if (absOffset < JOY_DEADZONE) {
    digitalWrite(GO_PIN, LOW);  // Motor OFF

    if (currentAction != "STOP") {
      currentAction = "STOP";
      Serial.println("MOTOR STOP");
    }
    return;
  }

  // Determine direction based on joystick offset
  bool goPositive;
  if (offset > 0) {
    goPositive = !INVERT_JOYSTICK;
  } else {
    goPositive = INVERT_JOYSTICK;
  }

  String newAction;
  if (goPositive) {
    digitalWrite(DIR_PIN, HIGH);
    newAction = POSITIVE_NAME;
  } else {
    digitalWrite(DIR_PIN, LOW);
    newAction = NEGATIVE_NAME;
  }

  digitalWrite(GO_PIN, HIGH);  // Motor ON

  if (newAction != currentAction) {
    currentAction = newAction;
    Serial.println(currentAction);
  }
}
