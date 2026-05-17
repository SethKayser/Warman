// Pin definitions
#define BUTTON_PIN  22
#define ENABLE_PIN  42
#define STEP_PIN    43
#define DIR_PIN     57

// Stepper settings
#define STEP_DELAY_US  500   // Microseconds between steps (adjust for speed)

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Button pulls LOW when pressed
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(STEP_PIN,   OUTPUT);
  pinMode(DIR_PIN,    OUTPUT);

  digitalWrite(ENABLE_PIN, HIGH);  // Disable motor initially (most drivers: HIGH = disabled)
  digitalWrite(DIR_PIN,    LOW);   // Set direction
}

void loop() {
  if (digitalRead(BUTTON_PIN) == HIGH) {  // Button held down
    digitalWrite(ENABLE_PIN, LOW);        // Enable motor
    doStep();
  } else {
    digitalWrite(ENABLE_PIN, HIGH);       // Disable motor (saves power, reduces heat)
  }
}

void doStep() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_DELAY_US);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(STEP_DELAY_US);
}