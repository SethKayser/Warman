const int buttonPin = 22;
const int ledPin = 12;

int buttonState = 0;
int brightness = 8;  // PWM value: 0 (off) to 255 (full brightness)

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
}

void loop() {
  buttonState = digitalRead(buttonPin);

  if (buttonState == HIGH) {
    analogWrite(ledPin, brightness);  // Dimmed ON
  } else {
    analogWrite(ledPin, 0);           // Off
  }
}