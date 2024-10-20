const int motorPin1 = 3;   // Motor driver input pin 1
const int motorPin2 = 4;   // Motor driver input pin 2
const int buttonPin = 2;   // Button pin

void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(buttonPin, INPUT); // Use internal pull-up resistor

  // Initialize motor to be off
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
}

void loop() {
  if (digitalRead(buttonPin) == 1) { // Button pressed
    // Spin motor forward for 10 seconds
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    delay(10000); // 10 seconds

    // Stop motor for a brief moment
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    delay(1000); // 1 second

    // Spin motor backward for 10 seconds
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    delay(10000); // 10 seconds

    // Stop motor
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
  }
}
