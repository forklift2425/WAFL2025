const int potPin = A0;
const int pwmPin = 6;
const int dirPin = 7;

float Kp = 20;  // Proportional gain — tune this!
float targetAngle = 0;  // Desired steering angle in degrees (-90 to 90)
float currentAngle = 0;

int deadband = 0.1; // degrees — adjust to prevent motor chatter

void setup() {
  Serial.begin(9600);
  pinMode(potPin, INPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  Serial.println("Enter target steering angle (-90 to 90 degrees):");
}

void loop() {
  // Read target angle from serial
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    targetAngle = input.toFloat();
    targetAngle = constrain(targetAngle, -90.0, 90.0);  // clamp for safety
    Serial.print("New target angle set: ");
    Serial.println(targetAngle);
  }

  // Read potentiometer and compute actual wheel angle
  int currentPotValue = analogRead(potPin);
  float potTurns = (currentPotValue - 512) / 1023.0 * 10.0; // ±5 turns
  float wheelTurns = potTurns * 3.0 / 47.0;
  currentAngle = wheelTurns * 360.0;

  // Calculate control error
  float error = targetAngle - currentAngle;

  // Optional deadband
  if (abs(error) < deadband) {
    analogWrite(pwmPin, 0);  // Stop motor
    return;
  }

  // Proportional control
  int controlSignal = abs(Kp * error);  // scale control effort
  controlSignal = constrain(controlSignal, 0, 255);  // limit PWM

  // Set direction
  digitalWrite(dirPin, error > 0 ? HIGH : LOW);  // HIGH = steer right?

  // Drive motor
  analogWrite(pwmPin, controlSignal);

  // Optional: Debugging output
  Serial.print("Current Angle: ");
  Serial.print(currentAngle);
  Serial.print(" | Error: ");
  Serial.print(error);
  Serial.print(" | PWM: ");
  Serial.println(controlSignal);

  delay(10);
}
