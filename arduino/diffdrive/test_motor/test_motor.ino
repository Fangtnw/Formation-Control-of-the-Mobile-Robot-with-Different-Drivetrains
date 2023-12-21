// Motor 1
const int motor1INA = 12;
const int motor1INB = 11;
const int motor1PWM = 10;

// Motor 2
const int motor2INA = 6;
const int motor2INB = 7;
const int motor2PWM = 5;

void setup() {
  // Set the motor control pins as outputs
  pinMode(motor1INA, OUTPUT);
  pinMode(motor1INB, OUTPUT);
  pinMode(motor1PWM, OUTPUT);

  pinMode(motor2INA, OUTPUT);
  pinMode(motor2INB, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
}

void loop() {

  moveMotorsForward();

}

void moveMotorsForward() {
  // Motor 1
  digitalWrite(motor1INA, HIGH);
  digitalWrite(motor1INB, LOW);
  analogWrite(motor1PWM, 255);  // 255 is the maximum PWM value for full speed

  // Motor 2
  digitalWrite(motor2INA, HIGH);
  digitalWrite(motor2INB, LOW);
  analogWrite(motor2PWM, 255);
}

void moveMotorsBackward() {
  // Motor 1
  digitalWrite(motor1INA, LOW);
  digitalWrite(motor1INB, HIGH);
  analogWrite(motor1PWM, 255);

  // Motor 2
  digitalWrite(motor2INA, LOW);
  digitalWrite(motor2INB, HIGH);
  analogWrite(motor2PWM, 255);
}

void stopMotors() {
  // Motor 1
  digitalWrite(motor1INA, LOW);
  digitalWrite(motor1INB, LOW);
  analogWrite(motor1PWM, 0);  // 0 PWM value stops the motor

  // Motor 2
  digitalWrite(motor2INA, LOW);
  digitalWrite(motor2INB, LOW);
  analogWrite(motor2PWM, 0);
}
