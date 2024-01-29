#include <Arduino.h>

// Motor control pins
#define Direction_motor1 14
#define PWM_motor1 6

#define Direction_motor2 5
#define PWM_motor2 4

#define Direction_motor3 30
#define PWM_motor3 12

#define Direction_motor4 32
#define PWM_motor4 13

void setup() {
  pinMode(Direction_motor1, OUTPUT);
  pinMode(PWM_motor1, OUTPUT);

  pinMode(Direction_motor2, OUTPUT);
  pinMode(PWM_motor2, OUTPUT);

  pinMode(Direction_motor3, OUTPUT);
  pinMode(PWM_motor3, OUTPUT);

  pinMode(Direction_motor4, OUTPUT);
  pinMode(PWM_motor4, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  // Test motor 1 forward
  digitalWrite(Direction_motor1, HIGH);
  analogWrite(PWM_motor1, 200); // Adjust PWM value as needed

  // Test motor 2 forward
  digitalWrite(Direction_motor2, HIGH);
  analogWrite(PWM_motor2, 200); // Adjust PWM value as needed

  // Test motor 3 forward
  digitalWrite(Direction_motor3, HIGH);
  analogWrite(PWM_motor3, 200); // Adjust PWM value as needed

  // Test motor 4 forward
  digitalWrite(Direction_motor4, HIGH);
  analogWrite(PWM_motor4, 200); // Adjust PWM value as needed

  delay(5000); // Run motors forward for 5 seconds

  // Stop motors
  digitalWrite(Direction_motor1, HIGH);
  analogWrite(PWM_motor1, 0);

  digitalWrite(Direction_motor2, HIGH);
  analogWrite(PWM_motor2, 0);

  digitalWrite(Direction_motor3, HIGH);
  analogWrite(PWM_motor3, 0);

  digitalWrite(Direction_motor4, HIGH);
  analogWrite(PWM_motor4, 0);

  delay(2000); // Pause for 2 seconds

  // Repeat the test for backward motion if needed
}
