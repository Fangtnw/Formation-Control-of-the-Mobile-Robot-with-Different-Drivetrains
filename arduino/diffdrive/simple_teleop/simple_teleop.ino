#include <Xicro_coop_teleop_ID_1.h>
#include <PID_v1_bc.h>

Xicro xicro;

// Motor 1
const int motor1INA = 12;
const int motor1INB = 11;
const int motor1PWM = 10;

// Motor 2
const int motor2INA = 6;
const int motor2INB = 7;
const int motor2PWM = 5;

const int PIN_ENCOD_A_MOTOR_LEFT = 2;
const int PIN_ENCOD_B_MOTOR_LEFT = 8;

const int PIN_ENCOD_A_MOTOR_RIGHT = 3;
const int PIN_ENCOD_B_MOTOR_RIGHT = 9;

const double radius = 0.06;
const double wheelbase = 0.55;
const double encoder_cpr = 600;
const double speed_to_pwm_ratio = 0.00235;
const double min_speed_cmd = 0.0882;



void setup() {
  pinMode(motor1INA, OUTPUT);
  pinMode(motor1INB, OUTPUT);
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor2INA, OUTPUT);
  pinMode(motor2INB, OUTPUT);
  pinMode(motor2PWM, OUTPUT);

  Serial.begin(115200);
  xicro.begin(&Serial);

  // Initialize PID instances

  Serial.println("Setup complete");
}

void loop() {
  // Extract the linear and angular velocities from the twist message
  float linearX = xicro.Subscription_cmd_vel.message.linear.x;
  float angularZ = xicro.Subscription_cmd_vel.message.angular.z;

  Serial.print(linearX);

  // Calculate the wheel velocities based on the linear and angular velocities
  float wheel1Velocity = linearX - angularZ * 0.5;
  float wheel2Velocity = linearX + angularZ * 0.5;

  // Set the motor directions and speeds based on the wheel velocities
  if (wheel1Velocity > 0) {
    digitalWrite(motor1INA, HIGH);
    digitalWrite(motor1INB, LOW);
    analogWrite(motor1PWM, map(wheel1Velocity, 0, 1, 0, 255));
  } else if (wheel1Velocity < 0) {
    digitalWrite(motor1INA, LOW);
    digitalWrite(motor1INB, HIGH);
    analogWrite(motor1PWM, map(-wheel1Velocity, 0, 1, 0, 255));
  } else {
    stopMotors();
  }

  if (wheel2Velocity > 0) {
    digitalWrite(motor2INA, HIGH);
    digitalWrite(motor2INB, LOW);
    analogWrite(motor2PWM, map(wheel2Velocity, 0, 1, 0, 255));
  } else if (wheel2Velocity < 0) {
    digitalWrite(motor2INA, LOW);
    digitalWrite(motor2INB, HIGH);
    analogWrite(motor2PWM, map(-wheel2Velocity, 0, 1, 0, 255));
    } else {
    stopMotors();
  }
  xicro.Spin_node();
}


void stopMotors() {
  digitalWrite(motor1INA, LOW);
  digitalWrite(motor1INB, LOW);
  analogWrite(motor1PWM, 0);

  digitalWrite(motor2INA, LOW);
  digitalWrite(motor2INB, LOW);
  analogWrite(motor2PWM, 0);
}
