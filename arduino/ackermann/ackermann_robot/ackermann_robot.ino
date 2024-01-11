#include <PID_v1.h>
#include <Xicro_ack_xicro_ID_2.h>
#include <Wire.h>
#include <Servo.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
Servo servo;
Xicro xicro;
//initializing all the variables
#define LOOPTIME                      100     //Looptime in millisecond
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter
unsigned long lastMilli = 0;

//DC MOTOR1
const int motor1INA = 12;
const int motor1INB = 11;
const int motor1PWM = 10;

//DC MOTOR2
const int motor2INA = 6;
const int motor2INB = 7;
const int motor2PWM = 5;

//SERVO
const int servoIN = 13;
bool key_pressed = false;
const int servoMAX = 180;
const int servoMIN = 0;

//Encoder
const int PIN_ENCOD_A_MOTOR_LEFT = 8;               //A channel for encoder of left motor                    
const int PIN_ENCOD_B_MOTOR_LEFT = 9;               //B channel for encoder of left motor

const int PIN_ENCOD_A_MOTOR_RIGHT = 47;         //A channel for encoder of right motor         
const int PIN_ENCOD_B_MOTOR_RIGHT = 49;         //B channel for encoder of right motor 

const int PIN_ENCOD_A_SERVO_LEFT = 48;              //A channel for encoder of left servo motor         
const int PIN_ENCOD_B_SERVO_LEFT = 46;              //B channel for encoder of left servo motor 

const int PIN_ENCOD_A_SERVO_RIGHT = 18;              //A channel for encoder of right servo motor         
const int PIN_ENCOD_B_SERVO_RIGHT = 19;              //B channel for encoder of right servo motor 

//Define Value
const double radius = 0.06;                   //Wheel radius, in m
const double wheelbase = 0.58;               //Wheelbase, in m
const double wheel_length = 0.58;               //Wheelbase, in m
const double encoder_cpr = 600;               //Encoder ticks or counts per rotation
const double speed_to_pwm_ratio = 0.00235;    //Ratio to convert speed (in m/s) to PWM value. It was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the slope of the linear function).
const double min_speed_cmd = 0.0882;          //(min_speed_cmd/speed_to_pwm_ratio) is the minimum command value needed for the motor to start moving. This value was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the constant of the linear function).

double speed_req = 0;                         //Desired linear speed for the robot, in m/s
double angular_speed_req = 0;                 //Desired angular speed for the robot, in rad/s

double speed_req_left = 0;                    //Desired speed for left wheel in m/s
double speed_act_left = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_left = 0;                    //Command speed for left wheel in m/s 

double speed_req_right = 0;                   //Desired speed for right wheel in m/s
double speed_act_right = 0;                   //Actual speed for right wheel in m/s
double speed_cmd_right = 0;                   //Command speed for right wheel in m/s 

double ang_req_servo = 0;                     //Desired angle for servo motor
double ang_act_servo = 0;                     //Actual angle for servo motor
double ang_cmd_servo = 0;                     //Command angle for servo motor

const double max_speed = 0.4;                 //Max speed (m/s)
const double max_angle = 60;                 //Max angle (degree)
const double min_angle = 0;                 //Min angle (degree)

double speedlinx = 0;
double speedliny = 0;
double speedangz = 0;                 //Desired angular speed for the robot, in rad/s
float ax, ay, az;  
float gx, gy, gz;  

float orientation[4]={0};
float orientation_covariance[9]={1,0,0,0,1,0,0,0,1};
float angular_velocity[3]={0};
float angular_velocity_covariance[9]={1,0,0,0,1,0,0,0,1};
float linear_acceleration[3]={0};
float linear_acceleration_covariance[9]={1,0,0,0,1,0,0,0,1};
double theta = 0 ;

int steer_cmd = 0;/* replace this with your input value */; 
int servo_cmd = 90;

int PWM_leftMotor = 0;                     //PWM command for left motor
int PWM_rightMotor = 0;                    //PWM command for right motor 

//PID Parameters
const double PID_left_param[] = { 1, 0, 0 }; //Respectively Kp, Ki and Kd for left motor PID
const double PID_right_param[] = { 1, 0, 0 }; //Respectively Kp, Ki and Kd for right motor PID
const double PID_servo_param[] = { 1, 0, 0 }; //Respectively Kp, Ki and Kd for servo motor PID

volatile float pos_left = 0;       //Left motor encoder position
volatile float pos_right = 0;      //Right motor encoder position
volatile float pos_servo = 0;      //Servo motor encoder position
volatile float pos_servo_left = 0;      //Servo motor encoder position
volatile float pos_servo_right = 0;      //Servo motor encoder position
double ang_act_servo_left = 0;                     //Actual angle for servo motor
double ang_act_servo_right = 0;                     //Actual angle for servo motor

PID PID_leftMotor(&speed_act_left, &speed_cmd_left, &speed_req_left, PID_left_param[0], PID_left_param[1], PID_left_param[2], DIRECT);          //Setting up the PID for left motor
PID PID_rightMotor(&speed_act_right, &speed_cmd_right, &speed_req_right, PID_right_param[0], PID_right_param[1], PID_right_param[2], DIRECT);   //Setting up the PID for right motor
PID PID_servoMotor(&ang_act_servo, &ang_cmd_servo, &ang_req_servo, PID_servo_param[0], PID_servo_param[1], PID_servo_param[2], DIRECT);   //Setting up the PID for right motor

//const unsigned long print_interval = 1000; // 1 second
//unsigned long previous_print_time = 0;

class motor{
    public:
        int pinA;
        int pinB;
        int pinPWM;
    
    void motor_setup(){
        pinMode(pinA,OUTPUT);
        pinMode(pinB,OUTPUT);
        pinMode(pinPWM,OUTPUT);
    }

    void setSpeed(const int speedInput){
        analogWrite(pinPWM, speedInput);
    }

    void run(const String runInput){
       if(runInput == "BRAKE"){
            digitalWrite(pinA, HIGH);
            digitalWrite(pinB, HIGH);
       }
       else if(runInput == "FORWARD"){
            digitalWrite(pinA, HIGH);
            digitalWrite(pinB, LOW);
       }
       else if(runInput == "BACKWARD"){
            digitalWrite(pinA, LOW);
            digitalWrite(pinB, HIGH);
       }
    }
//    steer_servo.write(servo_pos);
};

void handle_cmd() {
  noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication
  speed_req = xicro.Subscription_cmd_vel.message.linear.x;          //Extract the commanded linear speed from the message
  angular_speed_req = xicro.Subscription_cmd_vel.message.angular.z; //Extract the commanded angular speed from the message

  speed_req_right = speed_req;
  speed_req_left = speed_req;
  ang_req_servo = atan(angular_speed_req*wheel_length)/speed_req;

  pos_servo = (pos_servo_left + pos_servo_right)/2;
 
//  servo.write(servo_pos);
}

motor leftMotor, rightMotor;

void setup() {
  // Set the motor control pins as outputs
  Wire.begin();
  byte status = mpu.begin();
  mpu.calcOffsets(); // gyro and accelero

  leftMotor.pinA = 51;
  leftMotor.pinB = 53;
  leftMotor.pinPWM = 12;
  rightMotor.pinA = 54;
  rightMotor.pinB = 52;
  rightMotor.pinPWM = 11;
  
  pinMode(motor1INA, OUTPUT);
  pinMode(motor1INB, OUTPUT);
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor2INA, OUTPUT);
  pinMode(motor2INB, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
    

  Serial.begin(115200);
  xicro.begin(&Serial);
  Serial.println("Setup complete");
  
  leftMotor.setSpeed(0);
  leftMotor.run("BRAKE");
  rightMotor.setSpeed(0);
  rightMotor.run("BRAKE");

  servo.write(90);
  
  //setting PID parameters
  PID_leftMotor.SetSampleTime(95);
  PID_rightMotor.SetSampleTime(95);
  PID_servoMotor.SetSampleTime(95);
  PID_leftMotor.SetOutputLimits(-max_speed, max_speed);
  PID_rightMotor.SetOutputLimits(-max_speed, max_speed);
  PID_servoMotor.SetOutputLimits(min_angle, max_angle);

  PID_leftMotor.SetMode(AUTOMATIC);
  PID_rightMotor.SetMode(AUTOMATIC);
  PID_servoMotor.SetMode(AUTOMATIC);
    
  // Define the rotary encoder for left motor
  pinMode(PIN_ENCOD_A_MOTOR_LEFT, INPUT); 
  pinMode(PIN_ENCOD_B_MOTOR_LEFT, INPUT); 
  digitalWrite(PIN_ENCOD_A_MOTOR_LEFT, HIGH);                // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_MOTOR_LEFT, HIGH);
  attachInterrupt(0, encoderLeftMotor, RISING);

  // Define the rotary encoder for right motor
  pinMode(PIN_ENCOD_A_MOTOR_RIGHT, INPUT); 
  pinMode(PIN_ENCOD_B_MOTOR_RIGHT, INPUT); 
  digitalWrite(PIN_ENCOD_A_MOTOR_RIGHT, HIGH);                // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_MOTOR_RIGHT, HIGH);
  attachInterrupt(1, encoderRightMotor, RISING);


  // Define the rotary encoder for left servo motor
  pinMode(PIN_ENCOD_A_SERVO_LEFT, INPUT); 
  pinMode(PIN_ENCOD_B_SERVO_LEFT, INPUT); 
  digitalWrite(PIN_ENCOD_A_SERVO_LEFT, HIGH);                // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_SERVO_LEFT, HIGH);
  attachInterrupt(2, encoderLeftServo, RISING);

  //  Define the rotary encoder for right servo motor
  pinMode(PIN_ENCOD_A_SERVO_RIGHT, INPUT);
  pinMode(PIN_ENCOD_B_SERVO_RIGHT, INPUT);
  digitalWrite(PIN_ENCOD_A_SERVO_RIGHT, HIGH);                // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_SERVO_RIGHT, HIGH);
  attachInterrupt(2, encoderRightServo, RISING);


}

void loop() {
  xicro.Spin_node();
  handle_cmd();
  if((millis()-lastMilli) >= LOOPTIME)   
  {                                                                           // enter timed loop
    lastMilli = millis();
    byte status = mpu.begin();
    ang_cmd_servo = 90;
    ang_cmd_servo = constrain(ang_cmd_servo, min_angle, max_angle);
    //PID_servoMotor.Compute();                                               
    servo.write(ang_cmd_servo);

    if (abs(pos_servo) < 5){                                                   //Avoid taking in account small disturbances
      ang_act_servo = 0;
    }
    else {
      ang_act_servo=((pos_servo/encoder_cpr)*360)*(1000/LOOPTIME);          // calculate speed of left wheel
    }
    
    if (abs(pos_left) < 5){                                                   //Avoid taking in account small disturbances
      speed_act_left = 0;
    }
    else {
      speed_act_left=((pos_left/encoder_cpr)*2*PI)*(1000/LOOPTIME)*radius;           // calculate speed of left wheel
    }
    
    if (abs(pos_right) < 5){                                                  //Avoid taking in account small disturbances
      speed_act_right = 0;
    }
    else {
    speed_act_right=((pos_right/encoder_cpr)*2*PI)*(1000/LOOPTIME)*radius;          // calculate speed of right wheel
    }
    
    pos_left = 0;
    pos_right = 0;
        
    speed_cmd_left = constrain(speed_cmd_left, -max_speed, max_speed);
    PID_leftMotor.Compute();                                                 
    // compute PWM value for left motor. Check constant definition comments for more information.
    PWM_leftMotor = constrain(((speed_req_left+sgn(speed_req_left)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_left/speed_to_pwm_ratio), -255, 255); //
    
    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      leftMotor.setSpeed(0);
      leftMotor.run("BRAKE");
      rightMotor.setSpeed(0);
      rightMotor.run("BRAKE");
    }
    else if (speed_req_left == 0){                        //Stopping
      leftMotor.setSpeed(0);
      leftMotor.run("BRAKE");
    }
    else if (PWM_leftMotor > 0){                          //Going "FORWARD"
      leftMotor.setSpeed(abs(PWM_leftMotor));
      leftMotor.run("FORWARD");
    }
    else {                                               //Going ""BACKWARD""
      leftMotor.setSpeed(abs(PWM_leftMotor));
      leftMotor.run("BACKWARD");
    }
    speed_cmd_right = constrain(speed_cmd_right, -max_speed, max_speed);    
    PID_rightMotor.Compute();                                                 
    // compute PWM value for right motor. Check constant definition comments for more information.
    PWM_rightMotor = constrain(((speed_req_right+sgn(speed_req_right)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_right/speed_to_pwm_ratio), -255, 255); // 
    
    if (speed_req_right == 0){                       //Stopping
      rightMotor.setSpeed(0);
      rightMotor.run("BRAKE");
    }
    else if (PWM_rightMotor > 0){                         //Going "FORWARD"
      rightMotor.setSpeed(abs(PWM_rightMotor));
      rightMotor.run("FORWARD");
    }
    else {                                                //Going ""BACKWARD""
      rightMotor.setSpeed(abs(PWM_rightMotor));
      rightMotor.run("BACKWARD");
    }
    
    if((millis()-lastMilli) >= LOOPTIME){         //write an error if execution time of the loop in longer than the specified looptime
      Serial.println(" TOO LONG ");
    }

    noCommLoops++;
    if (noCommLoops == 65535){
      noCommLoops = noCommLoopMax;
    }
   publishSpeed(LOOPTIME);
 }
}
void publishSpeed(double time) {
    xicro.Spin_node();
    xicro.Publisher_ack_encoder_vel.message.vector.x = speed_act_right;
    xicro.Publisher_ack_encoder_vel.message.vector.y = speed_act_left;
    xicro.Publisher_ack_encoder_vel.message.vector.z = ang_act_servo;
    xicro.publish_ack_encoder_vel();
    xicro.Publisher_ack_encoder_tick.message.vector.x = pos_right;
    xicro.Publisher_ack_encoder_tick.message.vector.y = pos_left;
    xicro.publish_ack_encoder_tick();
    xicro.Publisher_ack_imu.message.vector.x = mpu.getAngleX();
    xicro.Publisher_ack_imu.message.vector.y = mpu.getAngleY();
    xicro.Publisher_ack_imu.message.vector.z = mpu.getAngleZ();
    xicro.publish_ack_imu();
    xicro.Publisher_ack_speed_req.message.vector.x = speed_req_left;
    xicro.Publisher_ack_speed_req.message.vector.y = speed_req_right;
    xicro.Publisher_ack_speed_req.message.vector.z = ang_req_servo;
    xicro.publish_ack_speed_req();
    xicro.Publisher_ack_speed_cmd.message.vector.x = speed_cmd_left;
    xicro.Publisher_ack_speed_cmd.message.vector.y = speed_cmd_right;
    xicro.Publisher_ack_speed_cmd.message.vector.z = ang_cmd_servo;
    xicro.publish_ack_speed_cmd();
  
    xicro.Publisher_ack_PWM_cmd.message.vector.x = PWM_leftMotor;
    xicro.Publisher_ack_PWM_cmd.message.vector.y = PWM_rightMotor;
          
    xicro.publish_ack_PWM_cmd();
//  xicro.loginfo("Publishing odometry");
}

void encoderLeftMotor() {
  if (digitalRead(PIN_ENCOD_A_MOTOR_LEFT) == digitalRead(PIN_ENCOD_B_MOTOR_LEFT)) pos_left--;
  else pos_left++;
}

//Right motor encoder counter
void encoderRightMotor() {
  if (digitalRead(PIN_ENCOD_A_MOTOR_RIGHT) == digitalRead(PIN_ENCOD_B_MOTOR_RIGHT)) pos_right++;
  else pos_right--;
}

void encoderRightServo() {
  if (digitalRead(PIN_ENCOD_A_SERVO_RIGHT) == digitalRead(PIN_ENCOD_B_SERVO_RIGHT)) pos_servo_right++;
  else pos_servo_right--;
}

void encoderLeftServo() {
  if (digitalRead(PIN_ENCOD_A_SERVO_RIGHT) == digitalRead(PIN_ENCOD_B_SERVO_RIGHT)) pos_servo_left++;
  else pos_servo_left--;
}
void Steertoservo(){
  if (steer_cmd < 0) {
  servo_cmd += steer_cmd;
} else if (steer_cmd > 0) {
  servo_cmd -= steer_cmd;
}
  
  }


template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
