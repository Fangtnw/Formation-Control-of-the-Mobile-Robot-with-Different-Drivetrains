#include <PID_v1.h>
#include <Xicro_ack_xicro_ID_2.h>
#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
Xicro xicro;

//initializing all the variables
#define LOOPTIME                      100     //Looptime in millisecond
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter
unsigned long lastMilli = 0;

//Encoder PIN (Require 1 interrupt pin per Encoder -> PIN 2,3,18,19,20,21)
const int PIN_ENCOD_A_MOTOR_LEFT = 17;               //A channel for encoder of left motor                    
const int PIN_ENCOD_B_MOTOR_LEFT = 19;               //B channel for encoder of left motor

const int PIN_ENCOD_A_MOTOR_RIGHT = 16;         //A channel for encoder of right motor         
const int PIN_ENCOD_B_MOTOR_RIGHT = 18;         //B channel for encoder of right motor  

const int PIN_ENCOD_A_STEER_RIGHT = 3;              //A channel for encoder of right servo motor         
const int PIN_ENCOD_B_STEER_RIGHT = 2;              //B channel for encoder of right servo motor 

//Define Value of Parameter
const double radius = 0.06;                     //Wheel radius, in m
const double wheelbase = 0.5;               //Wheelbase, in m
const double wheel_length = 0.5;               //Wheelbase, in m
const double encoder_cpr = 600;               //Encoder ticks or counts per rotation             
const double speed_to_pwm_ratio = 0.00582;    //Ratio to convert speed (in m/s) to PWM value. It was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the slope of the linear function).
const double min_speed_cmd = 0.0882;          //(min_speed_cmd/speed_to_pwm_ratio) is the minimum command value needed for the motor to start moving. This value was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the constant of the linear function).

//Set Value to Zero
double speed_req = 0;                         //Desired linear speed for the robot, in m/s
double angular_speed_req = 0;                 //Desired angular speed for the robot, in rad/s

double speed_req_left = 0;                    //Desired speed for left wheel in m/s
double speed_act_left = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_left = 0;                    //Command speed for left wheel in m/s 

double speed_req_right = 0;                   //Desired speed for right wheel in m/s
double speed_act_right = 0;                   //Actual speed for right wheel in m/s
double speed_cmd_right = 0;                   //Command speed for right wheel in m/s 

double ang_req_steer = 0;                     //Desired speed for steer in m/s
double ang_act_steer = 0;                     //Actual speed for steer
double ang_cmd_steer = 0;                     //Command speed for steer
double ang_desire_steer = 0;                  //Desire angle(deg) for steer
               
double ang_right_calculated = 0;              //Change from encoder tick to degree
double encode_err = 0;

const double max_speed = 0.4;                 //Max speed (m/s)

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
double theta = 0;

int PWM_leftMotor = 0;                     //PWM command for left motor
int PWM_rightMotor = 0;                    //PWM command for right motor 
int PWM_steerMotor = 0;                    //PWM command for steer motor

volatile float pos_left = 0;       //Left motor encoder position
volatile float pos_right = 0;      //Right motor encoder position
volatile float pos_steer = 0;      //Steer motor encoder position
volatile float pos_steer_right = 0;      //Steer motor encoder right position
volatile float pos_ack = 0;
volatile float rad = 0;

const int min_angle = -18;
const int max_angle = 18;

//PID Parameters
const double PID_left_param[] = { 2.2, 5, 0 }; //Respectively Kp, Ki and Kd for left motor PID
const double PID_right_param[] = { 2.2, 5, 0 }; //Respectively Kp, Ki and Kd for right motor PID
const double PID_steer_param[] = { 2.2, 0, 0 }; //Respectively Kp, Ki and Kd for servo motor PID

PID PID_leftMotor(&speed_act_left, &speed_cmd_left, &speed_req_left, PID_left_param[0], PID_left_param[1], PID_left_param[2], DIRECT);          //Setting up the PID for left motor
PID PID_rightMotor(&speed_act_right, &speed_cmd_right, &speed_req_right, PID_right_param[0], PID_right_param[1], PID_right_param[2], DIRECT);   //Setting up the PID for right motor
PID PID_steerMotor(&ang_act_steer, &ang_cmd_steer, &ang_req_steer, PID_steer_param[0], PID_steer_param[1], PID_steer_param[2], DIRECT);   //Setting up the PID for steer motor

//const unsigned long print_interval = 1000; // 1 second
//unsigned long previous_print_time = 0;

class motor{
    public:
        int pinA;
        int pinB;
        int pinPWM;
        int pinPWM_R;
        int pinPWM_L;
    
    void motor_setup(){
        pinMode(pinA,OUTPUT);
        pinMode(pinB,OUTPUT);
        pinMode(pinPWM,OUTPUT);
        pinMode(pinPWM_R,OUTPUT);
        pinMode(pinPWM_L,OUTPUT);
    }

    void setSpeed(const int speedInput){
        analogWrite(pinPWM, speedInput);
//        analogWrite(pinPWM_R, speedInput);  
//        analogWrite(pinPWM_L, speedInput);        
      
    }

    void run(const String runInput){
       if(runInput == "BRAKE"){
            digitalWrite(pinA, HIGH);
            digitalWrite(pinB, HIGH);
//            digitalWrite(pinPWM_R, LOW);
//            digitalWrite(pinPWM_L, LOW);
       }
       else if(runInput == "FORWARD"){
            digitalWrite(pinA, HIGH);
            digitalWrite(pinB, LOW);
//            digitalWrite(pinPWM_R, LOW);
//            digitalWrite(pinPWM_L, HIGH);
       }
       else if(runInput == "BACKWARD"){
            digitalWrite(pinA, LOW);
            digitalWrite(pinB, HIGH);
//            digitalWrite(pinPWM_R, HIGH);
//            digitalWrite(pinPWM_L, LOW);            
       }
    }
    
};

void handle_cmd() {
  noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication
  speed_req = xicro.Subscription_cmd_vel.message.linear.x;          //Extract the commanded linear speed from the message
  angular_speed_req = xicro.Subscription_cmd_vel.message.angular.z; //Extract the commanded angular speed from the message
  
  speed_req_right = speed_req;
  speed_req_left = speed_req;
  ang_req_steer = angular_speed_req;

  //Steer degree Command
  if (speed_req != 0){
    ang_desire_steer = (atan(angular_speed_req*wheel_length)/speed_req)*(180/PI)*0.52; }

    
  else {
    ang_desire_steer = 0;
  }
  
  pos_ack = (abs(pos_steer_right)/encoder_cpr)*360;    //Convert encoder tick to degree value
                  
  }

motor leftMotor, rightMotor,  steerMotor;
void setup() {
  Wire.begin();

  byte status = mpu.begin();
  mpu.calcOffsets(); // gyro and accelero

  //DC MOTOR PIN
  //Left DC
  leftMotor.pinA = 9;
  leftMotor.pinB = 8;
  leftMotor.pinPWM = 10;

  //Right DC
  rightMotor.pinA = 12;
  rightMotor.pinB = 13;
  rightMotor.pinPWM = 11;

  //Steer DC 5,4
  steerMotor.pinPWM_R = 5;
  steerMotor.pinPWM_L = 4;
  
 
  Serial.begin(115200);
  xicro.begin(&Serial);
  Serial.println("Setup complete");
  
  //Set SPD to 0 when start
  leftMotor.setSpeed(0);
  leftMotor.run("BRAKE");
  rightMotor.setSpeed(0);
  rightMotor.run("BRAKE");
  steerMotor.setSpeed(0);
  steerMotor.run("BRAKE");

  //Set DEG to 0 when start
  pos_steer_right = 0;
  
  //setting PID parameters
  PID_leftMotor.SetSampleTime(95);
  PID_rightMotor.SetSampleTime(95);
  PID_steerMotor.SetSampleTime(95);
  PID_leftMotor.SetOutputLimits(-max_speed, max_speed);
  PID_rightMotor.SetOutputLimits(-max_speed, max_speed);
  PID_steerMotor.SetOutputLimits(min_angle, max_angle);

  PID_leftMotor.SetMode(AUTOMATIC);
  PID_rightMotor.SetMode(AUTOMATIC);
  PID_steerMotor.SetMode(AUTOMATIC);
  
  //Define Encoder **Interrupt PIN: 0 = PIN2, 1 = PIN3, 2 = PIN21, 3 = PIN20, 4 = PIN19, 5 = PIN18**
  
  // Define the rotary encoder for left motor
  pinMode(PIN_ENCOD_A_MOTOR_LEFT, INPUT); 
  pinMode(PIN_ENCOD_B_MOTOR_LEFT, INPUT); 
  digitalWrite(PIN_ENCOD_A_MOTOR_LEFT, HIGH);                // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_MOTOR_LEFT, HIGH);
  attachInterrupt(4, encoderLeftMotor, RISING);

  // Define the rotary encoder for right motor
  pinMode(PIN_ENCOD_A_MOTOR_RIGHT, INPUT); 
  pinMode(PIN_ENCOD_B_MOTOR_RIGHT, INPUT); 
  digitalWrite(PIN_ENCOD_A_MOTOR_RIGHT, HIGH);                // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_MOTOR_RIGHT, HIGH);
  attachInterrupt(5, encoderRightMotor, RISING);            
  
  //  Define the rotary encoder for steer
  pinMode(PIN_ENCOD_A_STEER_RIGHT, INPUT);
  pinMode(PIN_ENCOD_B_STEER_RIGHT, INPUT);
  digitalWrite(PIN_ENCOD_A_STEER_RIGHT, HIGH);                // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_STEER_RIGHT, HIGH);
  attachInterrupt(1, encoderRightSteer, RISING);
}

void loop() {
  xicro.Spin_node();
  
  if((millis()-lastMilli) >= LOOPTIME)   
  {                                                                           // enter timed loopf
    lastMilli = millis();
    handle_cmd();
    byte status = mpu.begin();
    
    //PID_servoMotor.Compute();                                               
    if (abs(pos_steer) < 1){                                                   //Avoid taking in account small disturbances
      ang_act_steer = 0;
    }
    else {
      ang_act_steer = pos_steer;             // calculate degree of servo
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
    
    //Left Motor Command    
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

    //Right Motor Command
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

    //Steer Motor Command
//    ang_cmd_steer = constrain(ang_req_steer, -max_speed, max_speed);    
//    PID_steerMotor.Compute();                                                 
//    // compute PWM value for steer motor. Check constant definition comments for more information.
//    PWM_steerMotor = constrain(((ang_req_steer+sgn(ang_req_steer)*min_speed_cmd)/speed_to_pwm_ratio) + (ang_req_steer/speed_to_pwm_ratio), -255, 255); // 
//    encode_err = ang_desire_steer - pos_ack;
//   
//    if (encode_err == 0){                       //Stopping
//      steerMotor.setSpeed(0);
//      steerMotor.run("BRAKE");
//    }
//    else if (encode_err > 0){                         //Going "FORWARD"
//      steerMotor.setSpeed(abs(PWM_steerMotor));
//      steerMotor.run("FORWARD");
//    }
//    else {                                                //Going ""BACKWARD""
//      steerMotor.setSpeed(abs(PWM_steerMotor));
//      steerMotor.run("BACKWARD");
//    }
    
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

void publishSpeed(double time) {
//    xicro.Spin_node();
    IMUbringup();
    handle_cmd();
    xicro.Publisher_ack_encoder_vel.message.angular.x = pos_steer_right;
    xicro.Publisher_ack_encoder_vel.message.angular.y = pos_ack;
    xicro.Publisher_ack_encoder_vel.message.angular.z = ang_req_steer;
    xicro.Publisher_ack_encoder_vel.message.linear.x = speed_act_left;
    xicro.Publisher_ack_encoder_vel.message.linear.y = speed_act_right;
    xicro.publish_ack_encoder_vel();
    xicro.Publisher_ack_encoder_tick.message.linear.x = pos_right;
    xicro.Publisher_ack_encoder_tick.message.linear.y = pos_left;
    xicro.Publisher_ack_encoder_tick.message.angular.x = 0;
    xicro.Publisher_ack_encoder_tick.message.angular.y = pos_steer_right;
    xicro.publish_ack_encoder_tick();
    xicro.Publisher_ack_imu.message.linear.x = mpu.getAngleX();
    xicro.Publisher_ack_imu.message.linear.y = mpu.getAngleY();
    xicro.Publisher_ack_imu.message.linear.z = mpu.getAngleZ();
    xicro.publish_ack_imu();
    xicro.Publisher_ack_speed_req.message.linear.x = speed_req_left;
    xicro.Publisher_ack_speed_req.message.linear.y = speed_req_right;
    xicro.Publisher_ack_speed_req.message.linear.z = ang_req_steer;
    xicro.publish_ack_speed_req();
    xicro.Publisher_ack_speed_cmd.message.linear.x = speed_cmd_left;
    xicro.Publisher_ack_speed_cmd.message.linear.y = speed_cmd_right;
    xicro.Publisher_ack_speed_cmd.message.linear.z = ang_cmd_steer;
    xicro.publish_ack_speed_cmd();
  
    xicro.Publisher_ack_PWM_cmd.message.linear.x = PWM_leftMotor;
    xicro.Publisher_ack_PWM_cmd.message.linear.y = PWM_rightMotor;
          
    xicro.publish_ack_PWM_cmd();
    xicro.publish_ack_imu_raw();
//  xicro.loginfo("Publishing odometry");
}

void eulerToQuaternion(float roll, float pitch, float yaw, float& w, float& x, float& y, float& z) {
    
    // Convert Euler angles to radians
    roll = roll * M_PI / 180.0;
    pitch = pitch * M_PI / 180.0;
    yaw = yaw * M_PI / 180.0;

    // Calculate the quaternion components
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    w = cy * cp * cr + sy * sp * sr;
    x = cy * cp * sr - sy * sp * cr;
    y = sy * cp * sr + cy * sp * cr;
    z = sy * cp * cr - cy * sp * sr;
}

void IMUbringup(){
    mpu.update(); 
    xicro.Publisher_ack_imu.message.linear.x = mpu.getAngleX();
    xicro.Publisher_ack_imu.message.linear.y = mpu.getAngleY();
    xicro.Publisher_ack_imu.message.angular.z = mpu.getAngleZ();
    xicro.Publisher_ack_imu_raw.message.header.frame_id = "Imu_ack";

    xicro.Publisher_ack_imu_raw.message.linear_acceleration.x = mpu.getAccX();
    xicro.Publisher_ack_imu_raw.message.linear_acceleration.y = mpu.getAccY();
    xicro.Publisher_ack_imu_raw.message.linear_acceleration.z = mpu.getAccZ();
    xicro.Publisher_ack_imu_raw.message.angular_velocity.x = mpu.getGyroX();
    xicro.Publisher_ack_imu_raw.message.angular_velocity.y = mpu.getGyroY();
    xicro.Publisher_ack_imu_raw.message.angular_velocity.z = mpu.getGyroZ();
//    xicro.Publisher_ack_imu_raw.message.orientation_covariance[0] = 0.01;
//    xicro.Publisher_ack_imu_raw.message.orientation_covariance[4] = 0.01;
//    xicro.Publisher_ack_imu_raw.message.orientation_covariance[8] = 0.01;
//    
//    xicro.Publisher_ack_imu_raw.message.linear_acceleration_covariance[0] = 0.01;
//    xicro.Publisher_ack_imu_raw.message.linear_acceleration_covariance[4] = 0.01;
//    xicro.Publisher_ack_imu_raw.message.linear_acceleration_covariance[8] = 0.01;
//    
//    xicro.Publisher_ack_imu_raw.message.angular_velocity_covariance[0] = 0.01;
//    xicro.Publisher_ack_imu_raw.message.angular_velocity_covariance[4] = 0.01;
//    xicro.Publisher_ack_imu_raw.message.angular_velocity_covariance[8] = 0.01;

    float roll = mpu.getAngleX();
    float pitch = mpu.getAngleY();
    float yaw = mpu.getAngleZ();
    float w, x, y, z;
    eulerToQuaternion(roll, pitch, yaw, w, x, y, z);
    
    xicro.Publisher_ack_imu_raw.message.orientation.w = w;
    xicro.Publisher_ack_imu_raw.message.orientation.x = x;
    xicro.Publisher_ack_imu_raw.message.orientation.y = y;
    xicro.Publisher_ack_imu_raw.message.orientation.z = z;
    
  
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

//Steer motor encoder
void encoderRightSteer() {
  if (digitalRead(PIN_ENCOD_A_STEER_RIGHT) == digitalRead(PIN_ENCOD_B_STEER_RIGHT)) pos_steer_right--;
  else pos_steer_right++;
}
  
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
