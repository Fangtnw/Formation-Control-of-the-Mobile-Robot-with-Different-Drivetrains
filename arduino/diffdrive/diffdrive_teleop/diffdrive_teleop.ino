#include <PID_v1.h>
#include <Xicro_diffdrive_ID_1.h>
#include <Wire.h>
#include <Servo.h>
#include <MPU6050_light.h>

 MPU6050 mpu(Wire);
Servo myservo; 
Xicro xicro;

//initializing all the variables
#define LOOPTIME                      100    //Looptime in millisecond
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter
  
unsigned long lastMilli = 0;

// Motor 1
const int motor1INA = 12;
const int motor1INB = 11;
const int motor1PWM = 10;

// Motor 2
const int motor2INA = 6;
const int motor2INB = 7;
const int motor2PWM = 5;

const int PIN_ENCOD_A_MOTOR_LEFT = 2;               //A channel for encoder of left motor                    
const int PIN_ENCOD_B_MOTOR_LEFT = 8;               //B channel for encoder of left motor

const int PIN_ENCOD_A_MOTOR_RIGHT = 3;              //A channel for encoder of right motor         
const int PIN_ENCOD_B_MOTOR_RIGHT = 9;              //B channel for encoder of right motor 

const double radius = 0.06;                   //Wheel radius, in m
const double wheelbase = 0.55;               //Wheelbase, in m
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
                        
const double max_speed = 0.4;                 //Max speed in m/s

double speedlinx = 0;
double speedliny = 0;
double speedangz = 0;                 //Desired angular speed for the robot, in rad/s
float ax, ay, az;  
float gx, gy, gz;  
double theta = 0 ;

int PWM_leftMotor = 0;                     //PWM command for left motor
int PWM_rightMotor = 0;                    //PWM command for right motor 
                                                      
// PID Parameters
const double PID_left_param[] = { 1, 0, 0 }; //Respectively Kp, Ki and Kd for left motor PID
const double PID_right_param[] = { 1, 0, 0 }; //Respectively Kp, Ki and Kd for right motor PID

volatile float pos_left = 0;       //Left motor encoder position
volatile float pos_right = 0;      //Right motor encoder position

//PID PID_leftMotor(&speed_act_left, &speed_cmd_left, &speed_req_left, PID_left_param[0], PID_left_param[1], PID_left_param[2], DIRECT);          //Setting up the PID for left motor
//PID PID_rightMotor(&speed_act_right, &speed_cmd_right, &speed_req_right, PID_right_param[0], PID_right_param[1], PID_right_param[2], DIRECT);   //Setting up the PID for right motor

PID PID_leftMotor(&speed_cmd_left,&speed_act_left , &speed_req_left, PID_left_param[0], PID_left_param[1], PID_left_param[2], DIRECT);          //Setting up the PID for left motor
PID PID_rightMotor(&speed_cmd_right,&speed_act_right, &speed_req_right, PID_right_param[0], PID_right_param[1], PID_right_param[2], DIRECT);   //Setting up the PID for right motor

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
};

void handle_cmd() {
  noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication
  
  speed_req = xicro.Subscription_cmd_vel.message.linear.x;          //Extract the commanded linear speed from the message

  angular_speed_req = xicro.Subscription_cmd_vel.message.angular.z; //Extract the commanded angular speed from the message
  
  speed_req_left = speed_req - angular_speed_req*(wheelbase/2);     //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
  speed_req_right = speed_req + angular_speed_req*(wheelbase/2);    //Calculate the required speed for the right motor to comply with commanded linear and angular speeds
      
}

motor leftMotor, rightMotor;

void setup() {
  // Set the motor control pins as outputs
  Wire.begin();
  byte status = mpu.begin();
  mpu.calcOffsets(); // gyro and accelero
  
  pinMode(motor1INA, OUTPUT);
  pinMode(motor1INB, OUTPUT);
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor2INA, OUTPUT);
  pinMode(motor2INB, OUTPUT);
  pinMode(motor2PWM, OUTPUT);

  Serial.begin(115200);
  xicro.begin(&Serial);

  Serial.println("Setup complete");
  
  leftMotor.pinA = 12;
  leftMotor.pinB = 11;
  leftMotor.pinPWM = 10;
  rightMotor.pinA = 6;
  rightMotor.pinB = 7;
  rightMotor.pinPWM = 5;
  
  leftMotor.setSpeed(0);
  leftMotor.run("BRAKE");
  rightMotor.setSpeed(0);
  rightMotor.run("BRAKE");
 
  //setting PID parameters
  PID_leftMotor.SetSampleTime(95);
  PID_rightMotor.SetSampleTime(95);
  PID_leftMotor.SetOutputLimits(-max_speed, max_speed);
  PID_rightMotor.SetOutputLimits(-max_speed, max_speed);
  PID_leftMotor.SetMode(AUTOMATIC);
  PID_rightMotor.SetMode(AUTOMATIC);
    
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
}

void loop() {
  xicro.Spin_node();
  handle_cmd();
  if((millis()-lastMilli) >= LOOPTIME)   
  {                                                                           // enter timed loop
    lastMilli = millis();
    
    byte status = mpu.begin();

    IMUbringup();
    
//    xicro.Publisher_IMU.message.vector.y = int(mpu.getAngleZ())%360;
    
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
    
    xicro.Publisher_diff_encoder_tick.message.vector.x = pos_right;
    xicro.Publisher_diff_encoder_tick.message.vector.y = pos_left;
    pos_left = 0;
    pos_right = 0;
    
//    theta = theta * 180 / 3.141592653589793238462643383279502 ;
//    if(theta > 360)
//    {
//      theta = abs(theta - 360);
//    }
//    xicro.Publisher_encoder_tick.message.vector.z = theta;
//    theta = theta * 3.141592653589793238462643383279502 / 180 ;
    
    speed_cmd_left = constrain(speed_cmd_left, -max_speed, max_speed);
    PID_leftMotor.Compute();                                                 
    // compute PWM value for left motor. Check constant definition comments for more information.
    PWM_leftMotor = constrain(((speed_req_left+sgn(speed_req_left)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_left/speed_to_pwm_ratio), -255, 255); //
    
    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      leftMotor.setSpeed(0);
      leftMotor.run("BRAKE");
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

    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      rightMotor.setSpeed(0);
      rightMotor.run("BRAKE");
    }
    else if (speed_req_right == 0){                       //Stopping
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
      Serial.println("Time out");
    }

    noCommLoops++;
    if (noCommLoops == 65535){
      noCommLoops = noCommLoopMax;
    }

   publishSpeed(LOOPTIME);

 }
}
void publishSpeed(double time) {
//  xicro.Publisher_encode.message.header.stamp = nh.now;
  xicro.Publisher_diff_encoder_vel.message.vector.x = speed_act_right;
  xicro.Publisher_diff_encoder_vel.message.vector.y = speed_act_left;
  xicro.Publisher_diff_encoder_vel.message.vector.z = time/1000;
  xicro.publish_diff_encoder_vel();
  xicro.publish_diff_encoder_tick();
  xicro.publish_diff_imu();
  xicro.publish_diff_imu_raw();
  xicro.Spin_node();
  xicro.Publisher_diff_speed_req.message.vector.x = speed_req_right;
  xicro.Publisher_diff_speed_req.message.vector.y = speed_req_left;
        
  xicro.publish_diff_speed_req();

    xicro.Publisher_diff_speed_cmd.message.vector.x = speed_cmd_right;
    xicro.Publisher_diff_speed_cmd.message.vector.y = speed_cmd_left;
          
    xicro.publish_diff_speed_cmd();
  
    xicro.Publisher_diff_PWM_cmd.message.vector.x = PWM_rightMotor;
    xicro.Publisher_diff_PWM_cmd.message.vector.y = PWM_leftMotor;
          
    xicro.publish_diff_PWM_cmd();
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
    xicro.Publisher_diff_imu.message.vector.x = mpu.getAngleX();
    xicro.Publisher_diff_imu.message.vector.y = mpu.getAngleY();
    xicro.Publisher_diff_imu.message.vector.z = mpu.getAngleZ();


    xicro.Publisher_diff_imu_raw.message.linear_acceleration.x = mpu.getAccX();
    xicro.Publisher_diff_imu_raw.message.linear_acceleration.y = mpu.getAccY();
    xicro.Publisher_diff_imu_raw.message.linear_acceleration.z = mpu.getAccZ();
    xicro.Publisher_diff_imu_raw.message.angular_velocity.x = mpu.getGyroX();
    xicro.Publisher_diff_imu_raw.message.angular_velocity.y = mpu.getGyroY();
    xicro.Publisher_diff_imu_raw.message.angular_velocity.z = mpu.getGyroZ();
    xicro.Publisher_diff_imu_raw.message.orientation_covariance[0] = 0.01;
    xicro.Publisher_diff_imu_raw.message.orientation_covariance[4] = 0.01;
    xicro.Publisher_diff_imu_raw.message.orientation_covariance[8] = 0.01;
    
    xicro.Publisher_diff_imu_raw.message.linear_acceleration_covariance[0] = 0.01;
    xicro.Publisher_diff_imu_raw.message.linear_acceleration_covariance[4] = 0.01;
    xicro.Publisher_diff_imu_raw.message.linear_acceleration_covariance[8] = 0.01;
    
    xicro.Publisher_diff_imu_raw.message.angular_velocity_covariance[0] = 0.01;
    xicro.Publisher_diff_imu_raw.message.angular_velocity_covariance[4] = 0.01;
    xicro.Publisher_diff_imu_raw.message.angular_velocity_covariance[8] = 0.01;

    float roll = mpu.getAngleX();
    float pitch = mpu.getAngleY();
    float yaw = mpu.getAngleZ();
    float w, x, y, z;
    eulerToQuaternion(roll, pitch, yaw, w, x, y, z);
    
    xicro.Publisher_diff_imu_raw.message.orientation.w = w;
    xicro.Publisher_diff_imu_raw.message.orientation.x = x;
    xicro.Publisher_diff_imu_raw.message.orientation.y = y;
    xicro.Publisher_diff_imu_raw.message.orientation.z = z;
      
  
  }

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
