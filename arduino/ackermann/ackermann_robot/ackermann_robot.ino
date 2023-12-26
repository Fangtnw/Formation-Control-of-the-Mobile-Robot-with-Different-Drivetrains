#include <Servo.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <PID_v1.h>

//initializing all the variables
#define LOOPTIME                      100     //Looptime in millisecond
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter

double speed_cmd_left2 = 0;   
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
int motor_spd = 30;
int motor_state=0;
int servo_pos;

//Encoder
const int PIN_ENCOD_A_MOTOR_LEFT = 2;               //A channel for encoder of left motor                    
const int PIN_ENCOD_B_MOTOR_LEFT = 8;               //B channel for encoder of left motor

const int PIN_ENCOD_A_MOTOR_RIGHT = 3;         //A channel for encoder of right motor         
const int PIN_ENCOD_B_MOTOR_RIGHT = 9;         //B channel for encoder of right motor 

const int PIN_ENCOD_A_SERVO = 18;              //A channel for encoder of right motor         
const int PIN_ENCOD_B_SERVO = 23;              //B channel for encoder of right motor 

//Define Value
const double radius = 0.06;                   //Wheel radius, in m
const double wheelbase = 0.58;               //Wheelbase, in m
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

int PWM_leftMotor = 0;                     //PWM command for left motor
int PWM_rightMotor = 0;                    //PWM command for right motor 

//PID Parameters
const double PID_left_param[] = { 1, 0, 0 }; //Respectively Kp, Ki and Kd for left motor PID
const double PID_right_param[] = { 1, 0, 0 }; //Respectively Kp, Ki and Kd for right motor PID

volatile float pos_left = 0;       //Left motor encoder position
volatile float pos_right = 0;      //Right motor encoder position
volatile float pos_servo = 0;      //Servo motor encoder position

//const unsigned long print_interval = 1000; // 1 second
//unsigned long previous_print_time = 0;

Servo servo;

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
    servo.write(servo_pos);
};

void handle_cmd() {
  noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication
  
  speed_req = xicro.Subscription_cmd_vel.message.linear.x;          //Extract the commanded linear speed from the message

  angular_speed_req = xicro.Subscription_cmd_vel.message.angular.z; //Extract the commanded angular speed from the message
  
  speed_req_left = speed_req - angular_speed_req*(wheelbase/2);     //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
  speed_req_right = speed_req + angular_speed_req*(wheelbase/2);    //Calculate the required speed for the right motor to comply with commanded linear and angular speeds

//ros::NodeHandle nh;
//std_msgs::String combined_msg;
//ros::Publisher combined_pub("combined", &combined_msg);

//void cmd_vel_callback(const geometry_msgs::Twist& twist_msg) {
  //int motor_state = twist_msg.linear.x;
  //int motor_spd = twist_msg.linear.y;
  //int servo_pos = twist_msg.angular.z;
  //unsigned long current_time = millis();

  //if (current_time - previous_print_time >= print_interval) {
    //previous_print_time = current_time;

    // Create the combined message
    //char combined_buffer[50];
    //snprintf(combined_buffer, sizeof(combined_buffer), "motor spd: %d servo_pos: %d", motor_spd, servo_pos);
    //combined_msg.data = combined_buffer;

    // Publish the combined message
    //combined_pub.publish(&combined_msg);
  }
  // Set the motor directions and speeds
  if (motor_state == 1) {
    analogWrite(pwm1, motor_spd);
    digitalWrite(motor1_pin1, HIGH);
    digitalWrite(motor1_pin2, LOW);
    analogWrite(pwm2, motor_spd+6);
    digitalWrite(motor2_pin1, HIGH);
    digitalWrite(motor2_pin2, LOW);
    key_pressed = true;
  } else if (motor_state == -1) {
    analogWrite(pwm1, motor_spd);
    digitalWrite(motor1_pin1, LOW);
    digitalWrite(motor1_pin2, HIGH);
    analogWrite(pwm2, motor_spd+6);
    digitalWrite(motor2_pin1, LOW);
    digitalWrite(motor2_pin2, HIGH);
    key_pressed = true;
  } else if (motor_state == 0) {
    analogWrite(pwm1, 0);
    digitalWrite(motor1_pin1, HIGH);
    digitalWrite(motor1_pin2, HIGH);
    analogWrite(pwm2, 0);
    digitalWrite(motor2_pin1, HIGH);
    digitalWrite(motor2_pin2, HIGH);
    key_pressed = false;
  }

  servo.write(servo_pos);
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmd_vel_callback);


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
  servo.attach(servoIN);

  erial.begin(115200);
  xicro.begin(&Serial);

  
  Serial.println("Setup complete");
  
  float linearX = xicro.Subscription_cmd_vel.message.linear.x;

  leftMotor.pinA = 12;
  leftMotor.pinB = 11;
  leftMotor.pinPWM = 10;
  rightMotor.pinA = 6;
  rightMotor.pinB = 7;
  rightMotor.pinPWM = 5;
  servo.pin = 13
  
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

  //Define the rotary encoder for servo motor
  pinMode(PIN_ENCOD_A_SERVO, INPUT);
  pinMode(PIN_ENCOD_B_SERVO, INPUT);
  digitalWrite(PIN_ENCOD_A_SERVO, HIGH);                // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_SERVO, HIGH);
  attachInterrupt(1, encoderServoMotor, RISING);

  //nh.initNode();
  //nh.subscribe(cmd_vel_sub);
  //nh.advertise(combined_pub);
}

void loop() {
  xicro.Spin_node();
  handle_cmd();
  if((millis()-lastMilli) >= LOOPTIME)   
  {                                                                           // enter timed loop
    lastMilli = millis();
    
    byte status = mpu.begin();
    
    xicro.Publisher_IMU.message.vector.x = mpu.getAngleX();
    xicro.Publisher_IMU.message.vector.y = mpu.getAngleY();
    xicro.Publisher_IMU.message.vector.z = mpu.getAngleZ();
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
    
    xicro.Publisher_encoder_tick.message.vector.x = pos_right;
    xicro.Publisher_encoder_tick.message.vector.y = pos_left;
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
//  xicro.Publisher_encode.message.header.stamp = nh.now;
  xicro.Publisher_encoder_vel.message.vector.x = speed_act_right;
  xicro.Publisher_encoder_vel.message.vector.y = speed_act_left;
  xicro.Publisher_encoder_vel.message.vector.z = time/1000;
  xicro.publish_encoder_vel();
  xicro.publish_encoder_tick();
  xicro.publish_IMU();
  xicro.Spin_node();
  xicro.Publisher_speed_req.message.vector.x = speed_req_left;
  xicro.Publisher_speed_req.message.vector.y = speed_req_right;
        
  xicro.publish_speed_req();

    xicro.Publisher_speed_cmd.message.vector.x = speed_cmd_left;
    xicro.Publisher_speed_cmd.message.vector.y = speed_cmd_right;
          
    xicro.publish_speed_cmd();
  
    xicro.Publisher_PWM_cmd.message.vector.x = PWM_leftMotor;
    xicro.Publisher_PWM_cmd.message.vector.y = PWM_rightMotor;
          
    xicro.publish_PWM_cmd();
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

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
