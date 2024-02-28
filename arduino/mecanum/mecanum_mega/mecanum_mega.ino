#include <PID_v1.h>
#include <Xicro_mec_ID_3.h>
#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
Xicro xicro;

//initializing all the variables
#define LOOPTIME                      100  //Looptime in millisecond 200 ms = 5Hz   33 ms = 30Hz
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter
  

#define PWM_motor1 6 //2
#define PWM_motor2 4
#define PWM_motor3 12
#define PWM_motor4 13

#define Direction_motor1 14 //3
#define Direction_motor2 5
#define Direction_motor3 30
#define Direction_motor4 32

// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define encoder_encoder1A 2 //21
#define encoder_encoder2A 3 //20
#define encoder_encoder3A 19
#define encoder_encoder4A 18

// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define encoder_encoder1B 22
#define encoder_encoder2B 2
#define encoder_encoder3B 26
#define encoder_encoder4B 28

// True = Forward; False = Reverse
boolean dir1 = true;
boolean dir2 = true;
boolean dir3 = true;
boolean dir4 = true;

int dirreal1 = 0 ;
int dirreal2 = 0 ;
int dirreal3 = 0 ;
int dirreal4 = 0 ;

const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

// Keep track of the number of wheel ticks
volatile int counter1 = 0;
volatile int counter2 = 0;
volatile int counter3 = 0;
volatile int counter4 = 0;

volatile int lastpos1 = 0 ;
volatile int lastpos2 = 0 ;
volatile int lastpos3 = 0 ;
volatile int lastpos4 = 0 ;

unsigned long lastMilli = 0;

//--- Robot-specific constants ---
const double radius = 0.076;                   //Wheel radius, in m
const double wheelbase = 0.55;               //Wheelbase, in m
const double encoder_cpr = 360;               //Encoder ticks or counts per rotation
const double speed_to_pwm_ratio = 0.0383;    // 0.00235 Ratio to convert speed (in m/s) to PWM value. It was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the slope of the linear function).
const double min_speed_cmd = 0.09;          //0.0882 (min_speed_cmd/speed_to_pwm_ratio) is the minimum command value needed for the motor to start moving. This value was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the constant of the linear function).

double linx = 0;
double liny = 0;
double linz = 0;
double angx = 0;
double angy = 0;
double angz = 0;                 //Desired angular speed for the robot, in rad/s

double speed_req_motor1 = 0;                    //Desired speed for left wheel in m/s
double speed_act_motor1 = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_motor1 = 0;                    //Command speed for left wheel in m/s 

double speed_req_motor2 = 0;                    //Desired speed for left wheel in m/s
double speed_act_motor2 = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_motor2 = 0;                    //Command speed for left wheel in m/s 

double speed_req_motor3 = 0;                    //Desired speed for left wheel in m/s
double speed_act_motor3 = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_motor3 = 0;                    //Command speed for left wheel in m/s 

double speed_req_motor4 = 0;                    //Desired speed for left wheel in m/s
double speed_act_motor4 = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_motor4 = 0;                    //Command speed for left wheel in m/s 

                        
const double max_speed = 0.4;                 //Max speed in m/s

int PWM1 = 0;                     //PWM command for left motor
int PWM2 = 0;                    //PWM command for right motor 
int PWM3 = 0;                     //PWM command for left motor
int PWM4 = 0;                    //PWM command for right motor 
              
//PID

int PWM_Motor1 = 0;
int PWM_Motor2 = 0;
int PWM_Motor3 = 0;
int PWM_Motor4 = 0;


// PID Parameters

const double PID_param[] = { 0.4, 0, 0.001 }; //Respectively Kp, Ki and Kd for left motor PID
PID PID_Motor1(&speed_act_motor1,&speed_cmd_motor1, &speed_req_motor1, PID_param[0], PID_param[1], PID_param[2], DIRECT);          //Setting up the PID for left motor
PID PID_Motor2(&speed_act_motor2,&speed_cmd_motor2, &speed_req_motor2, PID_param[0], PID_param[1], PID_param[2], DIRECT);          //Setting up the PID for left motor
PID PID_Motor3(&speed_act_motor3,&speed_cmd_motor3, &speed_req_motor3, PID_param[0], PID_param[1], PID_param[2], DIRECT);          //Setting up the PID for left motor
PID PID_Motor4(&speed_act_motor4,&speed_cmd_motor4, &speed_req_motor4, PID_param[0], PID_param[1], PID_param[2], DIRECT);          //Setting up the PID for left motor

class motor{
    public:
        int pinA;
        int pinPWM;
    
    void motor_setup(){
        pinMode(pinA,OUTPUT);
        pinMode(pinPWM,OUTPUT);
    }

    void setSpeed(const int speedInput){
        analogWrite(pinPWM, speedInput);
    }

    void run(const String runInput){
       if(runInput == "BRAKE"){
            digitalWrite(pinA, HIGH);
            analogWrite(pinPWM, 0);
       }
       else if(runInput == "FORWARD"){
            digitalWrite(pinA, HIGH);
       }
       else if(runInput == "BACKWARD"){
            digitalWrite(pinA, LOW);
       }
    }
};

void handle_cmd() {
  noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication
  linx = xicro.Subscription_cmd_vel_follower.message.linear.x;
  liny = xicro.Subscription_cmd_vel_follower.message.linear.y;
  linz = xicro.Subscription_cmd_vel_follower.message.linear.z;
  angx = xicro.Subscription_cmd_vel_follower.message.angular.x;
  angy = xicro.Subscription_cmd_vel_follower.message.angular.y;
  angz = xicro.Subscription_cmd_vel_follower.message.angular.z;

    speed_req_motor1 = (linx-liny-(wheelbase*angz));
    speed_req_motor2 = (linx+liny+(wheelbase*angz));
    speed_req_motor3 = (linx+liny-(wheelbase*angz));
    speed_req_motor4 = (linx-liny+(wheelbase*angz));
    
//  speed_req = xicro.Subscription_cmd_vel.message.linear.x;          //Extract the commanded linear speed from the message
//
//  angular_speed_req = xicro.Subscription_cmd_vel.message.angular.z; //Extract the commanded angular speed from the message
//  
//  speed_req_left = speed_req - angular_speed_req*(wheelbase/2);     //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
//  speed_req_right = speed_req + angular_speed_req*(wheelbase/2);    //Calculate the required speed for the right motor to comply with commanded linear and angular speeds
      
}

motor motor1, motor2 ,motor3 ,motor4;

void setup() {
  // Set the motor control pins as outputs
  Wire.begin();
  byte status = mpu.begin();
  mpu.calcOffsets(); // gyro and accelero

  Serial.begin(115200);
  xicro.begin(&Serial);

  Serial.println("Setup complete");
  
  motor1.pinA = Direction_motor1;
  motor1.pinPWM = PWM_motor1;  
  motor1.motor_setup();

  motor2.pinA = Direction_motor2;
  motor2.pinPWM = PWM_motor2;  
  motor2.motor_setup();

  motor3.pinA = Direction_motor3;
  motor3.pinPWM = PWM_motor3;  
  motor3.motor_setup();

  motor4.pinA = Direction_motor4;
  motor4.pinPWM = PWM_motor4;  
  motor4.motor_setup();

  //set PID
  PID_Motor1.SetSampleTime(95);
  PID_Motor1.SetOutputLimits(-max_speed, max_speed);
  PID_Motor1.SetMode(AUTOMATIC);

  PID_Motor2.SetSampleTime(95);
  PID_Motor2.SetOutputLimits(-max_speed, max_speed);
  PID_Motor2.SetMode(AUTOMATIC);

  PID_Motor3.SetSampleTime(95);
  PID_Motor3.SetOutputLimits(-max_speed, max_speed);
  PID_Motor3.SetMode(AUTOMATIC);

  PID_Motor4.SetSampleTime(95);
  PID_Motor4.SetOutputLimits(-max_speed, max_speed);
  PID_Motor4.SetMode(AUTOMATIC);

  
  //setting motor speeds to zero
  motor1.setSpeed(0);
  motor1.run("BRAKE");
  motor2.setSpeed(0);
  motor2.run("BRAKE");
  motor3.setSpeed(0);
  motor3.run("BRAKE");
  motor4.setSpeed(0);
  motor4.run("BRAKE");
  //setting PID parameters
   
  pinMode(encoder_encoder1A , INPUT_PULLUP);
  pinMode(encoder_encoder1B , INPUT_PULLUP); 
  pinMode(encoder_encoder2A , INPUT_PULLUP);
  pinMode(encoder_encoder2B , INPUT_PULLUP);
  pinMode(encoder_encoder3A , INPUT_PULLUP);
  pinMode(encoder_encoder3B , INPUT_PULLUP); 
  pinMode(encoder_encoder4A , INPUT_PULLUP);
  pinMode(encoder_encoder4B , INPUT_PULLUP);
  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(encoder_encoder1A), encoder1_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder_encoder2A), encoder2_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder_encoder3A), encoder3_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder_encoder4A), encoder4_tick, RISING);
}

void loop() {
    
    xicro.Spin_node();     
  if((millis()-lastMilli) >= LOOPTIME)   
  {                                                                           // enter timed loop
    lastMilli = millis();
    handle_cmd();
    byte status = mpu.begin();

    

//    xicro.Publisher_IMU.message.vector.y = int(mpu.getAngleZ())%360;
    
    if(abs(counter1 - lastpos1) < 5 )
    {
      speed_act_motor1 = 0;
    }
    else
    {
      speed_act_motor1 = ((((counter1-lastpos1)/encoder_cpr)*2*PI)*(1000/LOOPTIME)*radius);
      lastpos1 = counter1 ;
    }

    if(abs(counter2 - lastpos2) < 5 )
    {
      speed_act_motor2 = 0;
    }
    else
    {
      speed_act_motor2 = ((((counter2-lastpos2)/encoder_cpr)*2*PI)*(1000/LOOPTIME)*radius);
      lastpos2 = counter2 ;
    }

    if(abs(counter3 - lastpos3) < 5 )
    {
      speed_act_motor3 = 0;
    }
    else
    {
      speed_act_motor3 = ((((counter3-lastpos3)/encoder_cpr)*2*PI)*(1000/LOOPTIME)*radius);
      lastpos3 = counter3 ;
    }

    if(abs(counter4 - lastpos4) < 5 )
    {
      speed_act_motor4 = 0;
    }
    else
    {
      speed_act_motor4 = ((((counter4-lastpos4)/encoder_cpr)*2*PI)*(1000/LOOPTIME)*radius);
      lastpos4 = counter4 ;
    }
    
//    pos_left = 0;
//    pos_right = 0;
    
    speed_cmd_motor1 = constrain(speed_cmd_motor1, -max_speed, max_speed);
    speed_cmd_motor2 = constrain(speed_cmd_motor2, -max_speed, max_speed);
    speed_cmd_motor3 = constrain(speed_cmd_motor3, -max_speed, max_speed);
    speed_cmd_motor4 = constrain(speed_cmd_motor4, -max_speed, max_speed);
    PID_Motor1.Compute();
    PID_Motor2.Compute();
    PID_Motor3.Compute();
    PID_Motor4.Compute();                                                 
    // compute PWM value for left motor. Check constant definition comments for more information.
    PWM_Motor1 = constrain(speed_req_motor1*(255/0.29), -255, 255); 
    PWM_Motor2 = constrain(speed_req_motor2*(255/0.29), -255, 255);
    PWM_Motor3 = constrain(speed_req_motor3*(255/0.29), -255, 255);
    PWM_Motor4 = constrain(speed_req_motor4*(255/0.29), -255, 255);
    
    if(PWM_Motor1 >= 0) // forward left
    {
      motor1.run("FORWARD");
    }
    else if(PWM_Motor1 < 0)
    {
      motor1.run("BACKWARD");
      PWM_Motor1 = PWM_Motor1*(-1);
    }

    if(PWM_Motor2 >= 0) // forward left
    {
      motor2.run("FORWARD");
    }
    else if(PWM_Motor2 < 0)
    {
      motor2.run("BACKWARD");
      PWM_Motor2 = PWM_Motor2*(-1);
    }

    if(PWM_Motor3 >= 0) // forward left
    {
      motor3.run("FORWARD");
    }
    else if(PWM_Motor3 < 0)
    {
      motor3.run("BACKWARD");
      PWM_Motor3 = PWM_Motor3*(-1);
    }

    if(PWM_Motor4 >= 0) // forward left
    {
      motor4.run("FORWARD");
    }
    else if(PWM_Motor4 < 0)
    {
      motor4.run("BACKWARD");
      PWM_Motor4 = PWM_Motor4*(-1);
    }     

//
//    PWMout.linear.x = PWM_Motor1 ;
//    PWMout.linear.y = PWM_Motor2 ;
//    PWMout.angular.x = PWM_Motor3 ;
//    PWMout.angular.y = PWM_Motor4 ;

    motor1.setSpeed(PWM_Motor1);
    motor2.setSpeed(PWM_Motor2);
    motor3.setSpeed(PWM_Motor3);
    motor4.setSpeed(PWM_Motor4);
    noCommLoops++;
    
    if (noCommLoops == 65535){
      noCommLoops = noCommLoopMax;
    }
    xicro.publish_mec_encoder_vel();
   publishSpeed(LOOPTIME);
  
 }
}
void publishSpeed(double time) {
//  xicro.Publisher_encode.message.header.stamp = nh.now;
    handle_cmd();
    IMUbringup();
  xicro.Publisher_mec_encoder_tick.message.linear.x = counter1;
  xicro.Publisher_mec_encoder_tick.message.linear.y = counter2;
  xicro.Publisher_mec_encoder_tick.message.angular.x = counter3;
  xicro.Publisher_mec_encoder_tick.message.angular.y = counter4;
  xicro.Publisher_mec_encoder_vel.message.linear.x = speed_act_motor1;
  xicro.Publisher_mec_encoder_vel.message.linear.y = speed_act_motor2;
  xicro.Publisher_mec_encoder_vel.message.angular.x = speed_act_motor3;
  xicro.Publisher_mec_encoder_vel.message.angular.y = speed_act_motor4;
  xicro.publish_mec_encoder_vel();
  xicro.publish_mec_encoder_tick();
  xicro.publish_mec_imu();
  xicro.publish_mec_imu_raw();
  
//  xicro.Publisher_mec_speed_req.message.vector.x = speed_req_right;
//  xicro.Publisher_mec_speed_req.message.vector.y = speed_req_left;
        
  xicro.publish_mec_speed_req();
    xicro.publish_mec_speed_cmd();
  
//    xicro.Publisher_mec_PWM_cmd.message.vector.x = PWM_rightMotor;
//    xicro.Publisher_mec_PWM_cmd.message.vector.y = PWM_leftMotor;
          
    xicro.publish_mec_PWM_cmd();
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
    xicro.Publisher_mec_imu.message.linear.x = mpu.getAngleX();
    xicro.Publisher_mec_imu.message.linear.y = mpu.getAngleY();
    xicro.Publisher_mec_imu.message.angular.z = mpu.getAngleZ();
    xicro.Publisher_mec_imu_raw.message.header.frame_id = "Imu";

    xicro.Publisher_mec_imu_raw.message.linear_acceleration.x = mpu.getAccX();
    xicro.Publisher_mec_imu_raw.message.linear_acceleration.y = mpu.getAccY();
    xicro.Publisher_mec_imu_raw.message.linear_acceleration.z = mpu.getAccZ();
    xicro.Publisher_mec_imu_raw.message.angular_velocity.x = mpu.getGyroX();
    xicro.Publisher_mec_imu_raw.message.angular_velocity.y = mpu.getGyroY();
    xicro.Publisher_mec_imu_raw.message.angular_velocity.z = mpu.getGyroZ();
//    xicro.Publisher_mec_imu_raw.message.orientation_covariance[0] = 0.01;
//    xicro.Publisher_mec_imu_raw.message.orientation_covariance[4] = 0.01;
//    xicro.Publisher_mec_imu_raw.message.orientation_covariance[8] = 0.01;
//    
//    xicro.Publisher_mec_imu_raw.message.linear_acceleration_covariance[0] = 0.01;
//    xicro.Publisher_mec_imu_raw.message.linear_acceleration_covariance[4] = 0.01;
//    xicro.Publisher_mec_imu_raw.message.linear_acceleration_covariance[8] = 0.01;
//    
//    xicro.Publisher_mec_imu_raw.message.angular_velocity_covariance[0] = 0.01;
//    xicro.Publisher_mec_imu_raw.message.angular_velocity_covariance[4] = 0.01;
//    xicro.Publisher_mec_imu_raw.message.angular_velocity_covariance[8] = 0.01;

    float roll = mpu.getAngleX();
    float pitch = mpu.getAngleY();
    float yaw = mpu.getAngleZ();
    float w, x, y, z;
    eulerToQuaternion(roll, pitch, yaw, w, x, y, z);
    
    xicro.Publisher_mec_imu_raw.message.orientation.w = w;
    xicro.Publisher_mec_imu_raw.message.orientation.x = x;
    xicro.Publisher_mec_imu_raw.message.orientation.y = y;
    xicro.Publisher_mec_imu_raw.message.orientation.z = z;
  
  }

void encoder1_tick() 
{

  // Read the value for the encoder for the right wheel
  int val = digitalRead(encoder_encoder1B);

  if(val == LOW) 
  {
    dir1 = false; // Reverse
  }
  else 
  {
    dir1 = true; // Forward
  }

  if (dir1) 
  {

    if (counter1 == encoder_maximum) 
    {
      counter1 = encoder_minimum;
    }
    else 
    {
      counter1++;  
    }    
  }
  else 
  {
    if (counter1 == encoder_minimum) 
    {   
      counter1 = encoder_maximum;
    }
    else 
    {
      counter1--;
    }   
  }
}   
void encoder2_tick() 
{

  // Read the value for the encoder for the right wheel
  int val2 = digitalRead(encoder_encoder2B);

  if(val2 == LOW) 
  {
    dir2 = false; // Reverse
  }
  else 
  {
    dir2 = true; // Forward
  }

  if (dir2) 
  {

    if (counter2 == encoder_maximum) 
    {
      counter2 = encoder_minimum;
    }
    else 
    {
      counter2++;  
    }    
  }
  else 
  {
    if (counter2 == encoder_minimum) 
    {   
      counter2 = encoder_maximum;
    }
    else 
    {
      counter2--;
    }   
  } 

}
void encoder3_tick() 
{

  // Read the value for the encoder for the right wheel
  int val3 = digitalRead(encoder_encoder3B);

  if(val3 == LOW) 
  {
    dir3 = false; // Reverse
  }
  else 
  {
    dir3 = true; // Forward
  }

  if (dir3) 
  {

    if (counter3 == encoder_maximum) 
    {
      counter3 = encoder_minimum;
    }
    else 
    {
      counter3++;  
    }    
  }
  else 
  {
    if (counter3 == encoder_minimum) 
    {   
      counter3 = encoder_maximum;
    }
    else 
    {
      counter3--;
    }   
  } 

}
void encoder4_tick() 
{

  // Read the value for the encoder for the right wheel
  int val4 = digitalRead(encoder_encoder4B);

  if(val4 == LOW) 
  {
    dir4 = false; // Reverse
  }
  else 
  {
    dir4 = true; // Forward
  }

  if (dir4) 
  {

    if (counter4 == encoder_maximum) 
    {
      counter4 = encoder_minimum;
    }
    else 
    {
      counter4++;  
    }    
  }
  else 
  {
    if (counter4 == encoder_minimum) 
    {   
      counter4 = encoder_maximum;
    }
    else 
    {
      counter4--;
    }   
  } 

}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
