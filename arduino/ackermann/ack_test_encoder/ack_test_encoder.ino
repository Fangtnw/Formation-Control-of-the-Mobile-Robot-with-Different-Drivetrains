// Motor 1
const int motor1INA = 11;
const int motor1INB = 12;
const int motor1PWM = 10;

// Motor 2
const int motor2INA = 7;
const int motor2INB = 6;
const int motor2PWM = 5;



const double radius = 0.06;                   //Wheel radius, in m
const double wheelbase = 0.55;               //Wheelbase, in m
const double encoder_cpr = 600;               //Encoder ticks or counts per rotation

const int PIN_ENCOD_A_MOTOR_LEFT = 2;               //A channel for encoder of left motor                    
const int PIN_ENCOD_B_MOTOR_LEFT = 9;               //B channel for encoder of left motor

const int PIN_ENCOD_A_MOTOR_RIGHT = 5;              //A channel for encoder of right motor         
const int PIN_ENCOD_B_MOTOR_RIGHT = 9;              //B channel for encoder of right motor 


volatile float pos_left = 0;       //Left motor encoder position
volatile float pos_right = 0;      //Right motor encoder position

#define LOOPTIME                      100     //Looptime in millisecond
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter
unsigned long lastMilli = 0;

double speed_req_left = 0;                    //Desired speed for left wheel in m/s
double speed_act_left = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_left = 0;                    //Command speed for left wheel in m/s 

double speed_req_right = 0;                   //Desired speed for right wheel in m/s
double speed_act_right = 0;                   //Actual speed for right wheel in m/s
double speed_cmd_right = 0;                   //Command speed for right wheel in m/s 



void setup() {
  // Set the motor control pins as outputs
  pinMode(motor1INA, OUTPUT);
  pinMode(motor1INB, OUTPUT);
  pinMode(motor1PWM, OUTPUT);

  pinMode(motor2INA, OUTPUT);
  pinMode(motor2INB, OUTPUT);
  pinMode(motor2PWM, OUTPUT);

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
  
  Serial.begin(115200);
}

void loop() {

// moveMotorsForward();


  if((millis()-lastMilli) >= LOOPTIME)   
  {                                                                           // enter timed loop
    lastMilli = millis();
    Serial.println(pos_left);
    Serial.println(speed_act_left);
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

  }
}

void encoderLeftMotor() {
  if (digitalRead(PIN_ENCOD_A_MOTOR_LEFT) == digitalRead(PIN_ENCOD_B_MOTOR_LEFT)) pos_left++;
  else pos_left--;
}

//Right motor encoder counter
void encoderRightMotor() {
  if (digitalRead(PIN_ENCOD_A_MOTOR_RIGHT) == digitalRead(PIN_ENCOD_B_MOTOR_RIGHT)) pos_right++;
  else pos_right--;
}

void moveMotorsForward() {
  // Motor 1
  digitalWrite(motor1INA, HIGH);
  digitalWrite(motor1INB, LOW);
  analogWrite(motor1PWM, 30);  // 255 is the maximum PWM value for full speed

  // Motor 2
  digitalWrite(motor2INA, HIGH);
  digitalWrite(motor2INB, LOW);
  analogWrite(motor2PWM, 30);
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
