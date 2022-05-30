// move_base_input_code
// for Mega 2560

//ros
#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <MsTimer2.h>

ros::NodeHandle  nh;
geometry_msgs::Twist encoder_msg;
ros::Publisher pub_encoder("encoder_pulse", &encoder_msg);

//R-MOTOR
#define ENABLE_B  4  //Orange 
#define IN3_B     2   //Green
#define IN4_B     3   //Yellow

//L-MOTOR
#define ENABLE_A  7  //Gray
#define IN1_A     5  //Blue
#define IN2_A     6  //Purple

//L-ENCODER
#define encoderAPinA 18 //Brown
#define encoderAPinB 19 //White

//R-ENCODER
#define encoderBPinA 20 //Gray
#define encoderBPinB 21 //Purple

#define rad(x) (x*3.141592)/180.0
#define WHEEL_DIAMETER 115
#define PULSE_PER_CYCLE 76
#define MOTOR_ENCODER_GEAR_RATIO 12
#define INTERPOLATION 4
#define DISTANCE_CENTER_TO_WHEEL 240

//VELOCITY CONTROL
#define MAX_PWM 600
#define MAX_RPM 285 //per minute
#define WHEEL_RADIUS 57.5 //mm

//PID CONTROL
#define P_GAIN 0.7
#define I_GAIN 0.0002 
#define D_GAIN 3.5
#define POSE_GAIN 0.2
#define TIME 1.5

//PID VARIABLE 
float velocity_input = 0; 
float angle_input = 0;
int interrupt_count = 0;

typedef struct 
{  
  double target, 
         current, 
         pControl, 
         iControl, 
         dControl, 
         pidControl, 
         realError, 
         accError, 
         errorGap;
         
     int move_pwm;
}PID_BLOCK;

PID_BLOCK PID_R;
PID_BLOCK PID_L;


// ENCODER VARIABLE
int encoderAPos = 0;
int encoderBPos = 0;
int average_encoder = 0;

//MOTOR WAY FUNCTION
void Motor_Forward (void);
void Motor_Backward (void);
void Motor_Rightward (void);
void Motor_Leftward (void);
void STOP(void);

//MOTOER INTERRUPT FUNCTION
void doEncoderAA(void); 
void doEncoderAB(void);
void doEncoderBA(void);
void doEncoderBB(void);
void Encoder_Count(void);

//PID CONTROL FUNCTION
void calculateErrors(double target, double current, double realError, double accError, double errorGap); 
double pControlSystem(double realError);
double iControlSystem(double accError); 
double dControlSystem(double errorGap); 
double pidControlSystem(double pControl, double iControl, double dControl); 

int Calculate_Velocity_to_PWM(float input_velocity);

void MOTOR (const geometry_msgs::Twist& msg)
{
  velocity_input = msg.linear.x; //m/s
  angle_input = (-1) * msg.angular.z; //rad/s

  if(velocity_input >= 0.2) //모터 최대 출력이 1.76 m/s
    velocity_input = 0.2;

  else if(velocity_input <= -0.2)
    velocity_input = -0.2;

  else;

  if(angle_input >= 0.1)
    angle_input = 0.1;

  else if(angle_input <= -0.1)
    angle_input = -0.1;

  else;
      
  PID_R.current = (( 3.14159 * WHEEL_DIAMETER )/ PULSE_PER_CYCLE / 2 ) * encoderAPos / 0.1 / 1000;
  PID_L.current = (( 3.14159 * WHEEL_DIAMETER )/ PULSE_PER_CYCLE / 2) * encoderBPos / 0.1 / 1000;
  
  PID_R.target = abs(velocity_input - (angle_input * POSE_GAIN));
  PID_L.target = abs(velocity_input + (angle_input * POSE_GAIN));  

// 
  PID_R.realError = PID_R.target - PID_R.current; 
  PID_R.errorGap = PID_R.target - PID_R.current - PID_R.realError; 
  PID_R.accError += PID_R.realError;

  PID_L.realError = PID_L.target - PID_L.current; 
  PID_L.errorGap = PID_L.target - PID_L.current - PID_L.realError; 
  PID_L.accError += PID_L.realError;

  PID_R.pControl = pControlSystem(PID_R.realError);
  PID_L.pControl = pControlSystem(PID_L.realError);

  PID_R.iControl = iControlSystem(PID_R.accError); 
  PID_L.iControl = iControlSystem(PID_L.accError);
  
  PID_R.dControl = dControlSystem(PID_R.errorGap); 
  PID_L.dControl = dControlSystem(PID_L.errorGap); 
  
  PID_R.pidControl = pidControlSystem(PID_R.pControl, PID_R.iControl, PID_R.dControl); 
  PID_L.pidControl = pidControlSystem(PID_L.pControl, PID_L.iControl, PID_L.dControl);

//

  PID_R.move_pwm = Calculate_Velocity_to_PWM(PID_R.pidControl);
  PID_L.move_pwm = Calculate_Velocity_to_PWM(PID_L.pidControl);

//  PID_R.move_pwm = Calculate_Velocity_to_PWM(0);
//  PID_L.move_pwm = Calculate_Velocity_to_PWM(0);

  if(velocity_input > 0)
    Motor_Forward ();

  else if(velocity_input < 0)
    Motor_Backward();

  else;
  
  if(velocity_input == 0 && angle_input < 0)
    Motor_Rightward();
    
  else if(velocity_input == 0 && angle_input > 0)
    Motor_Leftward(); 

  else if(velocity_input == 0 && angle_input == 0)
    STOP();  

  else;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &MOTOR);

void setup()
{
  //L-MOTOR
  pinMode(IN1_A, OUTPUT); // 방향1
  pinMode(IN2_A, OUTPUT); // 방향2
  pinMode(ENABLE_A, OUTPUT); // 속도

  //R-MOTOR
  pinMode(IN3_B, OUTPUT); // 방향3
  pinMode(IN4_B, OUTPUT); // 방향4
  pinMode(ENABLE_B, OUTPUT); // 속도

  //L-ENCODER
  pinMode(encoderAPinA, INPUT); 
  pinMode(encoderAPinB, INPUT); 
  attachInterrupt(5, doEncoderAA, CHANGE); // encoder pin on interrupt 3 (pin 18)
  attachInterrupt(4, doEncoderAB, CHANGE); // encoder pin on interrupt 2 (pin 19)
 
  //R-ENCODER
  pinMode(encoderBPinA, INPUT); 
  pinMode(encoderBPinB, INPUT); 
  attachInterrupt(3, doEncoderBA, CHANGE); // encoder pin on interrupt 1 (pin 20)
  attachInterrupt(2, doEncoderBB, CHANGE); // encoder pin on interrupt 0 (pin 21)

  MsTimer2::set(10,Encoder_Count);
  MsTimer2::start();
  

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub_encoder);
}

void loop()
{  
//  encoder_msg.angular.x = PID_R.pidControl;
//  encoder_msg.angular.y = PID_L.pidControl;

  
  encoder_msg.angular.x = encoderAPos;
  encoder_msg.angular.y = encoderBPos;
  
  average_encoder = ( encoderAPos + encoderBPos ) / 2;
  encoder_msg.angular.z = average_encoder;
  
  pub_encoder.publish(&encoder_msg);
  
  nh.spinOnce();
  delay(1);
}

int Calculate_Velocity_to_PWM(float input_velocity) //not complete. if something error happens then rewire 
{
  float distance_per_once_wheel_move = 0;
  float velocity_per_second = 0; //m/m
  int output_pwm = 0;

  if(input_velocity >= 0.7)
    input_velocity = 0.7;
  
  distance_per_once_wheel_move = 2 * 3.141592 * WHEEL_RADIUS; //mm
  velocity_per_second = ((MAX_RPM * distance_per_once_wheel_move) / 1000 ) / 60; //m/s
  
  output_pwm = (input_velocity * MAX_PWM) / velocity_per_second;

  return output_pwm;
}

/////////////////////////////////////

void Motor_Forward ()
{
  digitalWrite(IN1_A, LOW);  
  digitalWrite(IN2_A, HIGH);
  
  digitalWrite(IN3_B, HIGH);  
  digitalWrite(IN4_B, LOW);
  
  analogWrite(ENABLE_A, PID_R.move_pwm);
  analogWrite(ENABLE_B, PID_L.move_pwm);
}

void Motor_Backward ()
{
  digitalWrite(IN1_A, HIGH);  
  digitalWrite(IN2_A, LOW);
  
  digitalWrite(IN3_B, LOW);  
  digitalWrite(IN4_B, HIGH);

  analogWrite(ENABLE_A, PID_R.move_pwm);
  analogWrite(ENABLE_B, PID_L.move_pwm);
}

void Motor_Leftward ()
{
  digitalWrite(IN1_A, HIGH);  
  digitalWrite(IN2_A, LOW);
  
  digitalWrite(IN3_B, HIGH);  
  digitalWrite(IN4_B, LOW);

  analogWrite(ENABLE_A, PID_R.move_pwm);
  analogWrite(ENABLE_B, PID_L.move_pwm);  
}

void Motor_Rightward ()
{
  digitalWrite(IN1_A, LOW);  
  digitalWrite(IN2_A, HIGH);
 
  digitalWrite(IN3_B, LOW);  
  digitalWrite(IN4_B, HIGH);

  analogWrite(ENABLE_A, PID_R.move_pwm);
  analogWrite(ENABLE_B, PID_L.move_pwm);  
}

void STOP(void)
{
  digitalWrite(IN1_A, LOW);  
  digitalWrite(IN2_A, LOW);
 
  digitalWrite(IN3_B, LOW);  
  digitalWrite(IN4_B, LOW);

  analogWrite(ENABLE_A, 0);
  analogWrite(ENABLE_B, 0);  
}

/////////////////////////////////////

void calculateErrors(double target, double current, double realError, double accError, double errorGap) 
{ 
  realError = target - current; 
  errorGap = target - current - realError; 
  accError += realError; 
}

double pControlSystem(double realError) 
{ 
  double pControl = P_GAIN * realError; 
  
  return pControl; 
}

double iControlSystem(double accError) 
{ 
  double iControl = I_GAIN * (accError * TIME); 
  
  return iControl; 
}

double dControlSystem(double errorGap) 
{ 
  double dControl = D_GAIN * (errorGap / TIME); 
  
  return dControl; 
}

double pidControlSystem(double pControl, double iControl, double dControl) 
{ 
  double pidControl = pControl + iControl + dControl; 
  
  return pidControl; 
}

/////////////////////////////////////

void Encoder_Count(void)
{
  encoderAPos = 0;
  encoderBPos = 0;
  average_encoder = 0;

  interrupt_count ++;

  if(interrupt_count >= 30)
  {
    PID_R.accError = 0;
    PID_L.accError = 0;

    interrupt_count = 0;
  }
}

void doEncoderAA(void) 
{
  if (digitalRead(encoderAPinA) == HIGH) // look for a low-to-high on channel A
  { 
    if (digitalRead(encoderAPinB) == LOW) // check channel B to see which way encoder is turning
      encoderAPos++ ;

    else 
      encoderAPos-- ;
  }
  
  else   // must be a high-to-low edge on channel A                                       
  { 
    if (digitalRead(encoderAPinB) == HIGH) // check channel B to see which way encoder is turning
      encoderAPos++ ;  
      
    else 
      encoderAPos-- ;
  }
}

void doEncoderAB(void)
{ 
  if (digitalRead(encoderAPinB) == HIGH) // look for a low-to-high on channel B
  {
    if (digitalRead(encoderAPinA) == HIGH) // check channel A to see which way encoder is turning  
      encoderAPos++ ;
    
    else 
      encoderAPos-- ;
  }
  
  else // Look for a high-to-low on channel B
  { 
    if (digitalRead(encoderAPinA) == LOW) // check channel B to see which way encoder is turning  
      encoderAPos++ ;
     
    else 
      encoderAPos-- ;
  }
}

void doEncoderBA(void)
{
  
  if (digitalRead(encoderBPinA) == HIGH) // look for a low-to-high on channel A
  { 
    if (digitalRead(encoderBPinB) == LOW) // check channel B to see which way encoder is turning  
      encoderBPos-- ;
    
    else 
      encoderBPos++ ;
    
  }
  
  else // must be a high-to-low edge on channel A                                       
  { 
    
    if (digitalRead(encoderBPinB) == HIGH) // check channel B to see which way encoder is turning    
      encoderBPos-- ;
     
    else 
      encoderBPos++ ;   
  }
}

void doEncoderBB(void)
{
  
  if (digitalRead(encoderBPinB) == HIGH) // look for a low-to-high on channel B  
  { 
    if (digitalRead(encoderBPinA) == HIGH) // check channel A to see which way encoder is turning
      encoderBPos-- ;
     
    else 
      encoderBPos++ ;
    
  }
  
  else // Look for a high-to-low on channel B
  { 
     
    if (digitalRead(encoderBPinA) == LOW) // check channel B to see which way encoder is turning 
      encoderBPos-- ;
     
    else 
      encoderBPos++ ;
  }
}
