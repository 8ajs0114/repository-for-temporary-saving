
// race_code
// for Mega 2560

//ros
#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
//#include <nav_msgs/Odometry.h>
#include <MsTimer2.h>
#include <Wire.h>
#include "math.h"

ros::NodeHandle  nh;
geometry_msgs::Twist encoder_msg;
//nav_msgs::Odometry encoder_msg;
ros::Publisher pub_encoder("encoder_pulse", &encoder_msg);

//R-MOTOR
#define PWM_B  3   //Yellow
#define IN_B   4   //Orange

//L-MOTOR
#define PWM_A  6  //Purple
#define IN_A   5  //Blue  

//L-ENCODER
#define encoderAPinA 18 //Brown
#define encoderAPinB 19 //White

//R-ENCODERss
#define encoderBPinA 20 //Gray
#define encoderBPinB 21 //Purple

// ROBOT DESCRIPTION
#define MAX_PWM 255
#define MAX_RPM 285
#define DISTANCE_CENTER_TO_WHEEL 24.0
#define WHEEL_DIAMETER 115
#define PULSE_PER_CYCLE 76
#define MOTOR_ENCODER_GEAR_RATIO 12
#define TICK_PER_CYCLE 912 // PULSE_PER_CYCLE * MOTOR_ENCODER_GEAR_RATIO
#define TICK_PER_MAX_RPM_IN_MINUTE 259920// MAX_RPM * TICK_PER_CYCLE

//PID CONTROL
#define P_GAIN 0.2 
#define D_GAIN 2.0

// ENCODER VARIABLE
int encoderAPos = 0;
int encoderBPos = 0;
int average_encoder = 0;

// INPUT VARIABLE 
float velocity_input = 0; 
float angle_input = 0;

// CALCULATION
float velocity_gain = 0;
float anlge_gain = 0;
float L_Velocity_target = 0;
float L_Velocity_present = 0;
float L_Encoder_target = 0;
float L_error = 0;
float R_Velocity_target = 0;
float R_Velocity_present = 0;
float R_Encoder_target = 0;
float R_error = 0;

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
void Encoder_reset(void);

void MOTOR (const geometry_msgs::Twist& msg)
{
    // GET DATA
    velocity_input = msg.linear.x;
    angle_input = msg.angular.z;

    // SET DATA SAFETY
    if(angle_input > 0.25)
       angle_input = 0.25;
    else if(angle_input < -0.25)
       angle_input = -0.25;
    else;
    
    if(velocity_input > 0.1)
      velocity_input = 0.1;
    else if(velocity_input < -0.1)
      velocity_input =- 0.1;  
    else;  
      
    // SET DATA GAIN
    velocity_gain = 400*velocity_input;
    anlge_gain = 5*angle_input;
   
    L_Velocity_target = velocity_gain - (anlge_gain * DISTANCE_CENTER_TO_WHEEL);
    L_Velocity_target = abs(L_Velocity_target);
    R_Velocity_target = velocity_gain + (anlge_gain * DISTANCE_CENTER_TO_WHEEL);
    R_Velocity_target = abs(R_Velocity_target);     
   
    L_Velocity_present = L_Velocity_target;
    R_Velocity_present = R_Velocity_target; // 치우치면 좀 더해서 넣어준다.
    
    L_Encoder_target = (((L_Velocity_target * MAX_RPM) / MAX_PWM) * TICK_PER_CYCLE) / 12000;
    R_Encoder_target = (((R_Velocity_target * MAX_RPM) / MAX_PWM) * TICK_PER_CYCLE) / 12000; //6000 => 1m->60s / 1ms->0.001s

    // CONTROL FIELD
    L_error = encoderAPos - L_Encoder_target;
    L_error = abs(L_error);
    R_error = encoderBPos - R_Encoder_target;
    R_error = abs(R_error);
    
    if(encoderBPos < R_Encoder_target)
      R_Velocity_target = R_Velocity_present + D_GAIN * R_error * P_GAIN;      
    else if(encoderBPos > R_Encoder_target)
      R_Velocity_target = R_Velocity_present - D_GAIN * R_error * P_GAIN;
    else;   
    
    if(encoderAPos > L_Encoder_target)
      L_Velocity_target = L_Velocity_present - D_GAIN * L_error * P_GAIN;
    else if(encoderAPos < L_Encoder_target)
      L_Velocity_target = L_Velocity_present + D_GAIN * L_error * P_GAIN;
    else ;

    if(velocity_input > 0) //front
      Motor_Forward();
    else if(velocity_input < 0) //back
      Motor_Backward();
    else;
    
    if(velocity_input == 0)
    {
      if(angle_input > 0) //right
        Motor_Rightward();
      else if(angle_input < 0) //left
        Motor_Leftward();
      else if(angle_input == 0) //stop
        STOP();
      else;
    }
    else;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &MOTOR);

void setup()
{
  //L-MOTOR
  pinMode(IN_A, OUTPUT); // 방향L
  pinMode(PWM_A, OUTPUT); // 속도

  //R-MOTOR
  pinMode(IN_B, OUTPUT); // 방향R
  pinMode(PWM_B, OUTPUT); // 속도

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

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub_encoder);
}

void loop()
{
//  encoder_msg.linear.x = velocity_input;
//  encoder_msg.linear.y = angle_input;
//  encoder_msg.angular.x = L_Velocity_target;
//  encoder_msg.angular.y = R_Velocity_target;
 
//  encoder_msg.pose.pose.linear.y = encoderAPos;
//  encoder_msg.pose.pose.linear.x = encoderBPos;

//  encoder_msg.linear.y = encoderAPos;
//  encoder_msg.linear.x = encoderBPos;
  
  average_encoder = ( encoderAPos + encoderBPos ) / 2;
//  encoder_msg.pose.pose.angular.z = average_encoder;
  encoder_msg.linear.z = average_encoder;
  
  pub_encoder.publish(&encoder_msg);

  encoderAPos = 0;
  encoderBPos = 0;
  average_encoder = 0;
  
  nh.spinOnce();
  delay(10);
}

void Motor_Forward ()
{
  digitalWrite(IN_A, HIGH);    
  digitalWrite(IN_B, LOW);  
  
  analogWrite(PWM_A, L_Velocity_target);
  analogWrite(PWM_B, R_Velocity_target);
}

void Motor_Backward ()
{
  digitalWrite(IN_A, LOW);  
  digitalWrite(IN_B, HIGH);  

  analogWrite(PWM_A, L_Velocity_target);
  analogWrite(PWM_B, R_Velocity_target);
}

void Motor_Leftward ()
{
  digitalWrite(IN_A, LOW);  
  digitalWrite(IN_B, LOW); 

  analogWrite(PWM_A, L_Velocity_target);
  analogWrite(PWM_B, R_Velocity_target);  
}

void Motor_Rightward ()
{
  digitalWrite(IN_A, HIGH);  
  digitalWrite(IN_B, HIGH); 

  analogWrite(PWM_A, L_Velocity_target);
  analogWrite(PWM_B, R_Velocity_target);  
}

void STOP(void)
{
  digitalWrite(IN_A, HIGH);  
  digitalWrite(IN_B, HIGH); 

  analogWrite(PWM_A, 0);
  analogWrite(PWM_B, 0);  
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
