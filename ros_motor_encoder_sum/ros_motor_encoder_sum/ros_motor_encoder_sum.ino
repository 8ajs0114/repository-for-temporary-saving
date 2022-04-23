//ros
#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

//R-MOTOR
#define ENABLE_A  11   
#define IN1_A     9 
#define IN2_A     10 

//L-MOTOR
#define ENABLE_B  6
#define IN3_B     4
#define IN4_B     5

//R-ENCODER
#define encoderAPinA 8
#define encoderAPinB 7

//L-ENCODER
#define encoderBPinA 3
#define encoderBPinB 2

float encoderAPos = 0;
float encoderBPos = 0;

ros::NodeHandle  nh;
geometry_msgs::Twist encoder_msg;
ros::Publisher pub_encoder("encoder_pulse", &encoder_msg);

float velocity = 0;

void MOTOR( const geometry_msgs::Twist& msg)
{  
  //motor
  if(msg.linear.x == -1) //w
  {
    digitalWrite(IN1_A, HIGH);  
    digitalWrite(IN2_A, LOW);
    
    digitalWrite(IN3_B, LOW);  
    digitalWrite(IN4_B, HIGH);
    
    velocity = msg.angular.x;
  }

  else if(msg.linear.x == 1) //s
  {
    digitalWrite(IN1_A, LOW);  
    digitalWrite(IN2_A, HIGH);
    
    digitalWrite(IN3_B, HIGH);  
    digitalWrite(IN4_B, LOW);
    
    velocity = msg.angular.x;
  }
  
  else if(msg.linear.y == -1) //a
  {
    digitalWrite(IN1_A, HIGH);  
    digitalWrite(IN2_A, LOW);
   
    digitalWrite(IN3_B, HIGH);  
    digitalWrite(IN4_B, LOW);
    
    velocity = msg.angular.y;
  }
  
  else if(msg.linear.y == 1) //d
  {
    digitalWrite(IN1_A, LOW);  
    digitalWrite(IN2_A, HIGH);
    
    digitalWrite(IN3_B, LOW);  
    digitalWrite(IN4_B, HIGH);
    velocity = msg.angular.y;
  }
  
  else if(msg.linear.x == msg.linear.y)
    velocity = 0;
  
  else;
  
  analogWrite(ENABLE_A, velocity);
  analogWrite(ENABLE_B, velocity);
  
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &MOTOR);

void setup()
{
  //R-MOTOR
  pinMode(IN1_A, OUTPUT); // 방향1
  pinMode(IN2_A, OUTPUT); // 방향2
  pinMode(ENABLE_A, OUTPUT); // 속도

  //L-MOTOR
  pinMode(IN3_B, OUTPUT); // 방향3
  pinMode(IN4_B, OUTPUT); // 방향4
  pinMode(ENABLE_B, OUTPUT); // 속도

  //R-ENCODER
  pinMode(encoderAPinA, INPUT); 
  pinMode(encoderAPinB, INPUT); 
  attachInterrupt(4, doEncoderAA, CHANGE); // encoder pin on interrupt 4 (pin 4)
  attachInterrupt(3, doEncoderAB, CHANGE); // encoder pin on interrupt 3 (pin 3)
 
  //L-ENCODER
  pinMode(encoderBPinA, INPUT); 
  pinMode(encoderBPinB, INPUT); 
  attachInterrupt(0, doEncoderBA, CHANGE); // encoder pin on interrupt 0 (pin 0)
  attachInterrupt(1, doEncoderBB, CHANGE); // encoder pin on interrupt 1 (pin 1)

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub_encoder);
}

void loop()
{  
  encoder_msg.angular.x = encoderAPos;
  encoder_msg.angular.y = encoderBPos;
  pub_encoder.publish(&encoder_msg);
  nh.spinOnce();
  delay(1);
}

void doEncoderAA(){
  // look for a low-to-high on channel A
  if (digitalRead(encoderAPinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderAPinB) == LOW) {  
      encoderAPos = encoderAPos + 1;
    } 
    else {
      encoderAPos = encoderAPos - 1;
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoderAPinB) == HIGH) {   
      encoderAPos = encoderAPos + 1;
    } 
    else {
      encoderAPos = encoderAPos - 1;
    }
  }
//  Serial.println (encoderAPos);
}

void doEncoderAB(){
  // look for a low-to-high on channel B
  if (digitalRead(encoderAPinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoderAPinA) == HIGH) {  
      encoderAPos = encoderAPos + 1;
    } 
    else {
      encoderAPos = encoderAPos - 1;
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoderAPinA) == LOW) {   
      encoderAPos = encoderAPos + 1;
    } 
    else {
      encoderAPos = encoderAPos - 1;
    }
  }
//  Serial.println (encoderAPos);
}

void doEncoderBA(){
  // look for a low-to-high on channel A
  if (digitalRead(encoderBPinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderBPinB) == LOW) {  
      encoderBPos = encoderBPos + 1;
    } 
    else {
      encoderBPos = encoderBPos - 1;
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoderBPinB) == HIGH) {   
      encoderBPos = encoderBPos + 1;
    } 
    else {
      encoderBPos = encoderBPos - 1;
    }
  }
//  Serial.println (encoderBPos);
}

void doEncoderBB(){
  // look for a low-to-high on channel B
  if (digitalRead(encoderBPinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoderBPinA) == HIGH) {  
      encoderBPos = encoderBPos + 1;
    } 
    else {
      encoderBPos = encoderBPos - 1;
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoderBPinA) == LOW) {   
      encoderBPos = encoderBPos + 1;
    } 
    else {
      encoderBPos = encoderBPos - 1;
    }
  }
//  Serial.println (encoderBPos);
}
