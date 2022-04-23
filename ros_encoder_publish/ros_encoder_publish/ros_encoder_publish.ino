#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

//R-MOTOR
#define encoderAPinA 8
#define encoderAPinB 7

//L-MOTOR
#define encoderBPinA 3
#define encoderBPinB 2

float encoderAPos = 0;
float encoderBPos = 0;

ros::NodeHandle  nh;
geometry_msgs::Twist encoder_msg;
ros::Publisher pub_encoder("encoder_pulse", &encoder_msg);

void setup() 
{
  pinMode(encoderAPinA, INPUT); 
  pinMode(encoderAPinB, INPUT); 
  attachInterrupt(4, doEncoderAA, CHANGE); // encoder pin on interrupt 4 (pin 4)
  attachInterrupt(3, doEncoderAB, CHANGE); // encoder pin on interrupt 3 (pin 3)
  pinMode(encoderBPinA, INPUT); 
  pinMode(encoderBPinB, INPUT); 
  attachInterrupt(0, doEncoderBA, CHANGE); // encoder pin on interrupt 0 (pin 0)
  attachInterrupt(1, doEncoderBB, CHANGE); // encoder pin on interrupt 1 (pin 1)

  nh.initNode();
  nh.advertise(pub_encoder);
}

void loop()
{ 
  encoder_msg.angular.x = encoderAPos;
  encoder_msg.angular.y = encoderBPos;
  pub_encoder.publish(&encoder_msg);
  nh.spinOnce();
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
