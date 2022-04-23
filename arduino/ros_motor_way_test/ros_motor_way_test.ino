//ros
#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

//R - motor
#define ENABLE_A  11   
#define IN1_A     9 
#define IN2_A     10 

//L - motor
#define ENABLE_B  6
#define IN3_B     4
#define IN4_B     5

ros::NodeHandle  nh;

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
  pinMode(IN1_A, OUTPUT); // 방향1
  pinMode(IN2_A, OUTPUT); // 방향2
  pinMode(ENABLE_A, OUTPUT); // 속도
  
  pinMode(IN3_B, OUTPUT); // 방향3
  pinMode(IN4_B, OUTPUT); // 방향4
  pinMode(ENABLE_B, OUTPUT); // 속도

  nh.initNode();
  nh.subscribe(sub);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
