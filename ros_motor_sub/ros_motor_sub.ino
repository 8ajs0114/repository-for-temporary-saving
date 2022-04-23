//ros

#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

//motor
#define ENABLE_A  10  
#define IN1_A  9   
#define IN2_A  8   
 
ros::NodeHandle  nh;

void MOTOR( const geometry_msgs::Twist& msg)
{
  digitalWrite(IN1_A, HIGH);  
  digitalWrite(IN2_A, LOW);  
  analogWrite(ENABLE_A, msg.linear.x); 
}

ros::Subscriber<geometry_msgs::Twist> sub("basic_topic", &MOTOR);

void setup()
{
  pinMode(IN1_A, OUTPUT); // 방향1
  pinMode(IN2_A, OUTPUT); // 방향2
  pinMode(ENABLE_A, OUTPUT); // 속도

  nh.initNode();
  nh.subscribe(sub);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
