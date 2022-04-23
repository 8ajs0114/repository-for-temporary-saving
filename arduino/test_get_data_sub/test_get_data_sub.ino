#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>
#include <std_msgs/Int32.h>
#include <stdio.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

ros::NodeHandle  nh;

void print_data(const geometry_msgs::Twist& msg)
{  
  if(msg.linear.x >= 80)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
  }
/*
  else if(msg.linear.x < 180)
  { 
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
  }
  */
}

ros::Subscriber<geometry_msgs::Twist> sub("ros_motor_topic",print_data );

void setup() 
{
  pinMode(LED_BUILTIN, OUTPUT);
  
  nh.initNode();
  nh.subscribe(sub);
}



void loop() 
{
  nh.spinOnce();
  delay(10);

}
