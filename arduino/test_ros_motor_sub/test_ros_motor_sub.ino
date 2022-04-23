//ros
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>
#include <std_msgs/UInt16.h>

//motor
#define ENABLE_A  10  
#define IN1_A  9   
#define IN2_A  8   

 
ros::NodeHandle  nh;

void dc_l298n( const std_msgs::UInt16& cmd_msg)
{
  digitalWrite(IN1_A, HIGH);  //
  digitalWrite(IN2_A, LOW);  //
  analogWrite(ENABLE_A, cmd_msg.data); //set motor speed, should be from 0-255  
    
}


ros::Subscriber<std_msgs::UInt16> sub("dc_l298n", dc_l298n);

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
