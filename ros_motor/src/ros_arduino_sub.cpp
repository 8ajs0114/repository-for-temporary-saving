#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

void msgCallback(const geometry_msgs::Twist &msg)  //const 상수
{
    ROS_INFO("Encoder A : %f, Encoder B : %f, Encoder S : %f", msg.angular.x,msg.angular.y,msg.angular.z);
    ROS_INFO("Tick : %f, Distance : %f", msg.linear.x,msg.linear.y);	
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_arduino_sub");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("encoder_pulse",100,msgCallback);
    ros::spin(); //어떤 값이 들어오기 전까지 대기 (다시 위로 올라감)
    return 0;
}
