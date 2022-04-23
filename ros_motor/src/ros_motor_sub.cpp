#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

void msgCallback(const geometry_msgs::Twist &msg)  //const 상수
{
    ROS_INFO("receive msg l.x = %f", msg.linear.x);
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_motor_sub");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("ros_motor_topic",100,msgCallback);
    ros::spin(); //어떤 값이 들어오기 전까지 대기 (다시 위로 올라감)
    return 0;
}
