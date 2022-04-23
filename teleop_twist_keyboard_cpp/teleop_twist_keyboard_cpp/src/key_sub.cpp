#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

void response(const geometry_msgs::Twist::ConstPtr& msg)
{
	ROS_INFO("x = %f",msg->linear.x);
	ROS_INFO("y = %f",msg->linear.y);
	ROS_INFO("speed = %f",msg->angular.x);
	ROS_INFO("turn = %f",msg->angular.y);

	ROS_INFO("=======================");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "key_sub");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("cmd_vel",100,response);
	ros::spin();
	return 0;
}
