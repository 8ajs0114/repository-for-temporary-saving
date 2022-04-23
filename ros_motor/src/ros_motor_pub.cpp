#include <ros/ros.h>  //ROS 기본 헤더파일
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
 
int main(int argc, char **argv) //c++ 의 기본 함수형태
{
    ros::init(argc, argv, "ros_motor_pub"); //노드명 초기화
    ros::NodeHandle nh; //ROS시스템과 통신을 위한 노드핸들 선언
 
 
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);
    //퍼블리시어 선언, 패키지 ()의 메시지파일()을 이용
     //퍼블리시어()을 작성
    //토픽명은 ()이며 퍼블리시어큐 사이즈를 ()개로 설정한다.
    //publisher이기 때문에 advertise 만듬
    
    ros::Rate loop_rate(20); //루프 주기를 2Hz로 설정 (1초에 2번)
    
    geometry_msgs::Twist msg; //내가 만든 메시지 파일 형식으로 ()라는 메세지선언
    
    float count = 0;
 
    while(ros::ok()) //ros 가 활성화되면
    {
	    msg.angular.x =  127.1263365;
	    msg.angular.y =  37.3208234;

	    msg.linear.x =   127.1263555;
	    msg.linear.y = 37.3208367;


	    ROS_INFO("passing latitude = %f, passing longtitude = %f", msg.angular.x, msg.angular.y );
	    ROS_INFO("following latitude = %f, following longtitude = %f", msg.linear.x, msg.linear.y );	 

	    pub.publish(msg);
	    loop_rate.sleep();
	

	
    }
    return 0;
}

