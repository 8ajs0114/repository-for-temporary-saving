#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <map>

// Map for movement keys
std::map<char, std::vector<float>> moveBindings
{
  {'w', { 1,  0,  0}},
  {'a', { 0, -1,  0}},
  {'d', { 0,  1,  0}},
  {'s', {-1,  0,  0}}	
};

// Map for speed keys
std::map<char, std::vector<float>> speedBindings
{
  {'i', { 0.5,  0}},
  {'o', {-0.5,  0}},
  {'k', { 0,  1.0}},
  {'l', { 0, -1.0}}
};

// Reminder message
const char* msg = R"(

Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
        w    
   a    s    d

velocity
i : up   (+0.5) m/s
o : down (-0.5) m/s

turn weight
k : up   (+1,0) rad/s
l : down (-1,0) rad/s

anything else : stop

CTRL-C to quit

)";

// Init variables
float velocity(0.5); // Linear velocity (m/s)
float turn(1.0); // Angular velocity (rad/s)
float x(0), y(0), z(0), th(0); // Forward/backward/neutral direction vars
char key(' ');

// For non-blocking keyboard inputs
int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

int main(int argc, char** argv)
{
  // Init ROS node
  ros::init(argc, argv, "teleop_twist_keyboard");
  ros::NodeHandle nh;

  // Init cmd_vel publisher
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  // Create Twist message
  geometry_msgs::Twist twist;

  printf("%s", msg);
  printf("\rCurrent: velocity %f\tturn %f | Awaiting command...\r", velocity, turn);

  while(true)
{
    // Get the pressed key
    key = getch();

    // If the key corresponds to a key in moveBindings
    if (moveBindings.count(key) == 1)
    {
      // Grab the direction data
      x = moveBindings[key][0];
      y = moveBindings[key][1];
      z = moveBindings[key][2];
    }

    // Otherwise if it corresponds to a key in speedBindings
    else if (speedBindings.count(key) == 1)
    {
      // Grab the speed data
      velocity = velocity + speedBindings[key][0];
      turn = turn + speedBindings[key][1];
    }

    // Otherwise, set the robot to stop
    else
    {
      velocity = 0;
      turn = 0;
      
      // If ctrl-C (^C) was pressed, terminate the program
      if (key == '\x03')
        break;
    }
    
    printf("\rCurrent: velocity %f m/s\tturn %f rad/s\tLast command: %c   ", velocity, turn, key);
    
    // Update the Twist message
    twist.linear.x = x;
    twist.linear.y = y;
    twist.linear.z = 0;

    twist.angular.x = velocity;
    twist.angular.y = turn;
    twist.angular.z = 0;

    // Publish it and resolve any remaining callbacks
    pub.publish(twist);
    ros::spinOnce();
  }

  return 0;
}
