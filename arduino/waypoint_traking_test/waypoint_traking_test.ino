//ros
#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;
geometry_msgs::Twist encoder_msg;
ros::Publisher pub_encoder("encoder_pulse", &encoder_msg);

//R-MOTOR
#define ENABLE_A  11   
#define IN1_A     9 
#define IN2_A     10 

//L-MOTOR
#define ENABLE_B  6
#define IN3_B     4
#define IN4_B     5

//R-ENCODER
#define encoderAPinA 8
#define encoderAPinB 7

//L-ENCODER
#define encoderBPinA 3
#define encoderBPinB 2

#define rad(x) (x*3.141592)/180.0
#define WHEEL_DIAMETER 115
#define PULSE_PER_CYCLE 76
#define MOTOR_ENCODER_GEAR_RATIO 12
#define DISTANCE_CENTER_TO_WHEEL 240
float VELOCITY= 80;


// 좌표 => 거리 => encoder
// 속력 => pwm

float encoderAPos = 0;
float encoderBPos = 0;

struct GPS_points
{
  float x;
  float y;
};

float Point_Distance_By_Harversine_Formula(struct GPS_points pt1, struct GPS_points pt2)
{
  int radius = 6371;

  float Latitude_dist = rad( (pt2.y - pt1.y) ); // 위도차
  float Longitude_dist = rad( (pt2.x - pt1.x) ); // 경도차

  pt1.y = rad(pt1.y);
  pt2.y = rad(pt2.y);
  float a = sin(Latitude_dist/2) * sin(Latitude_dist/2) + sin(Longitude_dist/2) * sin(Longitude_dist/2) * cos(pt1.y) * cos(pt2.y);
  float c = 2 * atan2f(sqrtf(a), sqrtf(1 - a));
  float Point_Distance = radius * c; // killo meter

  Point_Distance *= 1000; //meter
  Point_Distance *= 1000; //milli meter

  return Point_Distance;
}
/*
double Calculate_Angle_to_Distance (double angle_for_turn)
{
  double distance_per_angle = DISTANCE_CENTER_TO_WHEEL * rad(angle_for_turn);

  return distance_per_angle;
}
*/
float Calculate_Distance_to_Tick(float Distance_in_Case)
{
  float distance_per_cycle = WHEEL_DIAMETER * 3.141592;
  float number_of_tick = PULSE_PER_CYCLE * MOTOR_ENCODER_GEAR_RATIO;
  float distance_to_tick = (Distance_in_Case * number_of_tick) / distance_per_cycle;

  return distance_to_tick;
}

void Motor_Forward ()
{
  digitalWrite(IN1_A, HIGH);  
  digitalWrite(IN2_A, LOW);
  
  digitalWrite(IN3_B, LOW);  
  digitalWrite(IN4_B, HIGH);
}

void Motor_Backward ()
{
  digitalWrite(IN1_A, LOW);  
  digitalWrite(IN2_A, HIGH);
  
  digitalWrite(IN3_B, HIGH);  
  digitalWrite(IN4_B, LOW);
}

void Motor_Rightward ()
{
  digitalWrite(IN1_A, LOW);  
  digitalWrite(IN2_A, HIGH);
  
  digitalWrite(IN3_B, LOW);  
  digitalWrite(IN4_B, HIGH);
}

void Motor_Leftward ()
{
  digitalWrite(IN1_A, HIGH);  
  digitalWrite(IN2_A, LOW);
 
  digitalWrite(IN3_B, HIGH);  
  digitalWrite(IN4_B, LOW);
}

void MOTOR (const geometry_msgs::Twist& msg)
{
  float move_distance = 0;
  float move_tick = 0;
  
  struct GPS_points passing_pt; // 현재 좌표
  struct GPS_points following_pt; // 목표 좌표
  
  passing_pt.x = msg.angular.x;
  passing_pt.y = msg.angular.y;
  following_pt.x = msg.linear.x;
  following_pt.y = msg.linear.y;

  // move_distance = (imu_angle >= -5 && imu_angle <= 5) ? Point_Distance_By_Harversine_Formula( passing_pt, following_pt ) : Calculate_Angle_to_Distance (imu_angle);
  move_distance = Point_Distance_By_Harversine_Formula( passing_pt, following_pt );
  move_tick = Calculate_Distance_to_Tick( move_distance );
  
  encoder_msg.linear.x = move_tick;
  encoder_msg.linear.y = move_distance; 
  
  if(encoder_msg.angular.z < move_tick)
  {
    Motor_Forward ();
    
    analogWrite(ENABLE_A, VELOCITY);
    analogWrite(ENABLE_B, VELOCITY);
  }

  else
  {
    analogWrite(ENABLE_A, 0);
    analogWrite(ENABLE_B, 0);
    //encoderAPos = 0;
   // encoderBPos = 0;
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &MOTOR);

void setup()
{
  //R-MOTOR
  pinMode(IN1_A, OUTPUT); // 방향1
  pinMode(IN2_A, OUTPUT); // 방향2
  pinMode(ENABLE_A, OUTPUT); // 속도

  //L-MOTOR
  pinMode(IN3_B, OUTPUT); // 방향3
  pinMode(IN4_B, OUTPUT); // 방향4
  pinMode(ENABLE_B, OUTPUT); // 속도

  //R-ENCODER
  pinMode(encoderAPinA, INPUT); 
  pinMode(encoderAPinB, INPUT); 
  attachInterrupt(4, doEncoderAA, CHANGE); // encoder pin on interrupt 4 (pin 4)
  attachInterrupt(3, doEncoderAB, CHANGE); // encoder pin on interrupt 3 (pin 3)
 
  //L-ENCODER
  pinMode(encoderBPinA, INPUT); 
  pinMode(encoderBPinB, INPUT); 
  attachInterrupt(0, doEncoderBA, CHANGE); // encoder pin on interrupt 0 (pin 0)
  attachInterrupt(1, doEncoderBB, CHANGE); // encoder pin on interrupt 1 (pin 1)

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub_encoder);
}

void loop()
{
  int average_encoder = 0;
  
  encoder_msg.angular.x = encoderAPos;
  encoder_msg.angular.y = encoderBPos;
  average_encoder = ( encoderAPos + encoderBPos ) / 2;
  encoder_msg.angular.z = average_encoder;
  
  pub_encoder.publish(&encoder_msg);
  nh.spinOnce();
  delay(1);
}

void doEncoderAA() 
{
  if (digitalRead(encoderAPinA) == HIGH) // look for a low-to-high on channel A
  { 
    if (digitalRead(encoderAPinB) == LOW) // check channel B to see which way encoder is turning
      encoderAPos++ ;

    else 
      encoderAPos-- ;
  }
  
  else   // must be a high-to-low edge on channel A                                       
  { 
    if (digitalRead(encoderAPinB) == HIGH) // check channel B to see which way encoder is turning
      encoderAPos++ ;  
      
    else 
      encoderAPos-- ;
  }
}

void doEncoderAB()
{ 
  if (digitalRead(encoderAPinB) == HIGH) // look for a low-to-high on channel B
  {
    if (digitalRead(encoderAPinA) == HIGH) // check channel A to see which way encoder is turning  
      encoderAPos++ ;
    
    else 
      encoderAPos-- ;
  }
  
  else // Look for a high-to-low on channel B
  { 
    if (digitalRead(encoderAPinA) == LOW) // check channel B to see which way encoder is turning  
      encoderAPos++ ;
     
    else 
      encoderAPos-- ;
  }
}

void doEncoderBA()
{
  
  if (digitalRead(encoderBPinA) == HIGH) // look for a low-to-high on channel A
  { 
    if (digitalRead(encoderBPinB) == LOW) // check channel B to see which way encoder is turning  
      encoderBPos++ ;
    
    else 
      encoderBPos-- ;
    
  }
  
  else // must be a high-to-low edge on channel A                                       
  { 
    
    if (digitalRead(encoderBPinB) == HIGH) // check channel B to see which way encoder is turning    
      encoderBPos++ ;
     
    else 
      encoderBPos-- ;   
  }
}

void doEncoderBB()
{
  
  if (digitalRead(encoderBPinB) == HIGH) // look for a low-to-high on channel B  
  { 
    if (digitalRead(encoderBPinA) == HIGH) // check channel A to see which way encoder is turning
      encoderBPos++ ;
     
    else 
      encoderBPos-- ;
    
  }
  
  else // Look for a high-to-low on channel B
  { 
     
    if (digitalRead(encoderBPinA) == LOW) // check channel B to see which way encoder is turning 
      encoderBPos ++ ;
     
    else 
      encoderBPos-- ;
  }
}
