#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h> 
#include "math.h"
#include <Wire.h>

#define encoder0PinA 3
#define encoder0PinB 2 
#define encoder1PinA 8
#define encoder1PinB 7 


//double encoderR[5]=0;
//double encoderL[5]=0;
//MPU6050
/*
const int MPU=0x68;//MPU6050 I2C주소
int AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
void get6050();*/
//int a=0;
//double yaw=0.0;
//double straight=0.0;
//double dir=0.0;

int count=0;
//MOTOR
int STBY = 7; //standby
//Motor A
int PWMA = 11; //Speed control
int AIN1 = 9; //Direction
int AIN2 = 10; //Direction
//Motor B
int PWMB = 6; //Speed control
int BIN1 = 4; //Direction
int BIN2 = 5; //Direction
//Encoder_val
int Encoder_value = 0;
int encoderRPos = 0;
int encoderLPos = 0;
double l_enc_w=0;
double r_enc_w=0;
//Moter_val
double vth = 0.0;
double vx = 0.0;
double gain_vx=0.0;
double gain_vth=0.0;
double lv=0;
double rv=0;
double lv_w=0;
double rv_w=0;
double r_error=0;
double l_error=0;

const int WHEEL_DIST = 26.5;


void move(int motor, int speed, int direction);
void doEncoderA();
void doEncoderB();
void doEncoder1A();
void doEncoder1B();

void motor(const geometry_msgs::Twist& msg){
    vx = msg.linear.x;
    vth = msg.angular.z;

    if(vth>0.2)
       vth = 0.15;
    if(vth<-0.2)
       vth = -0.15;

    if(vx>0.1)
      vx=0.1;
      
    /*if(vx==0){
      if(vth>0.2)
        vth= 0.2;
      if(vth<-0.2)
        vth=-0.2;
    }*/
          
    gain_vx=600*vx;
    gain_vth=15*vth;
    
    //510, 25
    //motor0:left, motor1:right
    //1:clock, 0:unclock
    //lv=vx*480-vth*50;
    //rv=vx*480+vth*50;

    //p-control
    /////////when mapping,turn off
   //500, 10 normal..


   
    lv = abs(gain_vx - (gain_vth * WHEEL_DIST)/ 2);
    rv = abs(gain_vx + (gain_vth * WHEEL_DIST)/ 2);      // vx*2- right_vel

   
    lv_w=lv+2.17;
    rv_w=rv;
    l_enc_w=(lv/10)*2-8;
    //rv_w=rv;
    r_enc_w=(rv/10)*2-8;

    l_error=abs(encoderLPos-l_enc_w);
    r_error=abs(encoderRPos-r_enc_w);
    
    if(encoderRPos<r_enc_w){
      rv=rv_w+2*r_error*0.3;
    }
    if(encoderRPos>r_enc_w){
      rv=rv_w-2*r_error*0.2;
    }
    
    if(encoderLPos>l_enc_w){
      lv=lv_w-2*l_error*0.2;
    }
    if(encoderLPos<l_enc_w){
      lv=lv_w+2*l_error*0.3;
    }

/*
    if(lv<20)
      lv=20;
    if(rv<20)
      rv=20; */ 
    if(msg.linear.x>0){
      move(1, rv, 1);
      move(0, lv, 0);
    }
    if(msg.linear.x<0){
      move(1, rv, 0);
      move(0, lv, 1);
    }
    if(msg.linear.x==0){

      if(msg.angular.z < 0)//right
      {
         move(1, rv, 0);
         move(0, lv, 0); 
      }
      if(msg.angular.z > 0) //left
      {
         move(1, rv, 1);
         move(0, lv, 1);     
      }
      if(msg.angular.z == 0) //
      {
         move(1, 0, 0);
         move(0, 0, 0);     
      }          
    }

//////////////mapping 
/*
    if(vx>0){
      move(0, 50, 0);
      move(1, 50, 1);
    }
    if(vx<0){
      move(0, 50, 1);
      move(1, 50, 0);
    }
    if(vx==0){
      move(0, 0, 0);       
      move(1, 0, 0);
    }
    if(vth>0){
      move(0, 50, 1);
      move(1, 50, 1);
    }
    if(vth<0){
      move(0, 50, 0);
      move(1, 50, 0);
    }
   */
}
ros::NodeHandle n;
std_msgs::Float64 encoder_msg;
ros::Publisher pub_encoder("encoder_pulse", &encoder_msg);
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", motor);
//ros::Subscriber<geometry_msgs::Twist> subs("yaw", sensor);

void setup() {
  //Moter
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  digitalWrite(PWMA, LOW);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  pinMode(encoder1PinA, INPUT); 
  pinMode(encoder1PinB, INPUT);
  attachInterrupt(4, doEncoder1A, CHANGE); // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(5, doEncoder1B, CHANGE); // encoder pin on interrupt 1 (pin 3)
    
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  digitalWrite(PWMB, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  pinMode(encoder0PinA, INPUT); 
  pinMode(encoder0PinB, INPUT);
  attachInterrupt(0, doEncoderA, CHANGE); // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(1, doEncoderB, CHANGE); // encoder pin on interrupt 1 (pin 3)
    
  // ROS Node
  n.initNode();
  n.subscribe(sub);
  //n.subscribe(subs);
  n.advertise(pub_encoder);
}

void loop() {
  //motor  
  if(vx < 0) {
  Encoder_value = (encoderRPos+encoderLPos)/2;
  }
  else if(vx > 0) {
  Encoder_value = (encoderRPos+encoderLPos)/2;
  }
  else {
  Encoder_value = 0;
  }
  Serial.println(encoderLPos);
  encoderRPos=0;
  encoderLPos=0;
  encoder_msg.data = Encoder_value;
  pub_encoder.publish(&encoder_msg);
  n.spinOnce();
  
  delay(10);
}

void move(int motor, int speed, int direction){
//Move specific motor at speed and direction
//motor: 0 for B 1 for A
//speed: 0 is off, and 255 is full speed
//direction: 0 clockwise, 1 counter-clockwise

  digitalWrite(STBY, HIGH); //disable standby
  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if(direction == 1){
    inPin1 = HIGH;
    inPin2 = LOW;
  }
  if(motor == 1){
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, speed);
  }
   else{
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
    analogWrite(PWMB, speed);
  }
}

void doEncoderA(){
  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == HIGH) {  
      encoderRPos = encoderRPos + 1;
    } 
    else {
      encoderRPos = encoderRPos - 1;
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinB) == LOW) {   
      encoderRPos = encoderRPos + 1;
    } 
    else {
      encoderRPos = encoderRPos - 1;
    }
  }
}

void doEncoderB(){
  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == LOW) {  
      encoderRPos = encoderRPos - 1;
    } 
    else {
      encoderRPos = encoderRPos + 1;
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinA) == HIGH) {   
      encoderRPos = encoderRPos - 1;
    } 
    else {
      encoderRPos = encoderRPos + 1;
    }
  }
}
void doEncoder1A(){
  // look for a low-to-high on channel A
  if (digitalRead(encoder1PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder1PinB) == HIGH) {  
      encoderLPos = encoderLPos + 1;
    } 
    else {
      encoderLPos = encoderLPos - 1;
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder1PinB) == LOW) {   
      encoderLPos = encoderLPos + 1;
    } 
    else {
      encoderLPos = encoderLPos - 1;
    }
  }
}

void doEncoder1B(){
  // look for a low-to-high on channel B
  if (digitalRead(encoder1PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder1PinA) == LOW) {  
      encoderLPos = encoderLPos + 1;
    } 
    else {
      encoderLPos = encoderLPos - 1;
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder1PinA) == HIGH) {   
      encoderLPos = encoderLPos + 1;
    } 
    else {
      encoderLPos = encoderLPos - 1;
    }
  }
}
