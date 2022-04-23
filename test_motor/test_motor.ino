#define ENABLE_A  10  
#define IN1_A  9   
#define IN2_A  8   
  
byte speedDC = 255;  
  
void setup()  
{  
  pinMode(ENABLE_A,OUTPUT);  
  pinMode(IN1_A,OUTPUT);  
  pinMode(IN2_A,OUTPUT);  
}  
  
void motor_speed(int spd)  
{  
  analogWrite(ENABLE_A,spd);  
}  
  
void motor_dir(int dir)  
{  
  if ( dir == 1 ) {  // forward   
     digitalWrite(IN1_A,HIGH);  
     digitalWrite(IN2_A,LOW);   
  }  
 
  else if ( dir == 2 ) { // backward  
     digitalWrite(IN1_A,LOW);  
     digitalWrite(IN2_A,HIGH);  
  }     
}  
  
void loop()  
{  
  motor_dir(1);  
  for ( int i = 0; i < 255; i++ ) 
  {  
      motor_speed(i);  
      delay(100);  
  }  

  motor_dir(2);  
  for ( int i = 0; i < 255; i++ ) 
  {  
      motor_speed(i);  
      delay(100);  
  }  
}  
