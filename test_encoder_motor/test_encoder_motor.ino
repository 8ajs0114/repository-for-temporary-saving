//motor
#define ENABLE_A  10  
#define IN1_A  9   
#define IN2_A  8   

byte speedDC = 255;  

//encoder
int encoder0PinA = 3;
int encoder0PinB = 4;
int encoder0Pos = 0;
int encoder0PinALast = LOW;
int n = LOW;


  
void setup()  
{ 
  //motor 
  pinMode(ENABLE_A,OUTPUT);  
  pinMode(IN1_A,OUTPUT);  
  pinMode(IN2_A,OUTPUT);  

  //encoder
  pinMode (encoder0PinA,INPUT);
  pinMode (encoder0PinB,INPUT);
  Serial.begin (115200);
  attachInterrupt(0, doEncoderA, CHANGE); 
  attachInterrupt(1, doEncoderB, CHANGE); 
}

//motor   
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

//encoder
void doEncoderA(){
  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {  
      encoder0Pos = encoder0Pos + 1;
    } 
    else {
      encoder0Pos = encoder0Pos - 1;
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinB) == HIGH) {   
      encoder0Pos = encoder0Pos + 1;
    } 
    else {
      encoder0Pos = encoder0Pos - 1;
    }
  }
  Serial.println (encoder0Pos);
}

void doEncoderB(){
  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {  
      encoder0Pos = encoder0Pos + 1;
    } 
    else {
      encoder0Pos = encoder0Pos - 1;
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinA) == LOW) {   
      encoder0Pos = encoder0Pos + 1;
    } 
    else {
      encoder0Pos = encoder0Pos - 1;
    }
  }
  Serial.println (encoder0Pos);
}
  
void loop()  
{  
  motor_dir(1);  
  for ( int i = 0; i < 100; i++ ) 
  {  
      motor_speed(i);  
      delay(10);  
  }  

  for ( int j = 100; j >0 ; j--)
  {
      motor_speed(j);  
      delay(10);
  }

  motor_dir(2);  
  for ( int i = 0; i < 100; i++ ) 
  {  
      motor_speed(i);  
      delay(10);  
  }  
  
  for ( int j = 100; j > 0 ; j--)
  {
      motor_speed(j);  
      delay(10);
  }
}  
