/* PID balance code with ping pong ball and distance sensor sharp 2y0a21
 *  by ELECTRONOOBS: https://www.youtube.com/channel/UCjiVhIvGmRZixSzupD0sS9Q
 *  Tutorial: http://electronoobs.com/eng_arduino_tut100.php
 *  Code: http://electronoobs.com/eng_arduino_tut100_code1.php
 *  Scheamtic: http://electronoobs.com/eng_arduino_tut100_sch1.php 
 *  3D parts: http://electronoobs.com/eng_arduino_tut100_stl1.php   
 */
#include <Wire.h>
#include <Servo.h>



///////////////////////Inputs/outputs///////////////////////
int Analog_in = A0;
Servo myservo;  // create servo object to control a servo, later attatched to D9
///////////////////////////////////////////////////////


////////////////////////Variables///////////////////////
int Read = 0;
float distance = 0.0;
float elapsedTime, time, timePrev;        //Variables for time control
float distance_previous_error, distance_error;
int period = 100;  //Refresh rate period of the loop is 50ms
///////////////////////////////////////////////////////


///////////////////PID constants///////////////////////
float kp=6; //Mine was 8 reacts to distance
float ki=0.5; //Mine was 0.2 reacts to standstill
float kd=3100; //Mine was 3100 reacts to speed
float distance_setpoint =14 ;           //Should be the distance from sensor to the middle of the bar in mm
float PID_p, PID_i, PID_d, PID_total;
///////////////////////////////////////////////////////

int servopin = 10;
int echo = 12;    //Echo pin of sensor
int trig = 11;    //Trigger pin of sensor


void setup() {
  //analogReference(EXTERNAL);
  Serial.begin(9600);  
  myservo.attach(servopin);  // attaches the servo on pin 9 to the servo object
  myservo.write(65); //Put the servco at angle 125, so the balance is in the middle
  //pinMode(Analog_in,INPUT); 
    pinMode(trig,OUTPUT);
  pinMode(echo,INPUT); 
  time = millis();
}

void loop() {
  if (millis() > time+period)
  {
    time = millis();    
    distance = readPosition();   
    distance_error = distance_setpoint - distance;   
    PID_p = kp * distance_error;
    float dist_diference = distance_error - distance_previous_error;     
    PID_d = kd*((distance_error - distance_previous_error)/period);
      
    if(-3 < distance_error && distance_error < 3)
    {
      PID_i = PID_i + (ki * distance_error);
    }
    else
    {
      PID_i = 0;
    }
  
    PID_total = PID_p + PID_i + PID_d;  
    PID_total = map(PID_total, -150, 150, 0, 150);
  
    if(PID_total < 20){PID_total = 20;}
    if(PID_total > 160) {PID_total = 160; } 
  
    myservo.write(PID_total);  
    distance_previous_error = distance_error;
  }
}











float readPosition() 
{
  delay(40);                                                            //Don't set too low or echos will run into eachother.        
long duration, cm;
unsigned long now = millis();  
  digitalWrite(trig, LOW);
  delayMicroseconds(5);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  pinMode(echo, INPUT);
  duration = pulseIn(echo, HIGH);
  cm = (duration/2)/(29.1);    
  if(cm > 30)     // 30 cm is the maximum position for the ball
  {cm=30;}  
  Serial.println(cm);
  return cm;                                          //Returns distance value.
}
