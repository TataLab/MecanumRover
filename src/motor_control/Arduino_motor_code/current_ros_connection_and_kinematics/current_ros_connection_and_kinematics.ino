#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <ros.h>  
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>


Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Motor declarations
const int FL = 1; //front left motor
const int RL = 2; //rear left
const int RR = 3; //rear right
const int FR = 4; //front right

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *FLMotor = AFMS.getMotor(FL);
Adafruit_DCMotor *RLMotor = AFMS.getMotor(RL);
Adafruit_DCMotor *RRMotor = AFMS.getMotor(RR);
Adafruit_DCMotor *FRMotor = AFMS.getMotor(FR);

float tireRad = 0.07; // radius in m
float distanceWidth = 0.30; // in m
float distanceLength = 0.19; // in m

float FLSpeed = 0;
float RLSpeed = 0;
float RRSpeed = 0;
float FRSpeed = 0;

int FLWheel = 0;
int RLWheel = 0;
int RRWheel = 0;
int FRWheel = 0;

ros::NodeHandle nh;  //enables Arduino to be used as a ros node

void TwistmessageCb(const geometry_msgs::Twist & msg){     // Cb (Callback) checks too see if messages are posted

  float linx = msg.linear.x;
  float liny = msg.linear.y;
  float angz = msg.angular.z;

  FLSpeed = ((1/tireRad) * (linx - liny - ((distanceWidth + distanceLength) * angz)));
  RLSpeed = ((1/tireRad) * (linx + liny - ((distanceWidth + distanceLength) * angz)));
  RRSpeed = ((1/tireRad) * (linx - liny + ((distanceWidth + distanceLength) * angz)));
  FRSpeed = ((1/tireRad) * (linx + liny + ((distanceWidth + distanceLength) * angz)));

  FLWheel = map(abs(FLSpeed), 0, 30, 0, 255);
  RLWheel = map(abs(RLSpeed), 0, 30, 0, 255);
  RRWheel = map(abs(RRSpeed), 0, 30, 0, 255);
  FRWheel = map(abs(FRSpeed), 0, 30, 0, 255);

  FLMotor->setSpeed(FLWheel);
  RLMotor->setSpeed(RLWheel);
  RRMotor->setSpeed(RRWheel);
  FRMotor->setSpeed(FRWheel);
  
  if(FLSpeed>=0.0){
    FLMotor->run(FORWARD);
  }
  else{
    FLMotor->run(BACKWARD); 
  }
  
  if(FRSpeed>=0.0){
    RLMotor->run(FORWARD);
  }
  else{
    RLMotor->run(BACKWARD); 
  }
  
  if(RLSpeed>=0.0){
    RRMotor->run(FORWARD);
  }
  else{
    RRMotor->run(BACKWARD); 
  }
  
  if(RRSpeed>=0.0){
    FRMotor->run(FORWARD);
  }
  else{
    FRMotor->run(BACKWARD); 
  }

  delay(500);
  
  // turn on motor
  FLMotor->run(RELEASE);
  RLMotor->run(RELEASE);
  RRMotor->run(RELEASE);
  FRMotor->run(RELEASE);
  // need to know what message names are being pubished

}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &TwistmessageCb);

void setup() 
{
  Serial.begin(9600); 

  AFMS.begin();  // create with the default frequency 1.6KHz
  
 // AllTires();

  nh.initNode();
  nh.subscribe(sub);
}



void loop() 
{
  
  nh.spinOnce();      // Run all the nh (Node Handle) code once
  delay(1);           //For Serial.print to work
 
}


