// roscore
// rosrun turtlesim turtle_teleop_key
// rostopic pub /cmd_vel geometry_msgs/Twist '{linear:{ x: y: z:}}'
// rosrun rosserial_python serial_node.py /"serial_port"   :: this is a terminal command to get ros talking to this rose node


#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <ros.h>  
#include <std_msgs/Empty.h>
// #include <geometry_msg.h> 
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>


Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Motor declarations
const int FL = 1; //front left motor
const int RL = 2; //rear left
const int RR = 3; //rear right
const int FR = 4; //front right

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor = AFMS.getMotor(FL);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(RL);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(RR);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(FR);

float tireRad = 7; // radius in cm
float distanceWidth = 30; // in cm
float distanceLength = 19; // in cm

float FLSpeed = 0;
float RLSpeed = 0;
float RRSpeed = 0;
float FRSpeed = 0;


ros::NodeHandle nh;  //enables Arduino to be used as a ros node



//ros::Subscriber<geometry_msgs::Twist> subscriber("turtle1/cmd_vel", &twistMessageCb);  //use this to drive the rover from the turtlesim console
//^^

linx = map(msg.linear.x, 0, 2, 0, 255)
liny = map(msg.linear.y, 0, 2, 0, 255)
angz = map(msg.angular.z, 0, 2, 0, 255)

void TwistmessageCb(const geometry_msgs::Twist & msg){     // Cb (Callback) checks too see if messages are posted
  
  FLSpeed = abs((1/tireRad) * (linx - liny - (distanceWidth + distanceLength) * angz));
  FRSpeed = abs(-1*((1/tireRad) * (linx + liny + (distanceWidth + distanceLength) * angz)));
  RLSpeed = abs((1/tireRad) * (linx + liny - (distanceWidth + distanceLength) * angz));
  RRSpeed = abs(-1*((1/tireRad) * (linx - liny + (distanceWidth + distanceLength) * angz)));

  //FLSpeed = map(FLSpeed, 0, 1024, 0, 255);
  //FRSpeed = map(FRSpeed, 0, 1024, 0, 255);
  //RLSpeed = map(RLSpeed, 0, 1024, 0, 255);
  //RRSpeed = map(RRSpeed, 0, 1024, 0, 255);
  
  myMotor->setSpeed(FLSpeed);
  myMotor2->setSpeed(FRSpeed);
  myMotor3->setSpeed(RLSpeed);
  myMotor4->setSpeed(RRSpeed);
  
  myMotor->run(FORWARD);
  myMotor2->run(FORWARD);
  myMotor3->run(FORWARD);
  myMotor4->run(FORWARD);
  delay(2000);
  
  // turn on motor
  myMotor->run(RELEASE);
  myMotor2->run(RELEASE);
  myMotor3->run(RELEASE);
  myMotor4->run(RELEASE);
  // need to know what message names are being pubished


}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &TwistmessageCb);
//ros::Subscriber<std_msgs::Empty>sub("cmd_vel", &TwistmessageCb); //Subscribing to topic


void setup() 
{
//  Serial.begin(9600); 

    //let the motors spin a bit in case they were controlled alread
    
//  frontLeftMotor->run(RELEASE);
//  frontRightMotor->run(RELEASE);
//  rearLeftMotor->run(RELEASE);
//  rearRightMotor->run(RELEASE);\
FLSpeed = (1/tireRad) * (msg.linear.x - msg.linear.y - (distanceWidth + distanceLength) * msg.angular.z);
  
  AFMS.begin();  // create with the default frequency 1.6KHz
  
 // AllTires();

  nh.initNode();
  nh.subscribe(sub);

/*  
  FLSpeed = (1/tireRad) * (linearX - linearY - (distanceWidth + distanceLength) * angularZ);
  FRSpeed = -1*(1/tireRad) * (linearX + linearY + (distanceWidth + distanceLength) * angularZ);
  RLSpeed = (1/tireRad) * (linearX + linearY - (distanceWidth + distanceLength) * angularZ);
  RRSpeed = -1*(1/tireRad) * (linearX - linearY + (distanceWidth + distanceLength) * angularZ);
*/
  

/*
//Taken from MecanunBalloonTracker code from Mecanum GitHub folder (Line 194-202)
  geometry_msgs::Vector3 linear = msg.linear;
  
  geometry_msgs::Vector3 angular = msg.angular;

    //unpack the Twist message
    float wheel_front_left_V = (1/WHEEL_RADIUS) * (linear.x-linear.y-(WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angular.z);
    float wheel_front_right_V = (1/WHEEL_RADIUS) * (linear.x + linear.y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angular.z);
    float wheel_rear_left_V = (1/WHEEL_RADIUS) * (linear.x + linear.y-(WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angular.z);
    float wheel_rear_right_V = (1/WHEEL_RADIUS) * (linear.x-linear.y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angular.z); 
*/    

}



void loop() 
{
    //Serial.println(FLSpeed);
    //Serial.println(FRSpeed);
    //Serial.println(RLSpeed);
    //Serial.println(RRSpeed);
    //Serial.println(" ");

  nh.spinOnce();      // Run all the nh (Node Handle) code once
  delay(1);           //For Serial.print to work

    
}

/*

void AllTires(){
  
    // Front left tire
   if (FLSpeed >= 0)
   {
    myMotor->run(FORWARD);
    myMotor->setSpeed(FLSpeed);
   } else {
    FLSpeed=abs(FLSpeed);
    myMotor->run(BACKWARD);
    myMotor->setSpeed(FLSpeed); 
   }

   // rear left tire
   if (RLSpeed >= 0)
   {
    myMotor2->run(FORWARD);
    myMotor2->setSpeed(RLSpeed);
   } else {
    RLSpeed=abs(RLSpeed);
    myMotor2->run(BACKWARD);
    myMotor2->setSpeed(RLSpeed);
    
   }

   // Rear right tire
   if (RRSpeed >= 0)
   {
    myMotor3->run(FORWARD);
    myMotor3->setSpeed(RRSpeed);
   } else {
    RRSpeed=abs(RRSpeed);
    myMotor3->run(BACKWARD);
    myMotor3->setSpeed(RRSpeed);  
   }

   // Front right tire
   if (FRSpeed >= 0)
   {
    myMotor4->run(FORWARD);
    myMotor4->setSpeed(FRSpeed);
   } else {
    FRSpeed=abs(FRSpeed);
    myMotor4->run(BACKWARD);
    myMotor4->setSpeed(FRSpeed);
   }

    delay(1000);
    // release the power to the tires
    myMotor->run(RELEASE);
    myMotor2->run(RELEASE);
    myMotor3->run(RELEASE);
    myMotor4->run(RELEASE);
}
*/



