/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/Bool.h>

//range finder setup stuff//

#define echoPin1 2 // Echo Pin
#define trigPin1 3 // Trigger Pin

#define echoPin2 4 // Echo Pin
#define trigPin2 5 // Trigger Pin

#define echoPin3 6// Echo Pin
#define trigPin3 7 // Trigger Pin

#define echoPin4 8
#define trigPin4 9

int maximumRange = 200; // Maximum range needed
int minimumRange = 0; // Minimum range needed
long duration, distance; // Duration used to calculate distance

float dangerRange=50;

//ROS setup stuff//
ros::NodeHandle  nh;
std_msgs::Bool bumper_msg;
ros::Publisher bumper("bumper", &bumper_msg);



void setup()
{
  nh.initNode();
  nh.advertise(bumper);
  
 pinMode(trigPin1, OUTPUT);
 pinMode(echoPin1, INPUT);
 pinMode(trigPin2, OUTPUT);
 pinMode(echoPin2, INPUT);
 pinMode(trigPin3, OUTPUT);
 pinMode(echoPin3, INPUT);
 pinMode(echoPin4, INPUT);
 pinMode(trigPin4, OUTPUT);
  
  
}

void loop()
{
  
  bool pleaseStop=0;
 
  float distance1=pollsensors(trigPin1,echoPin1);
  float distance2=pollsensors(trigPin2,echoPin2);
  float distance3=pollsensors(trigPin3,echoPin3);
  float distance4=pollsensors(trigPin4,echoPin4);
  
  if(distance1>dangerRange || distance2>dangerRange || distance3>dangerRange || distance4>dangerRange){
      bumper_msg.data = 1;
      bumper.publish( &bumper_msg );
  }
  
  

  nh.spinOnce();
  delay(100);
}


float pollsensors(int echoPin, int trigPin) {
/* The following trigPin/echoPin cycle is used to determine the
 distance of the nearest object by bouncing soundwaves off of it. */ 
 digitalWrite(trigPin, LOW); 
 delayMicroseconds(2); 

 digitalWrite(trigPin, HIGH);
 delayMicroseconds(10); 
 
 digitalWrite(trigPin, LOW);
 duration = pulseIn(echoPin, HIGH);
 
 //Calculate the distance (in cm) based on the speed of sound.
 distance = duration/58.2;
 
 if (distance > maximumRange){
   distance = maximumRange;
 }
 else if(distance < minimumRange){
   distance = minimumRange;
 }
 
return distance;
 
}


