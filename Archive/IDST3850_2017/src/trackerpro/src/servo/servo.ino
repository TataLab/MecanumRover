/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <Servo.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>

ros::NodeHandle  nh;

Servo YawServo;
Servo PitchServo;

void messageCb( const std_msgs::Int16MultiArray& servo)
{
  // move servo
  int x = servo.data[0];
  int y = servo.data[1];

  YawServo.write(x);
  PitchServo.write(y);
}

ros::Subscriber<std_msgs::Int16MultiArray> sub("servo", &messageCb );

void setup()
{ 
  nh.initNode();
  nh.subscribe(sub);
  YawServo.attach(9);
  PitchServo.attach(10);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}

