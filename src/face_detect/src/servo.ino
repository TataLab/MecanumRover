/* 
 * Subscriber of track.
 * Moves the servo to the
 * position of a face
 */

#include <ros.h>
#include <Servo.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>

ros::NodeHandle  nh;

Servo YawServo;
Servo PitchServo;

void messageCb( const std_msgs::Int16MultiArray& servo) {

  // grab the data received
  int x = servo.data[0];
  int y = servo.data[1];

  // move the servo
  YawServo.write(x);
  PitchServo.write(y);
}

ros::Subscriber<std_msgs::Int16MultiArray> sub("track", &messageCb );

void setup() {

  // Initialize the node 
  nh.initNode();

  // set subscriber
  nh.subscribe(sub);

  // set the pins
  YawServo.attach(9);
  PitchServo.attach(10);
}

void loop() {

  // standard ROS spin  
  nh.spinOnce();

  // wait for some time
  delay(1);
}

