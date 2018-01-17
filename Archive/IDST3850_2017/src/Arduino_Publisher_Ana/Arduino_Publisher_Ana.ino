/* 
 * rosserial Ultrasound
 * using seeestudio ultrasound sensor
 * (compatible with PING))))
 */

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

#define echoPin1 5 // Echo Pin
#define trigPin1 6 // Trigger Pin

int maximumRange = 200;
int minimumRange = 0;
long duration, distance;

ros::NodeHandle  nh;

sensor_msgs::Range range_msg;
ros::Publisher pub_range1("/ultrasound1", &range_msg);
ros::Publisher pub_range2("/ultrasound2", &range_msg);
// "/arduino_ultrasound"

char frameid[] = "/ultrasound";

// long duration;
float getRange_Ultrasound(int echoPin, int trigPin)
{
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(trigPin1, OUTPUT);
  pinMode (echoPin1, INPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  // pinMode(pin_num, INPUT);
  duration = pulseIn(echoPin, HIGH);
  distance = duration/58.2;
 /* if (distance >= maximumRange || distance <= minimumRange) {
    Serial.println("-1");
  }
  else {
    Serial.println(distance);
  }
  // convert the time into a distance
  // return duration/58; //    duration/29/2, return centimeters
*/ 
return distance;
}

void setup()
{
  nh.initNode();
  nh.advertise(pub_range1);
  nh.advertise(pub_range2);

  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.1;  // fake
  range_msg.min_range = 0.0;
  range_msg.max_range = 2.0;
}

long range_time;

void loop()
{
  //publish the adc value every 50 milliseconds
  //since it takes that long for the sensor to stabilize
  if ( millis() >= range_time ){
    range_msg.range = getRange_Ultrasound(echoPin1, trigPin1);
    range_msg.header.stamp = nh.now();
    pub_range1.publish(&range_msg);

//    range_msg.range = getRange_Ultrasound(6);
//    range_msg.header.stamp = nh.now();
//    pub_range2.publish(&range_msg);

    range_time =  millis() + 50;
  }

  nh.spinOnce();
}
