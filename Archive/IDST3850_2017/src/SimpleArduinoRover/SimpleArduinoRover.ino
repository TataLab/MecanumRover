
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

ros::NodeHandle  nh;



// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 


Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);


void messageCb( const sensor_msgs::Range& range_msg){

  if(range_msg.range > 20.0 || range_msg.range < 0) {
  motor1->setSpeed(100); 
  motor2->setSpeed(100); 
    motor1->run(FORWARD);
  motor2->run(FORWARD);
  } else {
  motor1->setSpeed(100); 
  motor2->setSpeed(100); 
    motor1->run(BACKWARD);
  motor2->run(FORWARD);
  }


}

ros::Subscriber<sensor_msgs::Range> sub("ultrasound1", messageCb );



void setup() {
  // put your setup code here, to run once:

Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  motor1->setSpeed(10);
  motor2->setSpeed(10);
  motor2->run(FORWARD);
  motor2->run(FORWARD);
  
  // turn on motor
  motor1->run(RELEASE);
  motor1->run(RELEASE);

  nh.initNode();
  nh.subscribe(sub);

}

void loop() {
  // put your main code here, to run repeatedly:

  

 nh.spinOnce();
 delay(100);
}
