//control the mechanum chasis without PID; just respond to Twist messages
#include "Arduino.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>



//function prototypes to keep things neat and tidy
void twistMessageCb(const geometry_msgs::Twist& msg);

//*******setup for ROS*********//
ros::NodeHandle  nodeHandle;

// subscriber callback object for twist message
// the callback function (2nd parameter) is regestered in setup()
// whenever a twist message is published to the "mecanum/cmd_vel" topic on this node, the callback 
// function (twistMessageCb) is called
ros::Subscriber<geometry_msgs::Twist> subscriber("mecanum/cmd_vel", &twistMessageCb);
//ros::Subscriber<geometry_msgs::Twist> subscriber("turtle1/cmd_vel", &twistMessageCb);  //use this to drive the rover from the turtlesim console

//*******done setting up ROS ************//


//******setup for motor control*********//

// Create the motor shield object with the default I2C address
Adafruit_MotorShield motorShield = Adafruit_MotorShield(); 


// Select which 'port' M1, M2, M3 or M4. In this case, M1 will be left and M2 will be right
Adafruit_DCMotor *frontLeftMotor = motorShield.getMotor(1);
Adafruit_DCMotor *frontRightMotor = motorShield.getMotor(4);
Adafruit_DCMotor *rearLeftMotor = motorShield.getMotor(2);
Adafruit_DCMotor *rearRightMotor = motorShield.getMotor(3);

//setup parameters of the rover
float WHEEL_SEPARATION_WIDTH = 0.305 / 2; //half the width of the rover wheelbase in m
float WHEEL_SEPARATION_LENGTH = 0.19 / 2; 
float WHEEL_RADIUS = 0.05;  //wheel radius in m


float MAX_VELOCITY_RADS =  3.7 * 2 * 3.1415; //in radians per second;  3.7 rotations per second is the observed max angular rate of the mecanum wheels
float MAX_VELOCITY_METRES = MAX_VELOCITY_RADS * WHEEL_RADIUS; //in radians per second;  3.7 rotations per second is the observed max angular rate of the mecanum wheels

//use these bounds to limit the ultimate motor mapping speed to protect the motors and gear train
float MIN_MOTOR_SPEED = 0.0;
float MAX_MOTOR_SPEED = 150; 

void setup()
{ 
  nodeHandle.initNode();

  // registers callback function on "mecanum/cmd_vel" topic via subscriber object
  nodeHandle.subscribe(subscriber);

  //initialize the motor shield
  motorShield.begin();
  //let the motors spin a bit in case they were controlled already
  frontLeftMotor->run(RELEASE);
  frontRightMotor->run(RELEASE);
  rearLeftMotor->run(RELEASE);
  rearRightMotor->run(RELEASE);
  
  delay(5000);

//for debugging
//Serial.begin(9600);           // set up Serial library at 9600 bps
     
}

void loop()
{
  //delay(50); 
  nodeHandle.spinOnce();
  
  //for debugging, pretend you're getting a twist message 
  //fakeTwistMessageCb();
  

}


//this is the twist message callback
void fakeTwistMessageCb()
{      
       // example of how to usethe variables contained in the twist message
        // differential drive systems only need a linear velocity (we'll use linear.x) and an angular velocity (we'll use angular.z)
	geometry_msgs::Vector3 linear;
	
	geometry_msgs::Vector3 angular;
        
        linear.x=2;
        linear.y=0;
        angular.z=0;
        
        //unpack the Twist message
        float wheel_front_left_V = (1/WHEEL_RADIUS) * (linear.x-linear.y-(WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angular.z);
        float wheel_front_right_V = (1/WHEEL_RADIUS) * (linear.x + linear.y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angular.z);
        float wheel_rear_left_V = (1/WHEEL_RADIUS) * (linear.x + linear.y-(WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angular.z);
        float wheel_rear_right_V = (1/WHEEL_RADIUS) * (linear.x-linear.y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angular.z); 
      

        //reverse the right wheels because they are mirror images of the left
        wheel_front_right_V *= -1.0;
        wheel_rear_right_V *= -1.0;
 
        //do a bit of logic to sort out angular direction because the motors aren't facing the same way!?
        if(wheel_front_left_V>=0.0){
          frontLeftMotor->run(FORWARD);
        }
        
        else{
          frontLeftMotor->run(BACKWARD);
        }
        
      if(wheel_front_right_V>=0.0){
          frontRightMotor->run(FORWARD);
        }
        
        else{
          frontRightMotor->run(BACKWARD);
        }
        
        if(wheel_rear_left_V>=0.0){
          rearLeftMotor->run(FORWARD);
        }
        
        else{
          rearLeftMotor->run(BACKWARD);
        }
        
        if(wheel_rear_right_V>=0.0){
          rearRightMotor->run(FORWARD);
        }
        
        else{
          rearRightMotor->run(BACKWARD);
        }
        
        //strip off the sign because we don't need it anymore
        wheel_front_left_V=abs(wheel_front_left_V);
        wheel_front_right_V=abs(wheel_front_right_V);
        wheel_rear_left_V=abs(wheel_rear_left_V);
        wheel_rear_right_V=abs(wheel_rear_right_V);
              
        //check bounds
        if(wheel_front_left_V > MAX_VELOCITY_RADS){wheel_front_left_V=MAX_VELOCITY_RADS;}
        if(wheel_front_right_V > MAX_VELOCITY_RADS){wheel_front_right_V=MAX_VELOCITY_RADS;}
        if(wheel_rear_left_V > MAX_VELOCITY_RADS){wheel_rear_left_V=MAX_VELOCITY_RADS;}
        if(wheel_rear_right_V > MAX_VELOCITY_RADS){wheel_rear_right_V=MAX_VELOCITY_RADS;}
      
      
        //map the velocities into 0-255 for the motor shield
        int wheel_front_left_motor=(int)mapFloat(abs(wheel_front_left_V),0.0,MAX_VELOCITY_RADS,MIN_MOTOR_SPEED,MAX_MOTOR_SPEED);
        int wheel_front_right_motor=(int)mapFloat(abs(wheel_front_right_V),0.0,MAX_VELOCITY_RADS,MIN_MOTOR_SPEED,MAX_MOTOR_SPEED);
        int wheel_rear_left_motor=(int)mapFloat(abs(wheel_rear_left_V),0.0,MAX_VELOCITY_RADS,MIN_MOTOR_SPEED,MAX_MOTOR_SPEED);  
        int wheel_rear_right_motor=(int)mapFloat(abs(wheel_rear_right_V),0.0,MAX_VELOCITY_RADS,MIN_MOTOR_SPEED,MAX_MOTOR_SPEED);
        
        
        
        Serial.println(wheel_front_left_V);
        Serial.println(wheel_front_right_V);
        Serial.println(wheel_rear_left_V);
        Serial.println(wheel_rear_right_V);
        Serial.println("\n\n\n");
       
        
        frontLeftMotor->setSpeed(wheel_front_left_motor);
        frontRightMotor->setSpeed(wheel_front_right_motor);
        rearLeftMotor->setSpeed(wheel_rear_left_motor);
        rearRightMotor->setSpeed(wheel_rear_right_motor);

}



//this is the twist message callback
void twistMessageCb(const geometry_msgs::Twist& msg){      
         // example of how to usethe variables contained in the twist message
        // differential drive systems only need a linear velocity (we'll use linear.x) and an angular velocity (we'll use angular.z)
	
        //let the motors spin a bit in case they were controlled already
        frontLeftMotor->run(RELEASE);
        frontRightMotor->run(RELEASE);
        rearLeftMotor->run(RELEASE);
        rearRightMotor->run(RELEASE);
        
        delay(100);


        geometry_msgs::Vector3 linear = msg.linear;
	
	geometry_msgs::Vector3 angular = msg.angular;

        //unpack the Twist message
        float wheel_front_left_V = (1/WHEEL_RADIUS) * (linear.x-linear.y-(WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angular.z);
        float wheel_front_right_V = (1/WHEEL_RADIUS) * (linear.x + linear.y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angular.z);
        float wheel_rear_left_V = (1/WHEEL_RADIUS) * (linear.x + linear.y-(WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angular.z);
        float wheel_rear_right_V = (1/WHEEL_RADIUS) * (linear.x-linear.y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angular.z); 
      

        //reverse the right wheels because they are mirror images of the left
        wheel_front_right_V *= -1.0;
        wheel_rear_right_V *= -1.0;
 
        //do a bit of logic to sort out angular direction because the motors aren't facing the same way!?
        if(wheel_front_left_V>=0.0){
          frontLeftMotor->run(FORWARD);
        }
        
        else{
          frontLeftMotor->run(BACKWARD);
        }
        
      if(wheel_front_right_V>=0.0){
          frontRightMotor->run(FORWARD);
        }
        
        else{
          frontRightMotor->run(BACKWARD);
        }
        
        if(wheel_rear_left_V>=0.0){
          rearLeftMotor->run(FORWARD);
        }
        
        else{
          rearLeftMotor->run(BACKWARD);
        }
        
        if(wheel_rear_right_V>=0.0){
          rearRightMotor->run(FORWARD);
        }
        
        else{
          rearRightMotor->run(BACKWARD);
        }
        
        //strip off the sign because we don't need it anymore
        wheel_front_left_V=abs(wheel_front_left_V);
        wheel_front_right_V=abs(wheel_front_right_V);
        wheel_rear_left_V=abs(wheel_rear_left_V);
        wheel_rear_right_V=abs(wheel_rear_right_V);
              
        //check bounds
        if(wheel_front_left_V > MAX_VELOCITY_RADS){wheel_front_left_V=MAX_VELOCITY_RADS;}
        if(wheel_front_right_V > MAX_VELOCITY_RADS){wheel_front_right_V=MAX_VELOCITY_RADS;}
        if(wheel_rear_left_V > MAX_VELOCITY_RADS){wheel_rear_left_V=MAX_VELOCITY_RADS;}
        if(wheel_rear_right_V > MAX_VELOCITY_RADS){wheel_rear_right_V=MAX_VELOCITY_RADS;}
      
      
        //map the velocities into 0-255 for the motor shield
        int wheel_front_left_motor=(int)mapFloat(abs(wheel_front_left_V),0.0,MAX_VELOCITY_RADS,MIN_MOTOR_SPEED,MAX_MOTOR_SPEED);
        int wheel_front_right_motor=(int)mapFloat(abs(wheel_front_right_V),0.0,MAX_VELOCITY_RADS,MIN_MOTOR_SPEED,MAX_MOTOR_SPEED);
        int wheel_rear_left_motor=(int)mapFloat(abs(wheel_rear_left_V),0.0,MAX_VELOCITY_RADS,MIN_MOTOR_SPEED,MAX_MOTOR_SPEED);  
        int wheel_rear_right_motor=(int)mapFloat(abs(wheel_rear_right_V),0.0,MAX_VELOCITY_RADS,MIN_MOTOR_SPEED,MAX_MOTOR_SPEED);
        
        
        frontLeftMotor->setSpeed(wheel_front_left_motor);
        frontRightMotor->setSpeed(wheel_front_right_motor);
        rearLeftMotor->setSpeed(wheel_rear_left_motor);
        rearRightMotor->setSpeed(wheel_rear_right_motor);
        
        
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
