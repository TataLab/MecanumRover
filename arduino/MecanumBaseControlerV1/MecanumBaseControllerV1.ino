//control the mechanum chasis
#include "Arduino.h"
#include "EncoderComm.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

//storage for each motor velocity and number of revolutions
int motors = 4;
float Ierror[4] = {0,0,0,0};
float preverror[4] = {0,0,0,0};     
float vel[4] = {0,0,0,0};
float revs[4] = {0,0,0,0};
byte power[4] = {0,0,0,0};
float setPoints[4] = {0,0,0,0};
float magnitude =1;
float angle = 3.14/4;
EncoderComm EC;

//function to retrieve velocity
void get_velocity(){
  EC.requestMsg('g');
  for(int i=0;i<4;i++) {
        vel[i] = EC.getVelocity(i);//Get the volocity from the EC after doing the requestMsg
        //Serial.print(vel[i]);
        revs[i] = EC.getRev(i);//Get the Rev from the EC after doing the requestMsg
   }  
}

//PID function
void doPID(float *setPoints, float *velocity, byte *power, float *Ierror, float *preverror){ 
    const int MotorCount = 4 ;
    const float KP = 1.00f;
    const float KI = 0.01f;
    const float KD = 0.005f;
    float error;
    float derror;
    float p;
    //Serial.println("POWERS");
    for (int i =0;i<MotorCount ;i++) {
        error = setPoints[i]-abs(velocity[i]);
        Ierror[i] = Ierror[i] + error;      
        derror = error - preverror[i];
        p = power[i] + round(((KP*error) + (KI*Ierror[i]) + (KD*derror)));
        if (p>255) power[i]=255;
        else if(p<0) power[i]=0;
        else power[i]=byte(p);
        preverror[i] = error;
        //Serial.println(power[i]);
  } 
}

//function prototypes to keep things neat and tidy
void twistMessageCb(const geometry_msgs::Twist& msg);

//*******setup for ROS*********//
ros::NodeHandle  nodeHandle;


// subscriber callback object for twist message
// the callback function (2nd parameter) is regestered in setup()
// whenever a twist message is published to the "miniq/cmd_vel" topic on this node, the callback 
// function (twistMessageCb) is called
ros::Subscriber<geometry_msgs::Twist> subscriber("/mecanum/cmd_vel", &twistMessageCb);
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

float MAX_VELOCITY_RADS = 3.7 * 2 * 3.1415; //in radians per second;  3.7 rotations per second is the observed max angular rate of the mecanum wheels
float MAX_VELOCITY_METRES = MAX_VELOCITY_RADS * WHEEL_RADIUS; //in radians per second;  3.7 rotations per second is the observed max angular rate of the mecanum wheels

void setup()
{ 
  EC.requestMsg('r');
  nodeHandle.initNode();

  // registers callback function on "mecanum/cmd_vel" topic via subscriber object
  nodeHandle.subscribe(subscriber);

  //initialize the motor shield
  motorShield.begin();

//  for(int i = 0; i<4; i++){
//    setPoints[i]=1.0;        
//  }

       
}

void loop()
{

  nodeHandle.spinOnce();

  //delay(5); 
  get_velocity();
  doPID( setPoints, vel, power,Ierror,preverror);
  frontLeftMotor->run(FORWARD);
  frontRightMotor->run(BACKWARD);
  rearLeftMotor->run(FORWARD);
  rearRightMotor->run(BACKWARD);
  frontLeftMotor->setSpeed(power[0]);
  frontRightMotor->setSpeed(power[3]);
  rearLeftMotor->setSpeed(power[1]); 
  rearRightMotor->setSpeed(power[2]);

}


//this is the twist message callback
void twistMessageCb(const geometry_msgs::Twist& msg)
{      
       // example of how to usethe variables contained in the twist message
        // differential drive systems only need a linear velocity (we'll use linear.x) and an angular velocity (we'll use angular.z)
	geometry_msgs::Vector3 linear = msg.linear;
	
	geometry_msgs::Vector3 angular = msg.angular;
        
        //unpack the Twist message
        float wheel_front_left_V = (1/WHEEL_RADIUS) * (linear.x-linear.y-(WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angular.z);
        float wheel_front_right_V = (1/WHEEL_RADIUS) * (linear.x + linear.y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angular.z);
        float wheel_rear_left_V = (1/WHEEL_RADIUS) * (linear.x + linear.y-(WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angular.z);
        float wheel_rear_right_V = (1/WHEEL_RADIUS) * (linear.x-linear.y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angular.z); 
        setPoints[0] = wheel_front_left_V;
        setPoints[3] = wheel_front_right_V;
        setPoints[1] = wheel_rear_left_V;
        setPoints[2] = wheel_rear_right_V;
        //map the velocities into 0-255 for the motor shield
        int wheel_front_left_motor=(int)mapFloat(wheel_front_left_V,0.0,MAX_VELOCITY_RADS,0.0,255.0);
        int wheel_front_right_motor=(int)mapFloat(wheel_front_right_V,0.0,MAX_VELOCITY_RADS,0.0,255.0);
        int wheel_rear_left_motor=(int)mapFloat(wheel_rear_left_V,0.0,MAX_VELOCITY_RADS,0.0,255.0);  
        int wheel_rear_right_motor=(int)mapFloat(wheel_rear_right_V,0.0,MAX_VELOCITY_RADS,0.0,255.0);
        power[0]=wheel_front_left_motor;
        power[3]=wheel_front_right_motor;
        power[1]=wheel_rear_left_motor;
        power[2]=wheel_rear_right_motor;
        
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
      EC.requestMsg('r');
      //now set the motors
      frontLeftMotor->setSpeed(abs(wheel_front_left_motor));
      frontRightMotor->setSpeed(abs(wheel_front_right_motor));
      rearLeftMotor->setSpeed(abs(wheel_rear_left_motor));
      rearRightMotor->setSpeed(abs(wheel_rear_right_motor));
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
