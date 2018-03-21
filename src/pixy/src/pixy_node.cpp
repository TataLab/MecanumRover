#include <iostream>
#include <ros/ros.h>
#include <angles/angles.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <cmath>

#include "pixy.h"

#define BLOCK_BUFFER_SIZE 25

#define PIXY_X_CENTER ((PIXY_MAX_X - PIXY_MIN_X) / 2)
#define PIXY_Y_CENTER ((PIXY_MAX_Y - PIXY_MIN_Y) / 2)

// offset of the camera relative to the "center" of the robot
#define X_POS 0
#define Y_POS 0
#define Z_POS 0

#define PIXY_RCS_PAN_CHANNEL 0
#define PIXY_RCS_TILT_CHANNEL 1
// PID control parameters //
#define PAN_PROPORTIONAL_GAIN 400
#define PAN_DERIVATIVE_GAIN 100
#define TILT_PROPORTIONAL_GAIN 600
#define TILT_DERIVATIVE_GAIN 200

const double PI = acos(-1);

// Pixy Block Buffer //
struct Block blocks[BLOCK_BUFFER_SIZE];

static bool run_flag = true;

struct Gimbal
{
  int32_t position;
  int32_t previous_error;
  int32_t proportional_gain;
  int32_t derivative_gain;
};

// PID control variables //

struct Gimbal pan;
struct Gimbal tilt;

void handle_SIGINT(int unused)
{
  // On CTRL+C - abort! //

  run_flag = false;
}

void initialize_gimbals()
{
  pan.position = PIXY_RCS_CENTER_POS;
  pan.previous_error = 0x80000000L;
  pan.proportional_gain = PAN_PROPORTIONAL_GAIN;
  pan.derivative_gain = PAN_DERIVATIVE_GAIN;
  tilt.position = PIXY_RCS_CENTER_POS;
  tilt.previous_error = 0x80000000L;
  tilt.proportional_gain = TILT_PROPORTIONAL_GAIN;
  tilt.derivative_gain = TILT_DERIVATIVE_GAIN;
}

void gimbal_update(struct Gimbal *gimbal, int32_t error)
{
  long int velocity;
  int32_t error_delta;
  int32_t P_gain;
  int32_t D_gain;

  if (gimbal->previous_error != 0x80000000L)
  {

    error_delta = error - gimbal->previous_error;
    P_gain = gimbal->proportional_gain;
    D_gain = gimbal->derivative_gain;

    /* Using the proportional and derivative gain for the gimbal,
       calculate the change to the position.  */
    velocity = (error * P_gain + error_delta * D_gain) >> 10;

    gimbal->position += velocity;

    if (gimbal->position > PIXY_RCS_MAX_POS)
    {
      gimbal->position = PIXY_RCS_MAX_POS;
    }
    else if (gimbal->position < PIXY_RCS_MIN_POS)
    {
      gimbal->position = PIXY_RCS_MIN_POS;
    }
  }

  gimbal->previous_error = error;
}

double RCStoRadiens(const Gimbal &gimbal)
{
  double radien = ((gimbal.position - 500)/1000.0) * PI;
  return radien;
}


std::string turtle_name;

int main(int argc, char *argv[])
{
  int pixy_init_status;
  char buf[128];
  int frame_index = 0;
  int result;
  int pan_error;
  int tilt_error;
  int blocks_copied;
  int index;

  printf("+ Pixy Tracking Demo Started +\n");
  printf("+ With ROS Broadcaster +\n");
  fflush(stdout);

  initialize_gimbals();

  // Catch CTRL+C (SIGINT) signals //
  signal(SIGINT, handle_SIGINT);

  // Connect to Pixy //
  pixy_init_status = pixy_init();

  // Was there an error initializing pixy? //
  if (pixy_init_status != 0)
  {
    // Error initializing Pixy //
    printf("Error: pixy_init() [%d] ", pixy_init_status);
    pixy_error(pixy_init_status);

    return pixy_init_status;
  }

  ros::init(argc, argv, "my_tf2_broadcaster");

  ros::NodeHandle private_node("~");

  if (!private_node.hasParam("turtle"))
  {
    if (argc != 2)
    {
      ROS_ERROR("need turtle name as argument");
      turtle_name = argv[1];
    }
    else
    {
      private_node.getParam("turtle", turtle_name);
    }
  }

  ros::Rate rate(10);
  ros::NodeHandle node;

  // twist message publisher
  ros::NodeHandle movement; 
  ros::Publisher pub = movement.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

  //ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);

  while (run_flag && ros::ok())
  {

    // Wait for new blocks to be available //
    while (!pixy_blocks_are_new() && run_flag)
      ;

    // Get blocks from Pixy //

    blocks_copied = pixy_get_blocks(BLOCK_BUFFER_SIZE, &blocks[0]);

    if (blocks_copied < 0)
    {
      // Error: pixy_get_blocks //
      printf("Error: pixy_get_blocks() [%d] ", blocks_copied);
      pixy_error(blocks_copied);
      fflush(stdout);
    }

    if (blocks_copied > 0)
    {
      // Calculate the difference between the   //
      // center of Pixy's focus and the target. //

      pan_error = PIXY_X_CENTER - blocks[0].x;
      tilt_error = blocks[0].y - PIXY_Y_CENTER;

      // Apply corrections to the pan/tilt with the goal //
      // of putting the target in the center of          //
      // Pixy's focus.                                   //

      gimbal_update(&pan, pan_error);
      gimbal_update(&tilt, tilt_error);

      result = pixy_rcs_set_position(PIXY_RCS_PAN_CHANNEL, 1000-pan.position);
      if (result < 0)
      {
        printf("Error: pixy_rcs_set_position() [%d] ", result);
        pixy_error(result);
        fflush(stdout);
      }

      result = pixy_rcs_set_position(PIXY_RCS_TILT_CHANNEL, 1000-tilt.position);
      if (result < 0)
      {
        printf("Error: pixy_rcs_set_position() [%d] ", result);
        pixy_error(result);
        fflush(stdout);
      }
    }

    // start
    double angle_tilt = RCStoRadiens(tilt);
    double angle_pan = RCStoRadiens(pan);

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::Twist moveMsg;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "pixy_node";
    transformStamped.transform.translation.x = X_POS;
    transformStamped.transform.translation.y = Y_POS;
    transformStamped.transform.translation.z = Z_POS;
    tf2::Quaternion q;
    q.setRPY(0, angle_tilt, angle_pan);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    double vectorMag = cos(angle_tilt);
    double move_x = vectorMag*cos(angle_pan);
    double move_y = vectorMag*sin(angle_pan);

    moveMsg.linear.x = move_x;
    moveMsg.linear.y = move_y;
    // publish the twist msg
    pub.publish(moveMsg);
    // publish the transform msg
    br.sendTransform(transformStamped);

    rate.sleep();

    //ROS_INFO("I heard: [%s]", transformStamped.data.angle());
    // end

    if (frame_index % 10 == 0)
    {
      // Display received blocks //
      printf("frame %d:\n", frame_index);
      /*
      for (index = 0; index != blocks_copied; ++index)
      {
        printf("  sig:%2d x:%4d y:%4d width:%4d height:%4d\n",
               blocks[index].signature,
               blocks[index].x,
               blocks[index].y,
               blocks[index].width,
               blocks[index].height);
      }
      */
      std::cout<<"Pan angle: "<<angle_pan<<"\n";
      std::cout<<"Tilt angle: "<<angle_tilt<<"\n";
      fflush(stdout);
    }

    frame_index++;
  }
  ros::spin();
  pixy_close();

  return 0;
}
