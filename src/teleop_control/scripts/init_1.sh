#!/bin/sh

sshpass -p 'raspberry' ssh pi@10.88.16.237

export ROS_MASTER_URI=http://localhost:11311

#rosrun teleop_twist_keyboard teleop_twist_keyboard.py