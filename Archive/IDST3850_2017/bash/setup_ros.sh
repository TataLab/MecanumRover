#change
source ./devel/setup.bash

export ROBOT_IP=$1
export MY_IP=$(hostname -I)
export ROS_MASTER_URI=http://$ROBOT_IP:11311
export ROS_IP=$MY_IP

roslaunch robot_2dnav kinect_rviz.launch
