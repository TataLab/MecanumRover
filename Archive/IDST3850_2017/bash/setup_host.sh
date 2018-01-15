source ./devel/setup.bash
export ROS_IP=$(hostname -I)

roslaunch robot_2dnav robot.launch
#roslaunch robot_2dnav move_base.launch
