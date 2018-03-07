#!/bin/sh

BASE=$(rospack find teleop_control)/scripts

#$BASE/term.sh roscore & 
#sleep 2

#rosparam set enable_statistics true

echo "arduino connector"
$BASE/init.sh 0
sleep 2

echo "Starting teleop" 
$BASE/init.sh 1
sleep 2

echo "echoing teleop msgs"
$BASE/init.sh 2 
sleep 2

