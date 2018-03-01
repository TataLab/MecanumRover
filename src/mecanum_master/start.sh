#!/bin/sh

echo "Starting roscore"

BASE=$(rospack find mecanum_master)/scripts


$BASE/term.sh roscore & 
sleep 2

rosparam set enable_statistics true

echo "Starting motor control & odometry"
$BASE/init.sh 1
sleep 2

echo "Starting RealSense camera"
$BASE/init.sh 2
sleep 2

echo "SLAMmin'"
$BASE/init.sh 3
sleep 2

echo "Navigation and routing"
$BASE/init.sh 4
sleep 2
