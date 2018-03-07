#!/bin/sh

echo "Starting roscore"

BASE=$(rospack find mecanum_master)/scripts

$BASE/term.sh roscore & 
sleep 2

rosparam set enable_statistics true

echo "Starting teleop control"
$BASE/init.sh 5
sleep 2