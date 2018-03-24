#!/bin/sh

echo "connecting to arduino"

rosrun rosserial_python serial_node.py /dev/ttyACM0