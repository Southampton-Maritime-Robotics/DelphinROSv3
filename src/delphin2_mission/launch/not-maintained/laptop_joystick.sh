#/bin/bash

export ROS_MASTER_URI=http://169.253.51.57:11311
rosrun joy joy_node &
rosrun DelphinROSv2 joystick.py

