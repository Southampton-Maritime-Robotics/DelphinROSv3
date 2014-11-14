#/bin/bash

export ROS_MASTER_URI=http://169.254.51.57:11311
rosrun joy joy_node &
rosrun DelphinROS joystick.py

