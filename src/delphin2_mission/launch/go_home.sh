#!/bin/bash
# Set prefix to be used for rosbags, parameter dumps and mapping
fileprefix="testwood_going_home"

killall -9 roscore
killall -9 rosmaster

# on my computer
#logpath="/home/sophia/simulation/delphin2/MPC-tests/"
#parampath="/home/sophia/delphin/private/src/delphin2_mission/launch/"
#delphinlogpath="/home/sophia/.ros/logFiles/"
# on delphin
logpath="/home/delphin2/sophia/testwood/"
parampath="/home/delphin2/DelphinROSv3/src/delphin2_mission/launch/"
delphinlogpath="/home/delphin2/.ros/logFiles/"

# test if date is set on delphin2
# (BIOS battery runs empty regularly, this is a short test to catch that)
if [ "$(date +%Y)" -lt 2015 ]
then
    echo "Date is not set correctly"
    exit 1
fi

timestamp="_$(date +%Y-%m-%d_%H:%M:%S)"
# set some colors for the output
GREEN='\033[0;32m'
NC='\033[0m'

# set ros log directory to timestamped directory
export ROS_LOG_DIR=$logpath$fileprefix$timestamp

# start roscore and move to background
roscore &
roscoreID=$!
echo ${GREEN} roscore process ID $roscoreID ${NC}
sleep 2

# load parameters
rosparam load ${parampath}default_parameters.yaml
# make sure the vehicle will always come home at the surface
rosparam set over-depth 0.5

# start mapping
roslaunch map-octomap.launch &
mapID=$!
echo ${GREEN} map process ID $mapID ${NC}

# record rosbag
# date and time should be appended automatically
# use split command to limit the size of one file (in MB)
# since the current version of ROS has a problem with more than 2GB files
rosbag record --split --size=1024 -a -o ${logpath}${fileprefix} &
rosbagID=$!
echo ${GREEN} rosbag process ID $rosbagID ${GREEN}

rostopic pub /tail_setpoints_horizontal hardware_interfaces/tail_setpoints 23 20 -r 10 &
tailID=$!
echo ${GREEN} Control Surfaces set ID $tailID ${GREEN}


# start mission
# note: As far as I understand at the moment, all required nodes need to be launched in one
#       launch file, since only the nodes from that launch file will be killed if one of the
#       required nodes finishes/dies
roslaunch delphin2_mission system_loader__to0.launch

# ... everything below here will run at the end of the mission ...
# save map
rosrun octomap_server octomap_saver -f ${logpath}${fileprefix}${timestamp}.bt

# save parameters
# saving parameters at the end helps finding if rogue launch scripts are setting paramters
# in unexpected ways
rosparam dump ${logpath}${fileprefix}${timestamp}_PARAM


# start remote control
# this won't work with mosh, since it doesn't do x forwarding
#roslaunch delphin2_mission remote_control.launch

# rosbag is hard to kill, so lets get rid of that one first
# (SIGINT doesn't work, and kill will not convert .bag.active to .bag)
rosbagnode=$(rosnode list | grep record)
rosnode kill $rosbagnode
# stop rosbag, mapping, and roscore with a sigint
kill -INT $mapID
kill -INT $roscoreID
kill -INT $tailID

sleep 5
# get the name of the most recent file
recentlog="$(ls -t ${delphinlogpath} | head -n1)"
echo ${delphinlogpath}${recentlog}
cp -r ${delphinlogpath}${recentlog} ${logpath}${fileprefix}${timestamp}_delphinLog
cp ${parampath}system_loader.launch ${logpath}${fileprefix}${timestamp}_system_loader
