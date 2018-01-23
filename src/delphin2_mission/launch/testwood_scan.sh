#!/bin/bash
# Set prefix to be used for rosbags, parameter dumps and mapping
# script for scanning the area accross the step in the middle of testwood lakes
fileprefix="testwood_pathA_surface"

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

#rostopic pub /light_onOff std_msgs/Bool True -r 10 &
#lightID=$!


# load parameters
rosparam load ${parampath}default_parameters.yaml
rosparam set xsens/filter_profile 39

rosparam set sonar/analyse/BlankDist 0.6
rosparam set sonar/analyse/Threshold 95

rosparam set vertical/PID/weight/thruster/U_star 0.9  
rosparam set vertical/PID/weight/thruster/delta 0.03 

rosparam set sonar/IGainB1 160
rosparam set sonar/IGainB2 160
rosparam set sonar/LLim 300
rosparam set sonar/RLim 359
rosparam set sonar/Range 7
rosparam set sonar/Step 64

rosparam set vertical/PID/weight/thruster/U_star 0.9
rosparam set vertical/PID/weight/thruster/delta 0.03

rosparam set mission/Altitude 2.
rosparam set mission/Prop 0.0 
rosparam set mission/ForwardsTime 0.0 
rosparam set max-depth-demand 2.1
rosparam set over-depth 2.4
# set mission timeout in minutes
# THIS IS FOR GOING ONE WAY AND THEN RETURNING!!
rosparam set mission-timeout 70   # mission timeout in minutes

############################################################### New Mission
rosparam set mission/DiveHeading 40.
rosparam set mission/PathName "path_over_step"    # to_start_A, to_start_Areverse, path_A
timestamp="_$(date +%Y-%m-%d_%H:%M:%S)"


roslaunch map-octomap.launch &  # start mapping
mapID=$!
echo ${GREEN} map process ID $mapID ${NC}

# record rosbag without full octomap
rosbag record --split --size=2000 -a -x "/octomap_full(.*)" -o ${logpath}${fileprefix} &
rosbagID=$!
echo ${GREEN} rosbag process ID $rosbagID ${GREEN}

# start mission
roslaunch delphin2_mission system_loader_testwood.launch

# ... everything below here will run at the end of the mission ...
# save map
rosrun octomap_server octomap_saver -f ${logpath}${fileprefix}${timestamp}.bt
# end rosbag
rosbagnode=$(rosnode list | grep record)
rosnode kill $rosbagnode
# save parameters
rosparam dump ${logpath}${fileprefix}${timestamp}_PARAM
# stop mapping with a sigint
kill -INT $mapID

recentlog="$(ls -t ${delphinlogpath} | head -n1)"
echo ${delphinlogpath}${recentlog}
cp -r ${delphinlogpath}${recentlog} ${logpath}${fileprefix}${timestamp}_delphinLog
cp ${parampath}system_loader.launch ${logpath}${fileprefix}${timestamp}_system_loader


sleep 5


############################################################### Clean up
sleep 5
# get the name of the most recent file
recentlog="$(ls -t ${delphinlogpath} | head -n1)"
echo ${delphinlogpath}${recentlog}
cp -r ${delphinlogpath}${recentlog} ${logpath}${fileprefix}${timestamp}_delphinLog
cp ${parampath}system_loader.launch ${logpath}${fileprefix}${timestamp}_system_loader

rosparam set over-depth 1.0

# make several attempts at getting her back
roslaunch delphin2_mission system_loader__to0.launch

roslaunch delphin2_mission system_loader__to0.launch

roslaunch delphin2_mission system_loader__to0.launch


# rosbag is hard to kill, so lets get rid of that one first
# (SIGINT doesn't work, and kill will not convert .bag.active to .bag)
rosbagnode=$(rosnode list | grep record)
rosnode kill $rosbagnode
# stop rosbag, mapping, and roscore with a sigint
kill -INT $mapID
kill -INT $roscoreID
