#!/bin/bash
# Set location for saving the final map 
map_path="/home/sophia/experimental-data/2017/maps/"
bag_path="/home/sophia/experimental-data/2017/10-18-testwood/"
bag_name="testwood_pathA_altitude_2017-10-18-13-47-44_0.bag"
map_prefix="testwood"
RVIZconfig="/home/sophia/.rviz/delphin2.rviz"

killall -9 roscore
killall -9 rosmaster

parampath="/home/sophia/delphin/private/src/delphin2_mission/launch/"
timestamp="_$(date +%Y-%m-%d_%H:%M:%S)"

# set some colors for the output
GREEN='\033[0;32m'
NC='\033[0m'

# start roscore and move to background
roscore &
roscoreID=$!
echo ${GREEN} roscore process ID $roscoreID ${NC}
sleep 2



# load parameters
rosparam load ${parampath}default_parameters.yaml
rosparam set sonar/analyse/BlankDist 0.6
rosparam set sonar/analyse/Threshold 80
rosparam set sonar/Range 7

# start mapping
roslaunch delphin2_mission map-octomap.launch &
mapID=$!
echo ${GREEN} map process ID $mapID ${NC}


# TODO remap octomap sensor max range to sonar range

# re-run positioning
rosrun navigation dead_reckoner.py &
deadReckonID=$!
# re-run sonar analysis
rosrun hardware_interfaces sonar_detect &
sonarDetectID=$!

rviz -d $RVIZconfig &
rvizID=$!

#################### repeat for every rosbag
# get rid of all octomap data, it gets replaced
# get rid of all processed sonar data, it gets replaced
# get rif of all tf data, since tf cannot deal with different time stamps it needs replacing, too
# get rid of position_dead, this is probably not necessary...
rosbag play ${bag_path}${bag_name} /octomap_binary:=/bla1 /octomap_full:=bla2 /octomap_point_cloud_centers:=/bla3 /octomap_server/parameter_updates:=/bla4 /sonar_mapping:=/bla5 /octomap_server/parameter_descriptions:=/bla6 /projected_map:=/bla7 /tf:=/bla8 /position_dead:=/bla9

###################

# save map
rosrun octomap_server octomap_saver -f ${map_path}${map_prefix}${timestamp}.bt

# stop everything with a sigint
kill -INT $deadReckonID
kill -INT $sonarDetectID
kill -INT $rvizID
kill -INT $mapID
kill -INT $roscoreID
