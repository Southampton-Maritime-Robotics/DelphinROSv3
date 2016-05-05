#!/usr/bin/python
###########
#  VERSION AS OF 19/06/2015
#  Analysis of sonar data 
#  for wall following, obstacle detection etc.

###########

import rospy
import numpy
import time
import sys
import signal

from std_msgs.msg import String
from std_msgs.msg import Float32
from hardware_interfaces.msg import sonar_data
from hardware_interfaces.msg import sonar	

from hardware_interfaces import sonar_analyse

################################################################

def get_sonar(msgData):
    """ add sonar data to sonar object as it comes in from ROS messages as *msgData*
    """
    sonar.add_message(msgData)
    detect()

def DR_callback(data):
    """ store pitch data, currently unused
    #TODO this should not use global variables!
    """
    global pitch    
    pitch  = data.pitch


def detect():
    result = sonar.detect_obstacle(0)  # 0 needs to be replaced with pitch if sonar is not run from fixed mount
    pub.publish(transBearing = result[0], pitch = result[1], TargetRange = result[2], meanIntinsity = result[3])
    print(result)
    time.sleep(0.01)
 

################################################################
################################################################
if __name__ == '__main__':

    global pub
    global pitch

    rospy.init_node('sonar_detect')
    # Uncomment to run this node in debug mode:
    #rospy.init_node('sonar_detect', log_level=rospy.DEBUG)
    rospy.Subscriber('sonar_output', String, get_sonar)
    # rospy.Subscriber('dead_reckoner', dead_reckoner, DR_callback)
    print("la")
    pub = rospy.Publisher('sonar_processed', sonar_data)
    sonar = sonar_analyse.sonar()
    rospy.spin()

   
