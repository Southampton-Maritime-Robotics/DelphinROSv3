#!/usr/bin/python

'''
Possible, a part of the sonar driver.

This node may not functioning!

'''

import rospy
import sys
import serial
import time
import numpy
from std_msgs.msg import String

# Import required data types
from DelphinROS.msg import sonar

def sonarTalker():
    print "Sending new parameters"
    pub.publish(LLim=3200,RLim=3200)

   
if __name__ == '__main__':
    rospy.init_node('sonar_updateParams')
    pub = rospy.Publisher('sonar_update', sonar)
    time.sleep(0.25)
    #while not rospy.is_shutdown():
    sonarTalker()
    time.sleep(3)
    #rospy.spin()
