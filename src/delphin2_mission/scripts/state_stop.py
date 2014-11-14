#!/usr/bin/env python

import roslib; roslib.load_manifest('delphin2_mission')
import rospy
import smach
import smach_ros
import time
from std_msgs.msg import String

# Stop class
# @param: smach.State - doesn't need to be passed by calling method - only used internally
#
# __init__ - initialises a Stop object
# execute - calls the stop() method in library_highlevel.py, ending the current mission and ROS session.

class Stop(smach.State):
	def __init__(self, lib):
		smach.State.__init__(self, outcomes=['succeeded'])
                self.__controller = lib
                   		
	def execute(self,userdata):
		#SYTEMS STOP
                global pub
                #Set Up Publisher for Mission Control Log
                pub = rospy.Publisher('MissionStrings', String)
                
                str= 'Delphin2 STOP state started at time = %s' %(time.time())
                pub.publish(str)
                
                self.__controller.stop()
                return 'succeeded'
