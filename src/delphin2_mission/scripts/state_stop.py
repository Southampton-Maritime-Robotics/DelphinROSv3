#!/usr/bin/env python

'''
A state to stop all the actuators or set to a neutral position before the mission is terminated.

Stop class
@param: smach.State - doesn't need to be passed by calling method - only used internally

__init__ - initialises a Stop object
execute - calls the stop() method in library_highlevel.py, ending the current mission and ROS session.

'''

import rospy
import smach
import smach_ros
import time
from std_msgs.msg import String

class Stop(smach.State):
    def __init__(self, lib):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.__controller = lib
                   		
    def execute(self,userdata):
		#SYTEMS STOP
		
        #Set Up Publisher for Mission Control Log
        pub = rospy.Publisher('MissionStrings', String, queue_size=10)
        
        text= 'Delphin2 STOP state started at time = %s' %(time.time())
        pub.publish(text)
        
        self.__controller.stop()
        return 'succeeded'
