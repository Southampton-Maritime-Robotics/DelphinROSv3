#!/usr/bin/env python

'''
Get the AUV moving forward/backward for a given period of time.

@return: preempted: if the backSeatErrorFlag has been raised
@return: succeeded: if the timeout criteria has been reached (for other states, timeout will lead to mission aborted)
@return: aborted: not in use

'''

import rospy
import smach
import smach_ros
import time
from std_msgs.msg import String

class GoForwards(smach.State):
    def __init__(self, lib, demandProp, timeout):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller           = lib
        self.__propDemand           = demandProp
        self.__timeout              = timeout
        self.__controlRate          = 5 # [Hz]
            
    def execute(self,userdata):
        
        #Set Up Publisher for Mission Control Log
        pubMissionLog = rospy.Publisher('MissionStrings', String)

        # Set Up Loop Timing Control
        r = rospy.Rate(self.__controlRate)

        str='Execute GoForwards State: propDemand = %s; action hold %ss' %(self.__propDemand, self.__timeout)
        pubMissionLog.publish(str)
        rospy.loginfo(str)
        
        timeStart = time.time()
        while not rospy.is_shutdown() and self.__controller.getBackSeatErrorFlag() == 0 and time.time()-timeStart < self.__timeout:
            self.__controller.setRearProp(self.__propDemand)
            r.sleep()
        else:
            self.__controller.setRearProp(0)

        if self.__controller.getBackSeatErrorFlag() == 1:
            str= 'goForwards preempted at time = %s' %(time.time())
            pubMissionLog.publish(str)
            rospy.loginfo(str)
            return 'preempted'
        else: # timeout means succeed in this state
            str= 'goForwards succeeded at time = %s' %(time.time())
            pubMissionLog.publish(str)
            rospy.loginfo(str)
            return 'succeeded'
