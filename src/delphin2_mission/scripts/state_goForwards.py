#!/usr/bin/env python

'''
Get the AUV moving forward/backward for a given period of time.

@return: preempted: if the backSeatErrorFlag has been raised
@return: succeeded: if the timeout criteria has been reached (for other states, timeout will lead to mission aborted)
@return: aborted: if other node that has a higher priority and running in paraller has finished

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

        text='Execute GoForwards State: propDemand = %s; action hold %ss' %(self.__propDemand, self.__timeout)
        pubMissionLog.publish(text)
        rospy.loginfo(text)
        
        timeStart = time.time()
        while not rospy.is_shutdown() and self.__controller.getBackSeatErrorFlag() == 0 and time.time()-timeStart < self.__timeout:
            if self.preempt_requested():
                text = "Force Exit GoForwards!!!" + str(self.__controller.getBackSeatErrorFlag()) + "preemept requested:" + str(self.preempt_requested())
                pubMissionLog.publish(text)
                rospy.loginfo(text)
                self.service_preempt()
                return 'aborted'
            self.__controller.setRearProp(self.__propDemand)
            r.sleep()
        else:
            self.__controller.setRearProp(0)

        if self.__controller.getBackSeatErrorFlag() == 1:
            text= 'goForwards preempted'
            pubMissionLog.publish(text)
            rospy.loginfo(text)
            return 'preempted'
        else: # timeout means succeed in this state
            text= 'goForwards succeeded'
            pubMissionLog.publish(text)
            rospy.loginfo(text)
            return 'succeeded'
