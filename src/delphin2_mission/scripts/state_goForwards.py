#!/usr/bin/env python

'''
Get the AUV moving forward/backward.

@return: preemped: if the backSeatErrorFlag has been raised
@return: succeeded: if the timeout criteria has been reached
@return: aborted: not in use

# TODO
- may need to assign the depthDemand at the same time

'''

import rospy
import smach
import smach_ros
import time
from std_msgs.msg import String

class GoForwards(smach.State):
    def __init__(self, lib, myUti, timeout, propDemand, controlRate):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller           = lib
        self.__uti                  = myUti
        self.__timeout              = timeout
        self.__propDemand           = propDemand
        self.__controlRate          = controlRate
            
    def execute(self,userdata):
        
        #Set Up Publisher for Mission Control Log
        pubMissionLog = rospy.Publisher('MissionStrings', String)
        
        if self.__controlRate>0:
            r = rospy.Rate(self.__controlRate)

        str='Entered GoForwards State with a propDemand = %s' %(self.__propDemand)
        pubMissionLog.publish(str)
        rospy.loginfo(str)
        
        timeStart = time.time()
        while not rospy.is_shutdown() and self.__controller.getBackSeatErrorFlag() == 0:
            if time.time()-timeStart < self.__timeout:
                self.__controller.setRearProp(self.__propDemand)
            else:
                self.__controller.setRearProp(0)
                str= 'goForwards succeeded at time = %s' %(time.time())
                pubMissionLog.publish(str)
                rospy.loginfo(str)
                return 'succeeded'
                
            if self.__controlRate>0:
                r.sleep()
                
        self.__controller.setRearProp(0)
        
        if self.__controller.getBackSeatErrorFlag() == 1:
            str= 'goForwards preempted at time = %s' %(time.time())
            pubMissionLog.publish(str)
            rospy.loginfo(str)
            return 'preempted'
        else:
            str= 'goForwards aborted at time = %s' %(time.time())
            pubMissionLog.publish(str)
            rospy.loginfo(str)
            return 'aborted'
