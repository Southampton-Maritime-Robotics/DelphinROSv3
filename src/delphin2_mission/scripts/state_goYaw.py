#!/usr/bin/env python

'''
Get the AUV turns by using the horizontal thrusters.

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

class GoYaw(smach.State):
    def __init__(self, lib, myUti, timeout, thrusterDemand, controlRate):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller           = lib
        self.__uti                  = myUti
        self.__timeout              = timeout
        self.__thrusterDemand       = thrusterDemand
        self.__controlRate          = controlRate
        
    def execute(self,userdata):
        
        #Set Up Publisher for Mission Control Log
        pubMissionLog = rospy.Publisher('MissionStrings', String)
        
        if self.__controlRate>0:
            r = rospy.Rate(self.__controlRate)

        str='Entered GoYaw State with a thrusterDemand = %s; action hold %ss' %(self.__thrusterDemand, self.__timeout)
        pubMissionLog.publish(str)
        rospy.loginfo(str)
        
        timeStart = time.time()
        while not rospy.is_shutdown() and self.__controller.getBackSeatErrorFlag() == 0:
            if time.time()-timeStart < self.__timeout:
                self.__controller.setArduinoThrusterHorizontal(self.__thrusterDemand,-self.__thrusterDemand) # (FrontHor,RearHor)
            else:
                self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
                str= 'goSideway succeeded at time = %s' %(time.time())
                pubMissionLog.publish(str)
                rospy.loginfo(str)
                return 'succeeded'
                
            if self.__controlRate>0:
                r.sleep()

        self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
        
        if self.__controller.getBackSeatErrorFlag() == 1:
            str= 'goSideway preempted at time = %s' %(time.time())
            pubMissionLog.publish(str)
            rospy.loginfo(str)
            return 'preempted'
        else:
            str= 'goSideway aborted at time = %s' %(time.time())
            pubMissionLog.publish(str)
            rospy.loginfo(str)
            return 'aborted'
