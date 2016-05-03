#!/usr/bin/env python

'''
Get the AUV to turn by using either horizontal thruster or rudder, or using both set of actuators simultaneously.

If the demand for a particular set of actuators is zero, the demand of this set will not be published !!!

@return: preempted: if the backSeatErrorFlag has been raised
@return: succeeded: if the timeout criteria has been reached (for other states, timeout will lead to mission aborted)
@return: aborted: if other node that has a higher priority and running in paraller has finished

'''

import rospy
import smach
import smach_ros
import time
from std_msgs.msg import String

class GoTurning(smach.State):
    def __init__(self, lib, demand_th_hor, demand_cs_ver, timeout):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller           = lib
        self.__th_hor_frt           = demand_th_hor
        self.__th_hor_aft           = - self.__th_hor_frt
        self.__rudder               = demand_cs_ver
        self.__timeout              = timeout
        self.__controlRate          = 5 # [Hz]
            
    def execute(self,userdata):
        
        #Set Up Publisher for Mission Control Log
        pubMissionLog = rospy.Publisher('MissionStrings', String)

        # Set Up Loop Timing Control
        r = rospy.Rate(self.__controlRate)

        str='Execute GoTurning State: th_hor = %s; rudder = %s; action hold %ss' %(self.__th_hor_frt, self.__rudder, self.__timeout)
        pubMissionLog.publish(str)
        rospy.loginfo(str)
        
        timeStart = time.time()
        while not rospy.is_shutdown() and self.__controller.getBackSeatErrorFlag() == 0 and time.time()-timeStart < self.__timeout:
            if self.preempt_requested():
                str = "Force Exit GoTurning!!!"
                pubMissionLog.publish(str)
                rospy.loginfo(str)
                self.service_preempt()
                return 'aborted'
                
            if self.__th_hor_frt != 0 or self.__th_hor_aft != 0:
                self.__controller.setArduinoThrusterHorizontal(self.__th_hor_frt,self.__th_hor_aft) # (FrontHor,RearHor)
            if self.__rudder != 0:
                self.__controller.setRudderAngle(self.__rudder)
            r.sleep()
        else:
            self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
            self.__controller.setRudderAngle(0)
            
        if self.__controller.getBackSeatErrorFlag() == 1:
            str= 'GoTurning preempted'
            pubMissionLog.publish(str)
            rospy.loginfo(str)
            return 'preempted'
        else: # timeout means succeed in this state
            str= 'GoTurning succeeded'
            pubMissionLog.publish(str)
            rospy.loginfo(str)
            return 'succeeded'
