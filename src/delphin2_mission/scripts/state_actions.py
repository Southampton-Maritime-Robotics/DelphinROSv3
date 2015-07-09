#!/usr/bin/env python

'''
A state that directly operates the actuators in a certain way for a certain period of time.

user needs to specify
-self.delay_action: how long the action will be held
-self.__controller.setRearProp(0)
-self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
-self.__controller.setArduinoThrusterVertical(0,0) # (FrontVer,RearVer)
-self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
-more commands are available in library_highlevel

'''

import rospy
import numpy
import smach
import smach_ros
import time
from std_msgs.msg import String

class actions(smach.State):
    def __init__(self, lib, controlRate):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller = lib
        self.__delay_action = 12 # let the vehicle doing those actions for a period of time (value is specified in second)
        self.__controlRate = controlRate
            
    def execute(self, userdata):

        #Set Up Publisher for Mission Control Log
        pubMissionLog = rospy.Publisher('MissionStrings', String)

        ####################################################################
        ### Perform actions ################################################
        ####################################################################

        # apply a setpoint to a relevant actuator
        self.__controller.setRearProp(0)
        self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
        self.__controller.setArduinoThrusterVertical(0,0) # (FrontVer,RearVer)
        self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
        time.sleep(self.delay_thruster) # allow the vehicle to gain a speed (delay is specified in second)
        
        # let the vehicle doing those actions for a period of time
        # and shutdown the actuators once finished
        
        controlPeriod = 1./self.__controlRate
        r = rospy.Rate(self.__controlRate)
        
        time.sleep(20)

        while not rospy.is_shutdown() and time.time()-timeStart < self.__delay_action:
            if self.__controller.getBackSeatErrorFlag() == 1:
                str= 'state_actions preempted at time = %s' %(time.time())    
                rospy.loginfo(str)
                pubMissionLog.publish(str)
                return 'preempted'
            else:
                self.__controller.setRearProp(0)
                self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
                self.__controller.setArduinoThrusterVertical(0,0) # (FrontVer,RearVer)
                self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
                r.sleep()
                pass
        
        # stop all the actuators
        self.__controller.setRearProp(0)
        self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
        self.__controller.setArduinoThrusterVertical(0,0) # (FrontVer,RearVer)
        self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
        return 'succeeded' # exit with a flag of 'succeeded'
