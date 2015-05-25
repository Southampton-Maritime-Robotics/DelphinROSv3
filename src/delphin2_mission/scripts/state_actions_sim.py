#!/usr/bin/env python

'''
A state that directly operates the virtual actuators in a certain way for a period of time.

user needs to specify
-self.delay_action: how long the action will be held
-self.__controller.setDepth(0)
-self.__controller.setHeading(0)
-self.__controller.setRearProp(0)
-self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
-self.__controller.setArduinoThrusterVertical(0,0) # (FrontVer,RearVer)
-self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)

note: if the actuator demand is assigned directly, the setHeading must be commented

See "auv_sim.py" for how to interact with this virtual vehicle.

'''

import rospy
import numpy
import smach
import smach_ros
import time

class actions(smach.State):
    def __init__(self, lib):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller = lib

    def execute(self, userdata):
        
        ####################################################################
        ### Perform actions ################################################
        ####################################################################

        time_zero=time.time()
        while not rospy.is_shutdown():

#            self.__controller.setHeading(0)
            self.__controller.setRearProp(0)
            self.__controller.setDepth(0)
            self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
            self.__controller.setArduinoThrusterHorizontal(1000,-1000) # (FrontHor,RearHor)

        return 'succeeded' # exit with a flag of 'succeeded'
