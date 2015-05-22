#!/usr/bin/env python

'''
A state that directly operates the virtual actuators for a period of time. When timeout, the actuator demands will be set to zero.

user needs to specify
-self.delay_action: how long the action will be held
-self.__controller.setRearProp(0)
-self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
-self.__controller.setArduinoThrusterVertical(0,0) # (FrontVer,RearVer)
-self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)

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

#            self.__controller.setRearProp(25)
            self.__controller.sway(0.2) # (this function actually requires tsl_setpoints ~500-1000, however here requires sway speed in m/s)
#            flagCon = 1
#            while flagCon:
#                self.__controller.setHeading(30)
#                self.__controller.setRearProp(10)
#                self.__controller.setDepth(2)
#                if self.__controller.getX()>1000:
#                    flagCon = 0
#                        
#            flagCon = 1
#            while flagCon:
#                self.__controller.setHeading(210)
#                self.__controller.setRearProp(14)
#                self.__controller.setDepth(2)
#                if self.__controller.getX<0:
#                    flagCon = 0

        return 'succeeded' # exit with a flag of 'succeeded'
