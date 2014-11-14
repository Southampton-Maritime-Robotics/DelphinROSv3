#!/usr/bin/env python

import roslib; roslib.load_manifest('delphin2_mission')
import rospy
import numpy
import smach
import smach_ros
import time

class actions(smach.State):
    def __init__(self, lib):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller = lib
        self.delay_thruster = 0 # allow the vehicle to gain a speed (value is specified in second) 
        self.delay_action = self.delay_thruster+5000 # let the vehicle doing those actions for a period of time (value is specified in second)
            
    def execute(self, userdata):
        outcome = 'aborted' # set exit flag to aborted by default
            
        ####################################################################
        ### Perform actions ################################################
        ####################################################################
        
        # initialise a reference time
        time_zero=time.time()
        # apply a setpoint to a relevant actuator
        self.__controller.setRearProp(0)
        self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
        self.__controller.setArduinoThrusterVertical(0,0) # (FrontVer,RearVer)
        self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
        time.sleep(self.delay_thruster) # allow the vehicle to gain a speed (delay is specified in second)
####        self.__controller.setArduinoThrusterVertical(0,0) # (FrontVer,RearVer)
####        self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
        
        # let the vehicle doing those actions for a period of time
        # and shutdown the actuators once time's up
        while not rospy.is_shutdown():
        
            if (time.time()-time_zero)<self.delay_action: # in second
#                pass
#                self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
#                self.__controller.setArduinoThrusterVertical(0,10) # (FrontVer,RearVer)
#                self.__controller.setArduinoThrusterHorizontal(0,10) # (FrontHor,RearHor)
                self.__controller.setHeading(128) # specified in a range of [0 360) degree
            else:
                # stop all the actuators
                self.__controller.setRearProp(0)
                self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
                self.__controller.setArduinoThrusterVertical(0,0) # (FrontVer,RearVer)
                self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
                outcome = 'succeeded' # exit with a flag of 'succeeded'
                
        return outcome
