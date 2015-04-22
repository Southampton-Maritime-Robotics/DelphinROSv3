#!/usr/bin/env python

import rospy
import numpy
import smach
import smach_ros
import time

class depthPitchControlTest(smach.State):
    def __init__(self, lib):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller = lib
        self.delay_thruster = 0 # allow the vehicle to gain a speed (value is specified in second) 
        self.delay_action = self.delay_thruster+30 # let the vehicle doing those actions for a period of time (value is specified in second)
            
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
        
        # let the vehicle doing those actions for a period of time
        # and shutdown the actuators once finished           
        time_zero=time.time()
        while not rospy.is_shutdown() and (time.time()-time_zero)<self.delay_action: # in second
            
            # check if the AUV is overdepth
            if self.__controller.getBackSeatErrorFlag():
                print "over-depth detected"
                    # stop all the actuators
                self.__controller.setRearProp(0)
                self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
                self.__controller.setArduinoThrusterVertical(0,0) # (FrontVer,RearVer)
                self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
                return 'aborted'
            
            self.__controller.setDepth(0.5) # specified depth demand in [metre]
            self.__controller.setPitch(0) # specified pitch demand in [degree] 
            self.__controller.setHeading(300)
        
        # stop all the actuators
        self.__controller.setRearProp(0)
        self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
        self.__controller.setArduinoThrusterVertical(0,0) # (FrontVer,RearVer)
        self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
        return 'succeeded' # exit with a flag of 'succeeded'
