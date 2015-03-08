#!/usr/bin/env python

# This state is to help determine a center of rotation on the vertical plane by varying 'cr'
# It is expected that a prover value of 'cr' will make the AUV drive down aith a zero pitch angle

import roslib; roslib.load_manifest('delphin2_mission')
import rospy
import numpy
import smach
import smach_ros
import time

class heaveCenterOfRotation(smach.State):
    def __init__(self, lib):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller = lib
        self.delay_thruster = 0 # allow the vehicle to gain a speed (value is specified in second) 
        self.delay_action = self.delay_thruster+180 # let the vehicle doing those actions for a period of time (value is specified in second)
            
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
        
        # determine relative arm lengths for thrust allocation
        global Ltf
        global Ltr
        L_th = 1.06         # distance between two vertical thrusters [metre]: measured
        cr = 1.15            # center of rotation on vertical plane from the AUV nose [metre]: trial-and-error
        Ltf_nose = 0.28    # location of the front vertical thruster from nose: measured
        Ltf = cr-Ltf_nose   # Moment arm of front vertical thruster from the cr [metre]
        Ltr = L_th-Ltf      # Moment arm of rear vertical thruster from the cr [metre]
        
        ## dictribute thruster demand to each thruster according to the relative arm length
        thrusterDemand = 100000
        
        thruster0 = float(thrusterDemand)/float(Ltf)
        thruster1 = float(thrusterDemand)/float(Ltr)
        
        print Ltf, Ltr
        
        thruster0 = int(numpy.sign(thruster0)*(numpy.abs(thruster0))**0.5) # according to a relationship between thrust and rpm
        thruster1 = int(numpy.sign(thruster1)*(numpy.abs(thruster1))**0.5) # according to a relationship between thrust and rpm
        
        sign = 1
        
        time_zero=time.time()
        
        while not rospy.is_shutdown() and (time.time()-time_zero)<self.delay_action: # in second
            
            # check if the AUV is overdepth
            if self.__controller.getBackSeatErrorFlag():
                    # stop all the actuators
                self.__controller.setRearProp(0)
                self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
                self.__controller.setArduinoThrusterVertical(0,0) # (FrontVer,RearVer)
                self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
                return 'aborted'
                
#            if current_depth > 0.3:
#                sign = -1
                
            self.__controller.setArduinoThrusterVertical(sign*thruster0,sign*thruster1)#'''sign*thruster0'''
            
        # stop all the actuators
        self.__controller.setRearProp(0)
        self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
        self.__controller.setArduinoThrusterVertical(0,0) # (FrontVer,RearVer)
        self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
        return = 'succeeded' # exit with a flag of 'succeeded'                
        
        return outcome
