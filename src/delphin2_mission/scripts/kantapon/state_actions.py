#!/usr/bin/env python

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
        self.delay_action = self.delay_thruster+900 # let the vehicle doing those actions for a period of time (value is specified in second)
            
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
        
        flagTest = True
        timeStart = time.time()
        
        nSamp = 500
        a = numpy.zeros([nSamp])
        while not rospy.is_shutdown() and (time.time()-time_zero)<self.delay_action: # in second
            timeRef = time.time()
            while time.time()-timeRef<3:
                self.__controller.setArduinoThrusterVertical(100,100) # (FrontVer,RearVer)
            timeRef = time.time()
            while time.time()-timeRef<3:
                self.__controller.setArduinoThrusterVertical(200,200) # (FrontVer,RearVer)
#            self.__controller.setRearProp(22)
#            a = numpy.append(a[1:nSamp],[self.__controller.getPropRPM()])
#            print "mean: ", numpy.mean(a), "current: ", self.__controller.getPropRPM()

#            self.__controller.setArduinoThrusterHorizontal(-100,-200) # (FrontHor,RearHor)            
            
#            if time.time()-timeStart < 10:
##                self.__controller.setHeading(-20)
#                self.__controller.setDepth(2)
##                self.__controller.setRearProp(15)
##                self.__controller.setControlSurfaceAngle(-30,-30,-30,-30) # (VerUp,HorRight,VerDown,HorLeft)
        
        # stop all the actuators
        self.__controller.setRearProp(0)
        self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
        self.__controller.setArduinoThrusterVertical(0,0) # (FrontVer,RearVer)
        self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
        return 'succeeded' # exit with a flag of 'succeeded'
