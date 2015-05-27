#!/usr/bin/env python

'''
A state to test the PID-based depth-pitch controller (see depthPitchPID.py).

# Available commands
-self.__controller.setHeading(100) # specified heading demand in [deg]

# Define
-self.delay_action # time span for the AUV to track one heading demand
-listDemandHeading, [deg] e.g. [90,180,270].

@return: preempted: if BackSeatErrorFlag is raised
@return: successed: tasks accomplished

'''

import rospy
import numpy
import smach
import smach_ros
import time
from std_msgs.msg import String

class headingControlTest(smach.State):
    def __init__(self, lib):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller = lib
        self.delay_thruster = 0 # allow the vehicle to gain a speed (value is specified in second) 
        self.delay_action = self.delay_thruster+70 # let the vehicle doing those actions for a period of time (value is specified in second)
            
    def execute(self, userdata):
    
        #Set Up Publisher for Mission Control Log
        pub = rospy.Publisher('MissionStrings', String)
        
        ####################################################################
        ### Perform actions ################################################
        ####################################################################
        
################################################################################
        # ensure all the actuator are turnned-off
        self.__controller.setRearProp(0)
        self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
        self.__controller.setArduinoThrusterVertical(0,0) # (FrontVer,RearVer)
        self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
        time.sleep(self.delay_thruster) # allow the vehicle to gain a speed (delay is specified in second)
        
################################################################################
        # let the vehicle do heading tracking
        listDemandHeading = [110,200,200]
        for demandHeading in listDemandHeading
            # set a reference time
            time_zero=time.time()
            while not rospy.is_shutdown() and (time.time()-time_zero)<self.delay_action: # in second

                # check if the AUV is overdepth
                if self.__controller.getBackSeatErrorFlag():
                    str = "backSeatErrorFlag is raised"
                    rospy.loginfo(str)
                    pub.publish(str)
                    self.__controller.setRearProp(0)
                    self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
                    self.__controller.setArduinoThrusterVertical(0,0) # (FrontVer,RearVer)
                    self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
                    return 'preempted'
                    
                str = "tracking a heading demand of = %s" %(demandHeading)
                rospy.loginfo(str)
                pub.publish(str)
                self.__controller.setHeading(demandHeading) # specified in a range of [0 360) degree
            
################################################################################
        # stop all the actuators before leave the state
        self.__controller.setRearProp(0)
        self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
        self.__controller.setArduinoThrusterVertical(0,0) # (FrontVer,RearVer)
        self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
        outcome = 'succeeded' # exit with a flag of 'succeeded'

        return outcome
