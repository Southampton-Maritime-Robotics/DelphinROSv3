#!/usr/bin/env python

import rospy
import numpy
import smach
import smach_ros
import time
from std_msgs.msg import String

class testDepthPitchMPC(smach.State):
    def __init__(self, lib):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller = lib
        self.delay_action = 120 # let the vehicle doing those actions for a period of time (value is specified in second)
        
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
        
################################################################################
        # let the vehicle do depth-pitch tracking

        listDemandDepth = [1.0,2.0,1.0]
        demandHeading = 270.
        demandPitch = 0.

        controlRate = 20 # Hz
        r = rospy.Rate(controlRate)

        for demandDepth in listDemandDepth:

            str = "tracking a depth demand of = %s" %(demandDepth)
            rospy.loginfo(str)
            pub.publish(str)
            
            # initialise a reference time
            time_zero=time.time()
            while not rospy.is_shutdown() and (time.time()-time_zero)<self.delay_action: # in second
                
                # check if the AUV is overdepth
                if self.__controller.getBackSeatErrorFlag():
                    str = "backSeatErrorFlag is raised"
                    rospy.loginfo(str)
                    pub.publish(str)
                    return 'preempted'
                
                # assign the demands
                self.__controller.setDepth(demandDepth) # specified depth demand in [metre]
                self.__controller.setPitch(demandPitch) # specified pitch demand in [degree] 
                self.__controller.setHeading(demandHeading)
                
                r.sleep()
                
################################################################################
        # stop all the actuators before leave the state
        self.__controller.setRearProp(0)
        self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
        self.__controller.setArduinoThrusterVertical(0,0) # (FrontVer,RearVer)
        self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
        return 'succeeded' # exit with a flag of 'succeeded'
