#!/usr/bin/env python

'''
A state to test the PID-based depth-pitch controller at forward speeds (see depthPitchPID.py).

# Available commands
-self.__controller.setDepth(0.45) # specified depth demand in [metre]
-self.__controller.setPitch(0) # specified pitch demand in [degree] 

# Specify
-self.delay_action # time span for the AUV to track one depth demand
-listDemandDepth, [metre] e.g. [1.,2.,3.]
-demandHeading, [deg] e.g. 280.
-demandPitch, [deg] e.g. 0.
-timeSteadyDepth
-demandProp

# Notes
heading controller will be functioning along side as to have the AUV heading remain unchanged.

@return: aborted: if BackSeatErrorFlag is raised
@return: successed: tasks accomplished

'''

import rospy
import numpy
import smach
import smach_ros
import time
import math
from std_msgs.msg import String

class testDepthPitchMPCAtSpeed(smach.State):
    def __init__(self, lib):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller = lib

    def execute(self, userdata):

        #Set Up Publisher for Mission Control Log
        pubMissionLog = rospy.Publisher('MissionStrings', String)        
        
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

        demandDepth = 1.0 # [m]
        depthTol = 0.2 # [m] tolerant that counts as approach the demandDepth
        demandHeading = 280. # [deg]
        demandPitch = 0. # [deg]
        demandSpeed = 0.8# [m/s]
        timeDepthSteady = 50. # [sec] the depth is steady for this many second then propeller will start spining
        timePropHold = 80. # [sec] time that propeller will keep spining

        controlRate = 20 # Hz
        r = rospy.Rate(controlRate)
        
        # initialise a reference time
        str = 'descend to a depth of %sm' % demandDepth
        rospy.loginfo(str)
        pubMissionLog.publish(str)
        timeStart = time.time()
        flag = 1
        while not rospy.is_shutdown() and self.__controller.getBackSeatErrorFlag() == 0:
        
            if abs(self.__controller.getDepth()-demandDepth)>depthTol:
                timeStart = time.time() # reset the reference time
            if time.time()-timeStart <= timeDepthSteady:
                self.__controller.setDepth(demandDepth)
                self.__controller.setPitch(demandPitch) # specified pitch demand in [degree] 
                self.__controller.setHeading(demandHeading)
                if flag:
                    str = 'let the depth becomes stady for timeDelay = %s sec' %timeDepthSteady
                    rospy.loginfo(str)
                    pubMissionLog.publish(str)
                    flag = 0
            else:
                # if the AUV get to the depth and stay there long enough, move onto the next step
                str = 'steady at desired depth'
                rospy.loginfo(str)
                pubMissionLog.publish(str)
                break
            r.sleep()

        # set demandSpeed
        str = 'apply speed demand = %s' %(demandSpeed)    
        rospy.loginfo(str)
        pubMissionLog.publish(str)
        timeStart = time.time()
        
        while not rospy.is_shutdown() and self.__controller.getBackSeatErrorFlag() == 0:
            
            self.__controller.setSpeed(demandSpeed)
            self.__controller.setDepth(demandDepth)
            self.__controller.setPitch(demandPitch)
            self.__controller.setHeading(demandHeading)
            if time.time()-timeStart>timePropHold:
                break
            r.sleep()
                        
####        # let the AUV maintains the depth after the propeller stop spining
####        str = 'maintain at depth of %sm' % demandDepth
####        rospy.loginfo(str)
####        pubMissionLog.publish(str)
####        timeStart = time.time()
####        flag = 1
####        while not rospy.is_shutdown() and self.__controller.getBackSeatErrorFlag() == 0:
####        
####            if abs(self.__controller.getDepth()-demandDepth)>depthTol:
####                timeStart = time.time() # reset the reference time
####            if time.time()-timeStart <= timeDepthSteady*1:
####                self.__controller.setDepth(demandDepth)
####                self.__controller.setPitch(demandPitch) # specified pitch demand in [degree] 
####                self.__controller.setHeading(demandHeading)
####                self.__controller.setRearProp(0)
####                self.__controller.setSpeed(0)
####                if flag:
####                    str = 'let the depth becomes stady for timeDelay = %s sec' %timeDepthSteady
####                    rospy.loginfo(str)
####                    pubMissionLog.publish(str)
####                    flag = 0
####            else:
####                # if the AUV get to the depth and stay there long enough, move onto the next step
####                str = 'steady at desired depth'
####                rospy.loginfo(str)
####                pubMissionLog.publish(str)
####                break
####            r.sleep()
            
################################################################################
        # stop all the actuators before leave the state
        self.__controller.setRearProp(0)
        self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
        self.__controller.setArduinoThrusterVertical(0,0) # (FrontVer,RearVer)
        self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
        return 'succeeded' # exit with a flag of 'succeeded'
