#!/usr/bin/env python

'''
######################################
# description
A state for a yaw manoeuvre by using horizontal thrusters.

A script to get a reference point (i.e. middle of the lake) and let it executes the thruster demand (rudder is set to amidships and propeller is not running).

If the depth demand is less than 0.5m, it will accaount as no depth demand specified.

Procedure:
    go to a reference point
    descend to a desired depth
    execute thruster setpoint
    stop and ascend to the surface
    
######################################
#Modifications
20/May/2015: make the AUV capable of performing an action at depth
28/May/2015: included the backSeatErrorFlag as a terminating criteria
29/May/2015: keep the AUV heading during descent

@return: preemped: if the backSeatErrorFlag has been raised
@return: succeeded: if the AUV execute all the thruster demands in a given list

'''

import rospy
import numpy
import smach
import smach_ros
import time
from pylab import *
from math import *
from std_msgs.msg import String

class manoeuvreYaw(smach.State):
    def __init__(self, lib):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller = lib
        self.__timeDemandHold = 60 # thruster demand will be hold for this many second
        self.__timeDepthSteady = 60 # to let the depth becomes steady
        self.__depthDemand = 1 # [m].
        self.__depthTol = 0.2 # [m]. It is account as the AUV get to the depth if the depth error is less than this.
        self.__depthDemandMin = 0.5 # [m] if the depthDemand is less than this, it is accounted as no depth demand specified.
        self.__thrusterDemand = 2000 # [100, 200, 400, 700, 1400, 2100] # list of thruster demand used in experiment
        self.__direction = 1 # 1:cw, -1:ccw 
        self.__controlRate = 20.
        self.__headingDemand = 100.

    def execute(self, userdata):
    
        #Set Up Publisher for Mission Control Log
        pubMissionLog = rospy.Publisher('MissionStrings', String)
        
        ####################################################################
        ### Perform actions ################################################
        ####################################################################
        
        if self.__controlRate>0:
            r = rospy.Rate(self.__controlRate)
            
        # bring the AUV to depth if the depth demand is specified
        if self.__depthDemand>=self.__depthDemandMin:
            str = 'descend to a depth of %sm' % self.__depthDemand
            rospy.loginfo(str)
            pubMissionLog.publish(str)
            timeStart = time.time()
            flag = 1
            while not rospy.is_shutdown() and self.__controller.getBackSeatErrorFlag() == 0:
            
                if abs(self.__controller.getDepth()-self.__depthDemand)>self.__depthTol:
                    timeStart = time.time() # reset the reference time
                if time.time()-timeStart <= self.__timeDepthSteady:
                    if flag:
                        str = 'let the depth becomes stady for timeDepthSteady = %s sec' %self.__timeDepthSteady
                        rospy.loginfo(str)
                        pubMissionLog.publish(str)
                        flag = 0
                    self.__controller.setHeading(self.__headingDemand) # specified in a range of [0 360) degree
                    self.__controller.setDepth(self.__depthDemand)
                else:
                    # if the AUV get to the depth and stay there long enough, move onto the next step
                    str = 'steady at desired depth'
                    rospy.loginfo(str)
                    pubMissionLog.publish(str)
                    break
                    
                if self.__controlRate>0:
                    r.sleep()

        # set thruster demand
        str = 'execute thruster with demand = %s' %(self.__thrusterDemand)
        rospy.loginfo(str)
        pubMissionLog.publish(str)
        timeStart = time.time()
        flag = 1
        while not rospy.is_shutdown() and self.__controller.getBackSeatErrorFlag() == 0:
        
            if time.time()-timeStart < self.__timeDemandHold:
                
                if flag:
                    str = 'hold the thruster demand for timeDemandHold = %s sec' %self.__timeDemandHold
                    rospy.loginfo(str)
                    pubMissionLog.publish(str)
                    flag = 0
         
                self.__controller.setArduinoThrusterHorizontal(self.__direction*self.__thrusterDemand,-self.__direction*self.__thrusterDemand)
                
                if self.__depthDemand>=self.__depthDemandMin:
                    self.__controller.setDepth(self.__depthDemand)
            else:
                self.__controller.setRearProp(0)
                self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
                self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
                if self.__depthDemand>=self.__depthDemandMin:
                    self.__controller.setDepth(0)
                break
                
            if self.__controlRate>0:
                r.sleep()
                                
        if self.__controller.getBackSeatErrorFlag() == 1:
            str= 'manoeuvreYaw preempted at time = %s' %(time.time())    
            rospy.loginfo(str)
            pubMissionLog.publish(str)
            return 'preempted'
        else:
            str= 'manoeuvreYaw succeed at time = %s' %(time.time())    
            rospy.loginfo(str)
            pubMissionLog.publish(str)
            return 'succeeded'
