#!/usr/bin/env python

'''
######################################
# description
A state for surge manoeuvre

AUV will be moving back and forth between two waypoints with a certain propeller demand.

If the depth demand is less than 0.5m, it will accaount as no depth demand specified.

Procedure:
    go to one reference point
    head to the other reference point
    descend to a desired depth
    move with a particular propeller demand
    stop and ascend to the surface
    
######################################
#Modifications
19/May/2015: make the AUV capable of performing an action at depth
28/May/2015: included the backSeatErrorFlag as a terminating criteria
29/May/2015: keep the AUV heading during descent

@return: preemped: if the backSeatErrorFlag has been raised
@return: succeeded: if the AUV execute all the propeller demands in a given list

'''

import rospy
import numpy
import smach
import smach_ros
import time
from pylab import *
from math import *
from std_msgs.msg import String

class manoeuvreSurge(smach.State):
    def __init__(self, lib, myUti, wp, uGain, uMax, errHeadingTol, wp_R, timeDelay, depthDemand, depthTol, depthDemandMin, controlRate):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller = lib
        self.__uti = myUti
        self.__wp = wp
        self.__uGain = uGain
        self.__uMax = uMax
        self.__errHeadingTol = errHeadingTol
        self.__wp_R = wp_R
        self.__timeDelay = timeDelay # [sec]. to let the motion decays
        self.__depthDemand = depthDemand # [m].
        self.__depthTol = depthTol # [m]. It is account as the AUV get to the depth if the depth error is less than this.
        self.__depthDemandMin = depthDemandMin # [m] if the depthDemand is less than this, it is accounted as no depth demand specified.
        self.__listProp = [10,13,16,19,22] # list of propeller demand used in experiment
        self.__controlRate = controlRate

    def execute(self, userdata):
        
        ####################################################################
        ### Perform actions ################################################
        ####################################################################

        wpRang,_ = self.__uti.rangeBearing([self.__wp[0][0], self.__wp[1][0]],[self.__wp[0][1], self.__wp[1][1]]) # determine a range between waypoints as a reference
        wpIndex = 0

        if self.__controlRate>0:
            r = rospy.Rate(self.__controlRate)
        
        #Set Up Publisher for Mission Control Log
        pubMissionLog = rospy.Publisher('MissionStrings', String)

        for demandProp in self.__listProp:
        
            if rospy.is_shutdown() or self.__controller.getBackSeatErrorFlag() == 1:
                break
            
            # go to the waypoint
            str = 'go to start point: %s' %self.__wp[:,wpIndex]
            rospy.loginfo(str)
            pubMissionLog.publish(str)
            while not rospy.is_shutdown() and self.__controller.getBackSeatErrorFlag() == 0:
                
                X = self.__controller.getX()
                Y = self.__controller.getY()
                heading = self.__controller.getHeading()
                rang, bear = self.__uti.rangeBearing([X,Y], [self.__wp[0][wpIndex], self.__wp[1][wpIndex]])
                if rang >= self.__wp_R:
                    errHeading = self.__uti.computeHeadingError(bear,heading)
                    u = self.__uti.surgeVelFromHeadingError(self.__uMax,self.__uGain,errHeading)
                    self.__controller.setRearProp(round(u*22.)) # turn speedDemand into propeller demand and send
                    self.__controller.setHeading(bear)
                else:
                    # when the AUV get to the start point, move onto the next step
                    self.__controller.setRearProp(0)
                    self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
                    self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
                    break
                    
                if self.__controlRate>0:
                    r.sleep()

            # point toward anoter waypoint
            str = 'head toward the target: %s' %self.__wp[:,1-wpIndex]
            rospy.loginfo(str)
            pubMissionLog.publish(str)
            timeStart = time.time()
            while not rospy.is_shutdown() and self.__controller.getBackSeatErrorFlag() == 0:
            
                X = self.__controller.getX()
                Y = self.__controller.getY()
                heading = self.__controller.getHeading()
                rang, bear = self.__uti.rangeBearing([X,Y], [self.__wp[0][1-wpIndex], self.__wp[1][1-wpIndex]])
                errHeading = self.__uti.computeHeadingError(bear,heading)
                if abs(errHeading)>self.__errHeadingTol:
                    timeStart = time.time()
                if time.time()-timeStart <= self.__timeDelay:
                    self.__controller.setHeading(bear)
                else:
                    # when the heading is steady, move onto the next step
                    self.__controller.setRearProp(0)
                    self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
                    self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
                    break
                    
                if self.__controlRate>0:
                    r.sleep()
                    
            # bring the AUV to depth if the depth demand is specified
            if self.__depthDemand>=self.__depthDemandMin:
                str = 'descend to a depth of %sm' % self.__depthDemand
                rospy.loginfo(str)
                pubMissionLog.publish(str)
                timeStart = time.time()
                while not rospy.is_shutdown() and self.__controller.getBackSeatErrorFlag() == 0:
                
                    if abs(self.__controller.getDepth()-self.__depthDemand)>self.__depthTol:
                        timeStart = time.time() # reset the reference time
                    if time.time()-timeStart <= self.__timeDelay:
                        self.__controller.setDepth(self.__depthDemand)
                        self.__controller.setHeading(bear)
                    else:
                        # if the AUV get to the depth and stay there long enough, move onto the next step
                        str = 'steady at desired depth'
                        rospy.loginfo(str)
                        pubMissionLog.publish(str)
                        break
                    
                    if self.__controlRate>0:
                        r.sleep()

            # set demandProp
            str = 'apply propeller demand = %s' %(demandProp)    
            rospy.loginfo(str)
            pubMissionLog.publish(str)
            while not rospy.is_shutdown() and self.__controller.getBackSeatErrorFlag() == 0:
            
                X = self.__controller.getX()
                Y = self.__controller.getY()
                rang, bear = self.__uti.rangeBearing([X,Y], [self.__wp[0][wpIndex], self.__wp[1][wpIndex]])
                
                if abs(wpRang-rang) >= self.__wp_R:
                    self.__controller.setRearProp(demandProp)
                    if self.__depthDemand>=self.__depthDemandMin:
                        self.__controller.setDepth(self.__depthDemand)
                else:
                    # if the AUV travel far enough, move onto the next step
                    self.__controller.setRearProp(0)
                    self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
                    self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
                    if self.__depthDemand>=self.__depthDemandMin:
                        self.__controller.setDepth(0)
                    break
                    
                if self.__controlRate>0:
                    r.sleep()

            # vehicle will stop for this many second as to let the AUV ascend to the surface
            if self.__depthDemand>=self.__depthDemandMin:
                self.__controller.setDepth(0) # by defult, the depth controller will turn off on its own after 1sec of not reciving new demand
                time.sleep(self.__timeDelay)
            
            # switch the role of two reference waypoints
            wpIndex = 1-wpIndex
            
        return 'succeeded' # exit with a flag of 'succeeded'
        
        
        if self.__controller.getBackSeatErrorFlag() == 1:
            str= 'manoeuvreSurge preempted at time = %s' %(time.time())    
            rospy.loginfo(str)
            pubMissionLog.publish(str)
            return 'preempted'
        else:
            str= 'manoeuvreSurge succeed at time = %s' %(time.time())    
            rospy.loginfo(str)
            pubMissionLog.publish(str)
            return 'succeeded'
