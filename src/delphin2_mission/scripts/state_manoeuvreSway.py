#!/usr/bin/env python

'''
######################################
# description
A state for sway manoeuvre.

AUV will move to a reference point (e.g. middle of the lake). Then it will move sideway by executing a certain thruster demand in a given list.

If the depth demand is less than 0.5m, it will accaount as no depth demand specified.

Procedure
    go to a reference point
    head to point O - approximately [0,0]
    descend to a desired depth
    execute one of the thruster demand from the given list
    stop and ascend to the surface
    
######################################
#Modifications
19/May/2015: make the AUV capable of performing an action at depth
28/May/2015: included the backSeatErrorFlag as a terminating criteria
29/May/2015: keep the AUV heading during descent


@return: preemped: if the backSeatErrorFlag has been raised
@return: aborted: mission timeout
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

class manoeuvreSway(smach.State):
    def __init__(self, lib, myUti, wp, uGain, uMax, errHeadingTol, wp_R, timeDemandHold, timeDelay, depthDemand, depthTol, depthDemandMin, missionTimeout):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller = lib
        self.__uti = myUti
        self.__wp = wp
        self.__uGain = uGain
        self.__uMax = uMax
        self.__errHeadingTol = errHeadingTol
        self.__wp_R = wp_R
        self.__timeDemandHold = timeDemandHold # thruster demand will be hold for this many second
        self.__timeDelay = timeDelay
        self.__depthDemand = depthDemand # [m].
        self.__depthTol = depthTol # [m]. It is account as the AUV get to the depth if the depth error is less than this.
        self.__depthDemandMin = depthDemandMin # [m] if the depthDemand is less than this, it is accounted as no depth demand specified.
        self.__missionTimeout = missionTimeout # [sec] a criterion for mission abort
        self.__listThrusterDemand = [700, 1400, 2100] # [100, 200, 400, 700, 1400, 2100] # list of thruster demand used in experiment
        self.__direction = 1 # 1:right, -1:left

    def execute(self, userdata):
        
        #Set Up Publisher for Mission Control Log
        pubMissionLog = rospy.Publisher('MissionStrings', String)
        
        ####################################################################
        ### Perform actions ################################################
        ####################################################################

        # execute sway motion response test
        for demandThruster in self.__listThrusterDemand:
        
            if rospy.is_shutdown() or time.time()-timeZero >= self.__missionTimeout or self.__controller.getBackSeatErrorFlag() == 1:
                break

            # go to the waypoint
            str = 'go to waypoint'
            rospy.loginfo(str)
            pubMissionLog.publish(str)

            while not rospy.is_shutdown() and time.time()-timeZero < self.__missionTimeout and self.__controller.getBackSeatErrorFlag() == 0:
            
                X = self.__controller.getX()
                Y = self.__controller.getY()
                heading = self.__controller.getHeading()
                rang, bear = self.__uti.rangeBearing([X,Y], [self.__wp[0], self.__wp[1]])
                if rang >= self.__wp_R:
                    errHeading = self.__uti.computeHeadingError(bear,heading)
                    u = self.__uti.surgeVelFromHeadingError(self.__uMax,self.__uGain,errHeading)
                    self.__controller.setRearProp(round(u*22.)) # turn speedDemand into propeller demand and send
                    self.__controller.setHeading(bear)
                else:
                    # when the AUV get to point M, move onto the next step
                    self.__controller.setRearProp(0)
                    self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
                    self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
                    time.sleep(self.__timeDelay) # allow the auv motion to decay
                    break

            # point toward the origin (pier)
            print 'point toward start location'
            timeStart = time.time()
            while not rospy.is_shutdown() and time.time()-timeZero < self.__missionTimeout and self.__controller.getBackSeatErrorFlag() == 0:
            
                X = self.__controller.getX()
                Y = self.__controller.getY()
                heading = self.__controller.getHeading()
                rang, bear = self.__uti.rangeBearing([X,Y], [0,0])
                errHeading = self.__uti.computeHeadingError(bear,heading)
                if abs(errHeading)>self.__errHeadingTol:
                    timeStart = time.time()
                if time.time()-timeStart<=self.__timeDelay:
                    self.__controller.setHeading(bear)
                else:
                    # when the heading is steady, move onto the next step
                    self.__controller.setRearProp(0)
                    self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
                    self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
                    break

            # bring the AUV to depth if the depth demand is specified
            if self.__depthDemand>=self.__depthDemandMin:
                str = 'descend to a depth of %sm' % self.__depthDemand
                rospy.loginfo(str)
                pubMissionLog.publish(str)
                timeStart = time.time()
                while not rospy.is_shutdown() and time.time()-timeZero < self.__missionTimeout and self.__controller.getBackSeatErrorFlag() == 0:
                
                    if abs(self.__controller.getDepth()-self.__depthDemand)>self.__depthTol:
                        timeRef = time.time() # reset the reference time
                    if time.time()-timeStart <= self.__timeDelay:
                        self.__controller.setDepth(self.__depthDemand)
                        self.__controller.setHeading(bear)
                    else:
                        # if the AUV get to the depth and stay there long enough, move onto the next step
                        str =  'steady at desired depth'
                        rospy.loginfo(str)
                        pubMissionLog.publish(str)
                        break
            
            # set demandThruster
            str = 'actuate thruster with demand = %s' %(demandThruster)
            rospy.loginfo(str)
            pubMissionLog.publish(str)
            timeStart = time.time()
            while not rospy.is_shutdown() and time.time()-timeZero < self.__missionTimeout and self.__controller.getBackSeatErrorFlag() == 0:
            
                if time.time()-timeStart <= self.__timeDemandHold:
                    self.__controller.setArduinoThrusterHorizontal(self.__direction*demandThruster,self.__direction*demandThruster) # (FrontHor,RearHor)
                    if self.__depthDemand>=self.__depthDemandMin:
                        self.__controller.setDepth(self.__depthDemand)
                else:
                    # if the AUV perform action long enoght, move onto the next step
                    self.__controller.setRearProp(0)
                    self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
                    self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
                    if self.__depthDemand>=self.__depthDemandMin:
                        self.__controller.setDepth(0)
                    break
                    
            # vehicle will stop for this many second as to let the AUV ascend to the surface
            if self.__depthDemand>=self.__depthDemandMin:
                self.__controller.setDepth(0) # by defult, the depth controller will turn off on its own after 1sec of not reciving new demand
                time.sleep(self.__timeDelay)
                    
        if self.__controller.getBackSeatErrorFlag() == 1:
            str= 'manoeuvreSway preempted at time = %s' %(time.time())    
            rospy.loginfo(str)
            pubMissionLog.publish(str)
            return 'preempted'
        elif time.time()-timeZero >= self.__missionTimeout:
            str= 'manoeuvreSway aborted at time = %s' %(time.time())    
            rospy.loginfo(str)
            pubMissionLog.publish(str)
            return 'aborted'
        else:
            str= 'manoeuvreSway succeed at time = %s' %(time.time())    
            rospy.loginfo(str)
            pubMissionLog.publish(str)
            return 'succeeded'
