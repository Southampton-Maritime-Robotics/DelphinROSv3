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

@return: preempted: if the backSeatErrorFlag has been raised
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
    def __init__(self, lib, myUti, wp, uMax, wp_R, timeDemandHold, timeDelay, depthDemand, depthTol, depthDemandMin, controlRate):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller = lib
        self.__uti = myUti
        self.__wp = wp
        self.__uMax = uMax
        self.__wp_R = wp_R
        self.__timeDemandHold = timeDemandHold # thruster demand will be hold for this many second
        self.__timeDelay = timeDelay
        self.__depthDemand = depthDemand # [m].
        self.__depthTol = depthTol # [m]. It is account as the AUV get to the depth if the depth error is less than this.
        self.__depthDemandMin = depthDemandMin # [m] if the depthDemand is less than this, it is accounted as no depth demand specified.
        self.__listThrusterDemand = [700, 1400, 2100] # [100, 200, 400, 700, 1400, 2100] # list of thruster demand used in experiment
        self.__direction = -1 # 1:cw, -1:ccw 
        self.__controlRate = controlRate

    def execute(self, userdata):
    
        #Set Up Publisher for Mission Control Log
        pubMissionLog = rospy.Publisher('MissionStrings', String)
        
        ####################################################################
        ### Perform actions ################################################
        ####################################################################
        
        if self.__controlRate>0:
            r = rospy.Rate(self.__controlRate)

        # execute yaw motion response test
        for demandThruster in self.__listThrusterDemand:
        
            if rospy.is_shutdown() or self.__controller.getBackSeatErrorFlag() == 1:
                break
                
            # go to the waypoint
            str = 'go to waypoint: %s' %(self.__wp)
            rospy.loginfo(str)
            pubMissionLog.publish(str)
            while not rospy.is_shutdown() and self.__controller.getBackSeatErrorFlag() == 0:
            
                X = self.__controller.getX()
                Y = self.__controller.getY()
                heading = self.__controller.getHeading()
                rang, bear = self.__uti.rangeBearing([X,Y], [self.__wp[0], self.__wp[1]])
                if rang >= self.__wp_R:
                    errHeading = self.__uti.computeHeadingError(bear,heading)
                    u = self.__uti.surgeVelFromHeadingError(self.__uMax,errHeading)
                    self.__controller.setRearProp(round(u*22.)) # turn speedDemand into propeller demand and send
                    self.__controller.setHeading(bear)  
                else:
                    self.__controller.setRearProp(0)
                    self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
                    self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
                    
                    str = 'rest at the waypoint for timeDelay = %s sec' %self.__timeDelay
                    rospy.loginfo(str)
                    pubMissionLog.publish(str)
                    time.sleep(self.__timeDelay) # allow the auv motion to decay
                    break
                    
                if self.__controlRate>0:
                    r.sleep()
                    
            # bring the AUV to depth if the depth demand is specified
            # does not care where the AUV is pointing to
            if self.__depthDemand>=self.__depthDemandMin:
                str = 'descend to a depth of %sm' % self.__depthDemand
                rospy.loginfo(str)
                pubMissionLog.publish(str)
                timeStart = time.time()
                flag = 1
                while not rospy.is_shutdown() and self.__controller.getBackSeatErrorFlag() == 0:
                
                    if abs(self.__controller.getDepth()-self.__depthDemand)>self.__depthTol:
                        timeStart = time.time() # reset the reference time
                    if time.time()-timeStart <= self.__timeDelay:
                        if flag:
                            str = 'let the depth becomes stady for timeDelay = %s sec' %self.__timeDelay
                            rospy.loginfo(str)
                            pubMissionLog.publish(str)
                            flag = 0
                        self.__controller.setDepth(self.__depthDemand)
                    else:
                        # if the AUV get to the depth and stay there long enough, move onto the next step
                        str = 'steady at desired depth'
                        rospy.loginfo(str)
                        pubMissionLog.publish(str)
                        break
                        
                    if self.__controlRate>0:
                        r.sleep()

            # set demandThruster
            str = 'execute thruster with demand = %s' %(demandThruster)
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
             
                    self.__controller.setArduinoThrusterHorizontal(self.__direction*demandThruster,-self.__direction*demandThruster)
                    
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
                    
            # vehicle will stop for this many second as to let the AUV ascend to the surface
            if self.__depthDemand>=self.__depthDemandMin:
                self.__controller.setDepth(0) # by defult, the depth controller will turn off on its own after 1sec of not reciving new demand
                
                str = 'rest for timeDelay = %s sec to let the AUV assend' %self.__timeDelay
                rospy.loginfo(str)
                pubMissionLog.publish(str)

                time.sleep(self.__timeDelay)
                
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
