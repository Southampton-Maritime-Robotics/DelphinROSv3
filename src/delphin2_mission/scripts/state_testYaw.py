#!/usr/bin/env python

'''
######################################
# description
A script to get the AUV to point M (middle of the lake) and let it executes the thruster demand when rudder is set to amidships and propeller is not running.
If the depth demand is less than 0.5m, it will accaount as no depth demand specified.

Routine
    go to point M
    descend to a desired depth
    execute thruster setpoint
    stop and ascend to the surface
    
######################################
#Modifications
20/May/2015: insert the depth control in the procedure

'''

import rospy
import numpy
import smach
import smach_ros
import time
from pylab import *
from math import *

class testYaw(smach.State):
    def __init__(self, lib, myUti, wp, uGain, uMax, wp_R, timeDemandHold, timeDelay, depthDemand, depthTol, depthDemandMin):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller = lib
        self.__uti = myUti
        self.__wp = wp
        self.__uGain = uGain
        self.__uMax = uMax
        self.__wp_R = wp_R
        self.__timeDemandHold = timeDemandHold # thruster demand will be hold for this many second
        self.__timeDelay = timeDelay
        self.__depthDemand = depthDemand # [m].
        self.__depthTol = depthTol # [m]. It is account as the AUV get to the depth if the depth error is less than this.
        self.__depthDemandMin = depthDemandMin # [m] if the depthDemand is less than this, it is accounted as no depth demand specified.
        self.__listThrusterDemand = [700, 1400, 2100] # [100, 200, 400, 700, 1400, 2100] # list of thruster demand used in experiment
        self.__direction = -1 # 1:cw, -1:ccw 

    def execute(self, userdata):
        
        ####################################################################
        ### Perform actions ################################################
        ####################################################################

        # execute yaw motion response test
        for demandThruster in self.__listThrusterDemand:
            # go to the waypoint
            print 'go to waypoint'
            while not rospy.is_shutdown():
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
                    self.__controller.setRearProp(0)
                    self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
                    self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
                    time.sleep(self.__timeDelay) # allow the auv motion to decay
                    break
                    
            # bring the AUV to depth if the depth demand is specified
            if self.__depthDemand>=self.__depthDemandMin:
                print 'descend to desired depth'
                timeStart = time.time()
                while not rospy.is_shutdown():
                    if abs(self.__controller.getDepth()-self.__depthDemand)>self.__depthTol:
                        timeRef = time.time() # reset the reference time
                    if time.time()-timeStart <= self.__timeDelay:
                        self.__controller.setDepth(self.__depthDemand)
                    else:
                        # if the AUV get to the depth and stay there long enough, move onto the next step
                        break

            # set demandThruster
            print 'actuate thruster with demand = ', demandThruster
            timeStart = time.time()
            while not rospy.is_shutdown():
                
                if time.time()-timeStart < self.__timeDemandHold:
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
                    
            # vehicle will stop for this many second as to let the AUV ascend to the surface
            if self.__depthDemand>=self.__depthDemandMin:
                self.__controller.setDepth(0) # by defult, the depth controller will turn off on its own after 1sec of not reciving new demand
                time.sleep(self.__timeDelay)
                    
        return 'succeeded' # exit with a flag of 'succeeded'
