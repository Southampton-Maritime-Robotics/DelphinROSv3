#!/usr/bin/env python

'''
######################################
# description
A script to get the AUV perform the turning circle manoeuvre.
Given two reference points, the AUV will move in a straight line from one point with a heading bias to the other point. When it is travel far enough and the surge speed is steady, it will execute actuator (either rudders, thrusters or their combination). The setpoint will hold until the AUV have performed two circle turns (720 degree).

Routine
    go to first reference point
    head to another reference point with a heading bias
    descend to a desired depth
    move forward with a fixed propeller setpoint
    when far enough, execute the actuators
    hold the demand until the AUV complete two turns
    stop and ascend to the surface
    
######################################
#TODO
-make the AUV performs action at depth
-include a criterion to let the AUV stop when it have performed 720 degree turn

#Modifications

'''

import rospy
import numpy
import smach
import smach_ros
import time
from pylab import *
from math import *

class testTurningCircle(smach.State):
    def __init__(self, lib, myUti, wp, uGain, uMax, errHeadingTol, wp_R, timeDemandHold, timeDelay):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller = lib
        self.__uti = myUti
        self.__wp = wp
        self.__uGain = uGain
        self.__uMax = uMax
        self.__errHeadingTol = errHeadingTol
        self.__wp_R = wp_R
        self.__timeDemandHold = timeDemandHold # demand will be hold for this many second
        self.__timeDelay = timeDelay
        self.__listProp = [10,22] # list of propeller demand used in experiment
        self.__listThruster = [400, 2000] # list of thruster demand used in experiment [+ve yaw CW]
        self.__listRudder = [10,30] # list of rudder angle used in experiment [+ve yaw CW]
        self.__headingBias = -25 # TODO [deg, +ve CW] point away from the target just so the vehicle have enough space for turning

    def execute(self, userdata):
        
        ####################################################################
        ### Perform actions ################################################
        ####################################################################

        wpRang,_ = self.__uti.rangeBearing([self.__wp[0][0], self.__wp[1][0]],[self.__wp[0][1], self.__wp[1][1]]) # determine a range between waypoints as a reference
        wpIndex = 0
        
        print 'execute turning circle test'
        
        # move back and forth between two waypoint with different combination of propeller, thruster and rudder demand
            
        for demandProp in self.__listProp:
            for demandThruster in self.__listThruster:
                for demandRudder in self.__listRudder:
                    # go to the waypoint
                    print 'go to start point'
                    while not rospy.is_shutdown():
                        X = self.__controller.getX()
                        Y = self.__controller.getY()
                        heading = self.__controller.getHeading()
                        rang, bear = self.__uti.rangeBearing([X,Y], [self.__wp[0][wpIndex], self.__wp[1][wpIndex]])
                        if rang < self.__wp_R:
                            self.__controller.setRearProp(0)
                            self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
                            self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
                            time.sleep(self.__timeDelay) # vehicle will stop for this many second as to let its motion decay
                            break
                        errHeading = self.__uti.computeHeadingError(bear,heading)
                        u = self.__uti.surgeVelFromHeadingError(self.__uMax,self.__uGain,errHeading)
#                        u = self.__uMax*exp(-self.__uGain*abs(errHeading)) # determine an appropriate speed demand based on the heading error
                        self.__controller.setRearProp(round(u*22.)) # turn speedDemand into propeller demand and send
                        self.__controller.setHeading(bear)
                     
                    # point toward anoter waypoint with a bias of headingBias
                    print 'head toward the target with a bias of ', headingBias, ' deg'
                    timeStart = time.time()
                    while not rospy.is_shutdown():
                        X = self.__controller.getX()
                        Y = self.__controller.getY()
                        heading = self.__controller.getHeading()
                        rang, bear = self.__uti.rangeBearing([X,Y], [self.__wp[0][1-wpIndex], self.__wp[1][1-wpIndex]])
                        headingDemand = mod(bear+self.__headingBias,360) # deviate from the bearing by headingBias
                        errHeading = self.__uti.computeHeadingError( headingDemand, heading )
                        self.__controller.setHeading(headingDemand)
                        if abs(errHeading)>self.__errHeadingTol:
                            timeStart = time.time()
                        timeElapse = time.time()-timeStart
                        if timeElapse>self.__timeDelay:
                            self.__controller.setRearProp(0)
                            self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
                            self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
                            break
                    
                    # get the AUV accelerated then apply demand
                    print 'demand is hold at [prop, thruster, rudder] = ', [demandProp, demandThruster, demandRudder]
                    while not rospy.is_shutdown():
                        X = self.__controller.getX()
                        Y = self.__controller.getY()
                        heading = self.__controller.getHeading()
                        rang, bear = self.__uti.rangeBearing([X,Y], [self.__wp[0][wpIndex], self.__wp[1][wpIndex]])
                        self.__controller.setRearProp(demandProp)
                        self.__controller.setHeading(headingDemand) # hold at the previous demand
                        if rang > wpRang*0.75:
                            break
                    # activate the actuator when AUV is far enough from the start location
                    timeStart = time.time()
                    while not rospy.is_shutdown():
                        self.__controller.setControlSurfaceAngle(demandRudder,0,demandRudder,0) # (VerUp,HorRight,VerDown,HorLeft)
                        self.__controller.setArduinoThrusterHorizontal(demandThruster,-demandThruster) # (FrontHor,RearHor)
                        if time.time()-timeStart > self.__timeDemandHold:
                            print 'enough! Time to move onto next a combination of actuator demands'
                            self.__controller.setRearProp(0)
                            self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
                            self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
                            break
                            
                    # switch the role of two reference waypoints
                    wpIndex = 1-wpIndex
            
        return 'succeeded' # exit with a flag of 'succeeded'
