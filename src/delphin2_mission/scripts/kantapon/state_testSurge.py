#!/usr/bin/env python

# import roslib; roslib.load_manifest('delphin2_mission')
import rospy
import numpy
import smach
import smach_ros
import time
from pylab import *
from math import *

#### from kantapon's folder
import sys

class testSurge(smach.State):
    def __init__(self, lib, myUti, wp, uGain, uMax, errHeadingTol, wp_R, timeDelay):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller = lib
        self.__uti = myUti
        self.__wp = wp
        self.__uGain = uGain
        self.__uMax = uMax
        self.__errHeadingTol = errHeadingTol
        self.__wp_R = wp_R
        self.__timeDelay = timeDelay

    def execute(self, userdata):
        
        ####################################################################
        ### Perform actions ################################################
        ####################################################################

        listProp = [10,14,18,22] # list of propeller demand used in experiment
        
        wpRang,_ = self.__uti.rangeBearing([self.__wp[0][0], self.__wp[1][0]],[self.__wp[0][1], self.__wp[1][1]]) # determine a range between waypoints as a reference
        wpIndex = 0
        
        print 'execute surge motion response test'
        
        # move back and forth between two waypoint with different demandProp
        while not rospy.is_shutdown():
            
            for demandProp in listProp:
                print 'prop demand is ', demandProp
                # go to the waypoint
                print 'go to start point'
                while True:
                    X = self.__controller.getX()
                    Y = self.__controller.getY()
                    heading = self.__controller.getHeading()
                    rang, bear = self.__uti.rangeBearing([X,Y], [self.__wp[0][wpIndex], self.__wp[1][wpIndex]])
                    if rang < self.__wp_R:
                        self.__controller.setRearProp(0)
                        break
                    errHeading = self.__uti.computeHeadingError(bear,heading)
                    u = 0.5*self.__uMax*exp(-self.__uGain*abs(errHeading)) # determine an appropriate speed demand based on the heading error
                    self.__controller.setRearProp(round(u*22.)) # turn speedDemand into propeller demand and send
                    self.__controller.setHeading(bear)

                time.sleep(self.__timeDelay) # vehicle will stop for this many second as to let its motion decay
                 
                # point toward anoter waypoint
                print 'head toward the target'
                timeStart = time.time()
                while True:
                    X = self.__controller.getX()
                    Y = self.__controller.getY()
                    heading = self.__controller.getHeading()
                    rang, bear = self.__uti.rangeBearing([X,Y], [self.__wp[0][1-wpIndex], self.__wp[1][1-wpIndex]])
                    errHeading = self.__uti.computeHeadingError(bear,heading)
                    self.__controller.setHeading(bear)
                    if abs(errHeading)>self.__errHeadingTol:
                        timeStart = time.time()
                    timeElapse = time.time()-timeStart
                    if timeElapse>self.__timeDelay:
                        break
                
                # set demandProp
                print 'apply propeller demand = ', demandProp
                while True:
                    X = self.__controller.getX()
                    Y = self.__controller.getY()
                    heading = self.__controller.getHeading()
                    rang, bear = self.__uti.rangeBearing([X,Y], [self.__wp[0][wpIndex], self.__wp[1][wpIndex]])
                    self.__controller.setRearProp(demandProp)
                    if abs(wpRang-rang) < self.__wp_R:
                        self.__controller.setRearProp(0)
                        break
                        
                # switch the role of two reference waypoints
                wpIndex = 1-wpIndex
                
            return 'succeeded' # exit with a flag of 'succeeded'
