#!/usr/bin/env python

# import roslib; roslib.load_manifest('delphin2_mission')
import rospy
import numpy
import smach
import smach_ros
import time
from pylab import *
from math import *

class actions(smach.State):
    def __init__(self, lib):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller = lib

    def execute(self, userdata):
        
        ####################################################################
        ### Perform actions ################################################
        ####################################################################

        listProp = [0.4,0.6,1] # TODO: replace this list with a propeller demand
        
        wp = array([[1,-20],
                    [3,40]])
        wpRang,_ = self.rangeBearing([wp[0][0], wp[1][0]],[wp[0][1], wp[1][1]]) # determine a range between waypoints as a reference
        wpIndex = 0
        wp_R = 5 # radius of acceptance
        uMax = 1
        uGain = -0.07
        errHeadingTol = 5 # acceptable heading error
        headingHold = 10 # hold heading for this amount of time in sec
        
        # move back and forth between two waypoint with different propDemand
        while not rospy.is_shutdown():
            for demandProp in listProp:
                
                # go to the waypoint
                while True:
                    X = self.__controller.getX()
                    Y = self.__controller.getY()
                    heading = self.__controller.getHeading()
                    rang, bear = rangeBearing([X,Y], [wp[0][wpIndex], wp[1][wpIndex]])
                    if rang < wp_R:
                        self.__controller.setSpeed(0)
                        break
                    errHeading = self.computeHeadingError(bear,heading)
                    u = 0.5*uMax*exp(-uGain*abs(errHeading)) # FIXME: this has to be changed to a propeller demand
                    self.__controller.setHeading(bear)
                    self.__controller.setSpeed(u)
                # time.sleep(10) FIXME: uncomment this section to allow the speed to decay    
                
                # point toward anoter waypoint
                timeStart = time.time()
                while True:
                    X = self.__controller.getX()
                    Y = self.__controller.getY()
                    heading = self.__controller.getHeading()
                    rang, bear = rangeBearing([X,Y], [wp[0][1-wpIndex], wp[1][1-wpIndex]])
                    errHeading = self.computeHeadingError(bear,heading)
                    self.__controller.setHeading(bear)
                    if abs(errHeading)>errHeadingTol:
                        timeStart = time.time()
                    timeElapse = time.time()-timeStart
                    if timeElapse>headingHold
                        break
                
                # set propDemand
                while True:
                    X = self.__controller.getX()
                    Y = self.__controller.getY()
                    heading = self.__controller.getHeading()
                    rang, bear = rangeBearing([X,Y], [wp[0][wpIndex], wp[1][wpIndex]])
                    self.__controller.setSpeed(propDemand) # FIXME: replace with setProp()
                    if rang>0.9*wpRang:
                        self.__controller.setSpeed(0) # FIXME: replace with setProp()
                        break
                        
                wpIndex = 1-wpIndex # switch the reference waypoint with another point

        return 'succeeded' # exit with a flag of 'succeeded'
    
    def rangeBearing(self, p1, p2):
        rang = sqrt( (p2[0]-p1[0])**2 + (p2[1]-p1[1])**2 )
        bear = atan2( (p2[0]-p1[0]), (p2[1]-p1[1]) )*180/pi
        if bear<0:
            bear = bear+360
            
        return rang, bear
    
    def computeHeadingError(self,demand,actual):
        errHeading = demand-actual
        if errHeading<-180:
            errHeading = errHeading%360
        elif errHeading>180:
            errHeading = -(-errHeading%360)
        return errHeading
