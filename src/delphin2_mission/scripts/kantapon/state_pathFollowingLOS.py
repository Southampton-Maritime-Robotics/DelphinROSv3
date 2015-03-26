#!/usr/bin/env python

import rospy
import numpy
import smach
import smach_ros
import time
from pylab import *
from math import *

#### from kantapon's folder
import sys

class pathFollowingLOS(smach.State):
    def __init__(self, lib, myUti, path, L_los, uGain, uMax):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller = lib
        self.__uti = myUti
        self.__path = path
        self.__L_los = L_los
        self.__uGain = uGain
        self.__uMax = uMax
        self.__wp_R = wp_R
        self.__errHeadingTol = errHeadingTol

    def execute(self, userdata):
        
        ####################################################################
        ### Perform actions ################################################
        ####################################################################
        
        X = self.__controller.getX()
        Y = self.__controller.getY()
        eta = [X,Y]
        
        # identify the waypoint to follow
        wpTarget = self.__uti.findFirstSegment(self.__path,eta)
        
        # move back and forth between two waypoint with different demandProp
        while not rospy.is_shutdown():
            
            X = self.__controller.getX()
            Y = self.__controller.getY()
            eta = [X,Y] # state vector denoted following Fossen's convention
            
            # waypoint switching criteria
            if (self.__uti.waypointSwitching(path[:,wpTarget],eta,wp_R)):
                if wpTarget == pathLen-1:
                    # if arrive to the last waypoint, terminate the mission
                    print 'arrived to the destination completed'
                    print eta
                    return 'secceeded'
                else:
                    # if reached the current waypoint, move onto the next line segment
                    wpTarget += 1
                    print 'move onto the line segment number ', wpTarget

            # compute line-of-sight parameters
            t,p_inter = self.__uti.interPointLine(path[:,wpTarget-1],path[:,wpTarget],eta)
            vecCross = p_inter-eta # cross track error
            ye = sqrt( vecCross[0]**2 + vecCross[1]**2 )
            if ye>=L_los:
                los_p = p_inter
            else:
                xe = sqrt( self.__L_los**2 - ye**2 ) # compute lookahead distance
                vecAlong = path[:,wpTarget]-path[:,wpTarget-1]
                vecAlongLen = sqrt( vecAlong[0]**2 + vecAlong[1]**2 )
                los_p = p_inter + xe*vecAlong/vecAlongLen

            los_vec = los_p-eta
            los_a = atan2(los_vec[0],los_vec[1])*180/pi # TODO: incorporated side slip angle
            if los_a<0: # confine the los_a within [0,2pi)
                los_a = los_a+360

            # determine heading error
            errHeading = self.__uti.computeHeadingError(los_a,heading)
            u = uMax*exp(-uGain*abs(errHeading))
            
            
            # only publish the heading demand when the error is greater than tolerance
            if errHeading>self.__errHeadingTol:
                self._controller.setHeading(los_a)
            # turn speedDemand into propeller demand and send
            self._controller.setRearProp(round(u*22.))
