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
sys.path.append('/home/delphin2/DelphinROSv3/src/delphin2_mission/scripts/kantapon')
from utilities                      import uti

class pathFollowingLOS(smach.State):
    def __init__(self, lib, myUti, path, L_los, uGain, uMax, wp_R):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller = lib
        self.__uti = myUti
        self.__path = path
        self.__L_los = L_los
        self.__uGain = uGain
        self.__uMax = uMax
        self.__wp_R = wp_R

    def execute(self, userdata):
        
        ####################################################################
        ### Perform actions ################################################
        ####################################################################
        
        _,pathLen = self.__path.shape
        
        X = self.__controller.getX()
        Y = self.__controller.getY()
        eta = [X,Y]
        
        # identify the waypoint to follow
#        wpTarget = self.__uti.findFirstSegment(self.__path,eta)
        wpTarget = 1

        print 'Execute path following algorithm with a following path'
        print 'X = ', self.__path[0,:]
        print 'Y = ', self.__path[1,:]
        print 'target waypoint ', self.__path[:,wpTarget]
        
        # move back and forth between two waypoint with different demandProp
        while not rospy.is_shutdown():
            
            X = self.__controller.getX()
            Y = self.__controller.getY()
            heading = self.__controller.getHeading()
            eta = [X,Y] # state vector denoted following Fossen's convention
            
            # waypoint switching criteria
#            target = self.__path[:,wpTarget]
#            print 'error in distance : ', sqrt( (target[0]-eta[0])**2 + (target[1]-eta[1])**2 )

            if (self.__uti.waypointSwitching(self.__path[:,wpTarget],eta,self.__wp_R)):
                if wpTarget == pathLen-1:
                    # if arrive to the last waypoint, terminate the mission
                    print 'arrived to within the circle of acceptance of the destination'
                    print 'current location is ', eta
                    self.__controller.setRearProp(0)
                    self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
                    self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
                    return 'succeeded'
                else:
                    # if reached the current waypoint, move onto the next line segment
                    wpTarget += 1
                    print 'target waypoint ', self.__path[:,wpTarget]

            # compute line-of-sight parameters
            t,p_inter = self.__uti.interPointLine(self.__path[:,wpTarget-1],self.__path[:,wpTarget],eta)
            vecCross = p_inter-eta # cross track error
            ye = sqrt( vecCross[0]**2 + vecCross[1]**2 )
            if ye>=self.__L_los:
                los_p = p_inter
            else:
                xe = sqrt( self.__L_los**2 - ye**2 ) # compute lookahead distance
                vecAlong = self.__path[:,wpTarget]-self.__path[:,wpTarget-1]
                vecAlongLen = sqrt( vecAlong[0]**2 + vecAlong[1]**2 )
                los_p = p_inter + xe*vecAlong/vecAlongLen

            los_vec = los_p-eta
            los_a = atan2(los_vec[0],los_vec[1])*180/pi # TODO: incorporated side slip angle
            if los_a<0: # confine the los_a within [0,2pi)
                los_a = los_a+360

            # determine heading error
            errHeading = self.__uti.computeHeadingError(los_a,heading)
            u = self.__uMax*exp(-self.__uGain*abs(errHeading))
            
            
            # publish heading demand
            self.__controller.setHeading(los_a)
            # turn speedDemand into propeller demand and send
            self.__controller.setRearProp(round(u*22.))
            
#            print heading, los_a
