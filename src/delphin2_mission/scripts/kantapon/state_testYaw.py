#!/usr/bin/env python

import rospy
import numpy
import smach
import smach_ros
import time
from pylab import *
from math import *

class testYaw(smach.State):
    def __init__(self, lib, myUti, wp, uGain, uMax, wp_R, timeDemandHold, timeDelay):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller = lib
        self.__uti = myUti
        self.__wp = wp
        self.__uGain = uGain
        self.__uMax = uMax
        self.__wp_R = wp_R
        self.__timeDemandHold = timeDemandHold # thruster demand will be hold for this many second
        self.__timeDelay = timeDelay

    def execute(self, userdata):
        
        ####################################################################
        ### Perform actions ################################################
        ####################################################################

        listThrusterDemand = [100, 200, 400, 700, 1400,2100] # list of thruster demand used in experiment
        
        print 'execute yaw motion response test'
        
        # execute yaw motion response test
        while not rospy.is_shutdown():
            
            for demandThruster in listThrusterDemand:
                print 'thruster demand is ', demandThruster
                # go to the waypoint
                print 'go to waypoint'
                while True:
                    if rospy.is_shutdown():
                        break
                    X = self.__controller.getX()
                    Y = self.__controller.getY()
                    heading = self.__controller.getHeading()
                    rang, bear = self.__uti.rangeBearing([X,Y], [self.__wp[0], self.__wp[1]])
                    if rang < self.__wp_R:
                        self.__controller.setRearProp(0)
                        self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
                        self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
                        time.sleep(self.__timeDelay) # allow the auv motion to decay
                        break
                    errHeading = self.__uti.computeHeadingError(bear,heading)
                    u = 0.5*self.__uMax*exp(-self.__uGain*abs(errHeading)) # determine an appropriate speed demand based on the heading error
                    self.__controller.setRearProp(round(u*22.)) # turn speedDemand into propeller demand and send
                    self.__controller.setHeading(bear)


                
                # set demandThruster
                print 'actuate thruster with demand = ', demandThruster
                timeStart = time.time()
                while True:
                    if rospy.is_shutdown():
                        break
                    self.__controller.setArduinoThrusterHorizontal(demandThruster,-demandThruster)
                    if time.time()-timeStart > self.__timeDemandHold:
                        self.__controller.setRearProp(0)
                        self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
                        self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
                        break
                        
            return 'succeeded' # exit with a flag of 'succeeded'
