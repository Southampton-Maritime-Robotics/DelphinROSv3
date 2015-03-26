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

class pathFollowingLOS(smach.State):
    def __init__(self, lib, myUti, path, L_los, uGain, uMax):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller = lib
        self.__uti = myUti
        self.__path = path
        self.__L_los = L_los
        self.__uGain = uGain
        self.__uMax = uMax

    def execute(self, userdata):
        
        ####################################################################
        ### Perform actions ################################################
        ####################################################################
        
        # move back and forth between two waypoint with different demandProp
        while not rospy.is_shutdown():
            pass
