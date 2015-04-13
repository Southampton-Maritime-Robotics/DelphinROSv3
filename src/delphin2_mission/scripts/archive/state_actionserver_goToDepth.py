#!/usr/bin/env python

import roslib; roslib.load_manifest('delphin2_mission')
import rospy
import smach
import smach_ros
import time
from delphin2_mission.msg import GoToDepthAction, GoToDepthGoal
from actionlib import *
from actionlib.msg import *

# GoToDepthServer ActionServer class
#
# __init__ - initialises an GoToDepthServer object
# @param: lib - instance of library_highlevel.py
# @param: auto_start_flag - must be set to false! (as specified in ROS documentation)
#
# execute_cb - attempts to drive the vehicle to a given depth - returns when desired depth is reached
# @return: aborted: if timeout is reached
# @return: succeeded: if depth is reached within the timeout
#
#
# TO DO:
# need to wait for depth to be roughly constant (at present overshooting will be considered a success)
# check pitch angle is within sensible limits

class GoToDepthServer:
    def __init__(self, name, lib, auto_start_flag):
        self.__controller = lib
        self.__sas = SimpleActionServer(name,
                GoToDepthAction, #from .action file
                execute_cb=self.execute_cb, auto_start=auto_start_flag)
        self.__sas.start()

    def execute_cb(self, goal):
        time_zero = time.time()
        self.__controller.setDepth(goal.depthDemand)
        while (time.time()-time_zero < goal.timeout) and self.__controller.getBackSeatErrorFlag() == 0:
            if abs(self.__controller.getDepth() - goal.depthDemand) <= goal.tolerance :  
                # TO DO:
                # need to wait for depth to be roughly constant (at present overshooting will be considered a success)
                # check pitch angle is within sensible limits
                self.__sas.set_succeeded()
                print 'success! '
                return

        if self.__controller.getBackSeatErrorFlag() == 1:
            self.__sas.set_preempted()
        else:
            self.__sas.set_aborted()
        
