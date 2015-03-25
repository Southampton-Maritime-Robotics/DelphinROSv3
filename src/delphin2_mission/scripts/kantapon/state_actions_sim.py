#!/usr/bin/env python

# import roslib; roslib.load_manifest('delphin2_mission')
import rospy
import numpy
import smach
import smach_ros
import time

class actions(smach.State):
    def __init__(self, lib):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller = lib

    def execute(self, userdata):
        
        ####################################################################
        ### Perform actions ################################################
        ####################################################################

        time_zero=time.time()
        while not rospy.is_shutdown():

            flagCon = 1
            while flagCon:
                self.__controller.setHeading(30)
                self.__controller.setSpeed(0.9)
                self.__controller.setDepth(2)
                if self.__controller.getX()>1000:
                    flagCon = 0
                        
            flagCon = 1
            while flagCon:
                self.__controller.setHeading(210)
                self.__controller.setSpeed(0.9)
                self.__controller.setDepth(2)
                if self.__controller.getX<0:
                    flagCon = 0

        return 'succeeded' # exit with a flag of 'succeeded'
