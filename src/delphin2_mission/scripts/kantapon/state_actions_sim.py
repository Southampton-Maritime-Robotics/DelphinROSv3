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

#            self.__controller.setRearProp(25)
            self.__controller.sway(0.2) # (this function actually requires tsl_setpoints ~500-1000, however here requires sway speed in m/s)
#            flagCon = 1
#            while flagCon:
#                self.__controller.setHeading(30)
#                self.__controller.setRearProp(10)
#                self.__controller.setDepth(2)
#                if self.__controller.getX()>1000:
#                    flagCon = 0
#                        
#            flagCon = 1
#            while flagCon:
#                self.__controller.setHeading(210)
#                self.__controller.setRearProp(14)
#                self.__controller.setDepth(2)
#                if self.__controller.getX<0:
#                    flagCon = 0

        return 'succeeded' # exit with a flag of 'succeeded'
