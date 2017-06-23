#!/usr/bin/env python

'''
A state to verify a condition of lights.

return:
- succeeded: can only return succeeded; need an operator to observe if the light is flashing OK.
- aborted: not being used
- preempted: not being used

'''

import rospy
import numpy
import smach
import smach_ros
import time

class verify_lights(smach.State):
    def __init__(self, lib):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller = lib

    def execute(self, userdata):
        text = "Flashing lights now"
        rospy.loginfo(text)
        timeDelay = 0.2 # sec
        for a in range(10):
            self.__controller.lightOnOff(True)
            time.sleep(timeDelay)
            self.__controller.lightOnOff(False)
            time.sleep(timeDelay)
        
        return 'succeeded'
