#!/usr/bin/env python

'''
A state to verify a condition of propeller.

return:
- succeeded: propeller check is passed
- aborted: faulty is detected
- preempted: not being used

'''

import rospy
import numpy
import smach
import smach_ros
import time

class verify_prop(smach.State):
    def __init__(self, lib):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller = lib
        self.__list_length = 10 # number of samples used when averaging
        self.__timeHold = 3 # time that actuator will be active [sec]
        self.__propSetpoint = 10
        self.__prop_rps_min = 2
            
    def execute(self, userdata):
        time_ref = time.time()
        prop_rps = [0]*self.__list_length # create a list of zero
        
        while time.time()-time_ref<self.__timeHold:
            self.__controller.setRearProp(self.__propSetpoint)
            prop_rps = prop_rps[1:] + [self.__controller.getPropRPS()]
        
        prop_rps_avg = sum(prop_rps)/float(self.__list_length)
        
        # turn off the propeller
        self.__controller.setRearProp(0)
        
        if prop_rps_avg > self.__prop_rps_min:
            text = "Rear prop - working"
            rospy.loginfo(text)
            return 'succeeded'
        else:
            text = "Problem with rear prop - average rps = %s" %prop_rps_avg
            rospy.logerr(text)
            return 'aborted'
