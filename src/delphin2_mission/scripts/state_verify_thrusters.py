#!/usr/bin/env python

'''
A state to verify a condition of thrusters.

return:
- succeeded: thruster check is passed
- aborted: faulty is detected
- preempted: not being used

'''

import rospy
import numpy
import smach
import smach_ros
import time

class verify_thrusters(smach.State):
    def __init__(self, lib):
            smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
            self.__controller = lib
            self.__list_length = 10 # number of samples used when averaging
            self.__time_hold = 3 # time that actuator will be active [sec]
            self.__thrusterDemand = [-150,150]
            self.__limitRPM_thruster = 120 # a minimum thruster speed that will be accounted as the propeller is functioning
            
    def execute(self, userdata):
            #### Thruster 0 and 1: vertical thruster ####
            rpmT0 = [0]*self.__list_length # create a list of zero
            rpmT1 = [0]*self.__list_length # create a list of zero

            for a in self.__thrusterDemand:
                time_ref = time.time()
                while time.time()-time_ref < self.__time_hold:
                    self.__controller.setArduinoThrusterVertical(a,a)
                    rpmT0 = rpmT0[1:] + [self.__controller.getT0rpm()]
                    rpmT1 = rpmT1[1:] + [self.__controller.getT1rpm()]
                self.__controller.setArduinoThrusterVertical(0,0)
            rpmT0_avg = sum(rpmT0)/float(self.__list_length)
            rpmT1_avg = sum(rpmT1)/float(self.__list_length)
            
            if (rpmT0_avg < self.__limitRPM_thruster):
                text = "Problem with thruster 0. Average speed = %s" %rpmT0_avg
                rospy.logerr(text)
                return 'aborted'
            if (rpmT1_avg < self.__limitRPM_thruster):
                text = "Problem with thruster 1. Average speed = %s" %rpmT1_avg
                rospy.logerr(text)
                return 'aborted'
            text = "Thruster 0 - working"
            rospy.loginfo(text)
            text = "Thruster 1 - working"
            rospy.loginfo(text)
            
            #### Thruster 2 and 3: horizontal thruster ####
            rpmT2 = [0]*self.__list_length # create a list of zero with a length N
            rpmT3 = [0]*self.__list_length # create a list of zero with a length N
            for a in self.__thrusterDemand:
                time_ref = time.time()
                while time.time()-time_ref < self.__time_hold:
                    self.__controller.setArduinoThrusterHorizontal(a,a)
                    rpmT2 = rpmT2[1:] + [self.__controller.getT2rpm()]
                    rpmT3 = rpmT3[1:] + [self.__controller.getT3rpm()]
                self.__controller.setArduinoThrusterHorizontal(0,0)
            rpmT2_avg = sum(rpmT2)/float(self.__list_length)
            rpmT3_avg = sum(rpmT3)/float(self.__list_length)
            
            if (rpmT2_avg < self.__limitRPM_thruster):
                text = "Problem with thruster 2. Average speed = %s" %rpmT2_avg
                rospy.logerr(text)
                return 'aborted'
            if (rpmT3_avg < self.__limitRPM_thruster):
                text = "Problem with thruster 3. Average speed = %s" %rpmT3_avg
                rospy.logerr(text)
                return 'aborted'
            text = "Thruster 2 - working"
            rospy.loginfo(text)
            text = "Thruster 3 - working"
            rospy.loginfo(text)
            
            return 'succeeded'
