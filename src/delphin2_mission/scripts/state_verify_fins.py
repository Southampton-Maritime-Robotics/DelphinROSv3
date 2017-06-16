#!/usr/bin/env python

'''
A state to verify a condition of control surfaces.

return:
- succeeded: control surfaces check is passed
- aborted: faulty is detected
- preempted: not being used

'''

import rospy
import numpy
import smach
import smach_ros
import time

class verify_fins(smach.State):
    def __init__(self, lib):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller = lib
        self.__timeHold = 3 # time that actuator will be active [sec]
        self.__angles = [-30.0, 0.0, 30.0]
        self.__fin_error_tol = 3.0

    def execute(self, userdata):    
        for a in self.__angles:
            time_ref = time.time()
            while time.time()-time_ref<self.__timeHold:
                self.__controller.setControlSurfaceAngle(a,a,a,a)
            
            if numpy.abs(self.__controller.getCS_b() - a) > self.__fin_error_tol:
                print_msg = "Problem with top control surface. Feedback = %d" %self.__controller.getCS_b()
                rospy.logerr(print_msg)
                return 'aborted' 
            if numpy.abs(self.__controller.getCS_c() - a) > self.__fin_error_tol:
                print_msg = "Problem with starboard control surface. Feedback = %d" %self.__controller.getCS_C()
                rospy.logerr(print_msg)
                return 'aborted' 
            if numpy.abs(self.__controller.getCS_d() - a) > self.__fin_error_tol:
                print_msg = "Problem with bottom control surface. Feedback = %d" %self.__controller.getCS_d()
                rospy.logerr(print_msg)
                return 'aborted' 
            if numpy.abs(self.__controller.getCS_e() - a) > self.__fin_error_tol:
                print_msg = "Problem with port control surface. Feedback = %d" %self.__controller.getCS_e()
                rospy.logerr(print_msg)
                return 'aborted'
        
        # set fins to a neutral position
        self.__controller.setControlSurfaceAngle(0,0,0,0)
        print_msg = "Control surfaces - working"
        rospy.loginfo(print_msg)
        time.sleep(self.__timeHold) # allow the control surfaces to get back to the neutral position
        
        return 'succeeded'
