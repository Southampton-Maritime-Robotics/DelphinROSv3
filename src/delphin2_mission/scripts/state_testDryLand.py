#!/usr/bin/env python

'''
A state to do a run all the actuators and verify if they are functioning properly.

######################################
#Modifications
12/May/2015 use the feedback signal to identify if the actuators is functioning
11/Sep/2015 use only last N samples when do average

#TODO
-include the test for camera

'''

import rospy
import numpy
import smach
import smach_ros
import time
import cv
#from opencv.highgui import *

class testDryLand(smach.State):
    def __init__(self, lib):
            smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
            self.__controller = lib
            self.N = 10 # number of samples used when averaging
            self.timeActive = 3 # time that actuator will be active [sec]
            
    def execute(self, userdata):
            ####################################################################
            ### CAMERAS ########################################################
            ####################################################################
            #
            #
            #
            
            ####################################################################
            ### LIGHTS #########################################################
            ####################################################################
            str = "Flashing lights now"
            rospy.loginfo(str)            
            i = 0
            while i < 10:
                self.__controller.lightOnOff(True)
                time.sleep(0.2)
                self.__controller.lightOnOff(False)
                time.sleep(0.2)
                i = i + 1
                    
            ####################################################################
            ### THRUSTERS ######################################################
            ####################################################################

            #### Thruster 0 and 1 ####
            rpmT0 = [0]*self.N # create a list of zero with a length N
            rpmT1 = [0]*self.N # create a list of zero with a length N
            thrusterDemand = [-150,150]
            limitRPM_thruster = 120 # a minimum thruster speed that will be accounted as the propeller is functioning
            for a in thrusterDemand:
                time_ref = time.time()
                while time.time()-time_ref < self.timeActive:
                    self.__controller.setArduinoThrusterVertical(a,a)
                    rpmT0 = rpmT0[1:] + [self.__controller.getT0rpm()]
                    rpmT1 = rpmT1[1:] + [self.__controller.getT1rpm()]
                self.__controller.setArduinoThrusterVertical(0,0)
            rpmT0_avg = sum(rpmT0)/float(self.N)
            rpmT1_avg = sum(rpmT1)/float(self.N)
            
            if (rpmT0_avg < limitRPM_thruster):
                str = "Problem with thruster 0. Average speed = %s" %rpmT0_avg
                rospy.logerr(str)
                return 'aborted'
            if (rpmT1_avg < limitRPM_thruster):
                str = "Problem with thruster 1. Average speed = %s" %rpmT1_avg
                rospy.logerr(str)
                return 'aborted'
            str = "Thruster 0 - working"
            rospy.loginfo(str)
            str = "Thruster 1 - working"
            rospy.loginfo(str)
            
            #### Thruster 2 and 3 ####
            rpmT2 = [0]*self.N # create a list of zero with a length N
            rpmT3 = [0]*self.N # create a list of zero with a length N
            for a in thrusterDemand:
                time_ref = time.time()
                while time.time()-time_ref < self.timeActive:
                    self.__controller.setArduinoThrusterHorizontal(a,a)
                    rpmT2 = rpmT2[1:] + [self.__controller.getT2rpm()]
                    rpmT3 = rpmT3[1:] + [self.__controller.getT3rpm()]
                self.__controller.setArduinoThrusterHorizontal(0,0)
            rpmT2_avg = sum(rpmT2)/float(self.N)
            rpmT3_avg = sum(rpmT3)/float(self.N)
            
            if (rpmT2_avg < limitRPM_thruster):
                str = "Problem with thruster 2. Average speed = %s" %rpmT2_avg
                rospy.logerr(str)
                return 'aborted'
            if (rpmT3_avg < limitRPM_thruster):
                str = "Problem with thruster 3. Average speed = %s" %rpmT3_avg
                rospy.logerr(str)
                return 'aborted'
            str = "Thruster 2 - working"
            rospy.loginfo(str)
            str = "Thruster 3 - working"
            rospy.loginfo(str)
            
            ####################################################################
            ### FINS ###########################################################
            ####################################################################

            angles = [-30.0, 0.0, 30.0]
            fin_error_tol = 3.0
            for a in angles:
                time_ref = time.time()
                while time.time()-time_ref<self.timeActive:
                    self.__controller.setControlSurfaceAngle(a,a,a,a)
                
                if numpy.abs(self.__controller.getCS_b() - a) > fin_error_tol:
                    str = "Problem with top control surface. Feedback = %d" %self.__controller.getCS_b()
                    rospy.logerr(str)
                    return 'aborted' 
                if numpy.abs(self.__controller.getCS_c() - a) > fin_error_tol:
                    str = "Problem with starboard control surface. Feedback = %d" %self.__controller.getCS_C()
                    rospy.logerr(str)
                    return 'aborted' 
                if numpy.abs(self.__controller.getCS_d() - a) > fin_error_tol:
                    str = "Problem with bottom control surface. Feedback = %d" %self.__controller.getCS_d()
                    rospy.logerr(str)
                    return 'aborted' 
                if numpy.abs(self.__controller.getCS_e() - a) > fin_error_tol:
                    str = "Problem with port control surface. Feedback = %d" %self.__controller.getCS_e()
                    rospy.logerr(str)
                    return 'aborted'
            
            self.__controller.setControlSurfaceAngle(0,0,0,0)
            str = "Control surfaces - working"
            rospy.loginfo(str)
            time.sleep(2) # allow the control surfaces to get back to the neutral position

            ####################################################################
            ### PROPELLER ######################################################
            ####################################################################
            
            time_ref = time.time()
            prop_rps = [0]*self.N # create a list of zero with a length N
            while time.time()-time_ref<self.timeActive:
                self.__controller.setRearProp(10)
                prop_rps = prop_rps[1:] + [self.__controller.getPropRPS()]
            prop_rps_avg = sum(prop_rps)/float(self.N)
            self.__controller.setRearProp(0)

            if prop_rps_avg > 2:
                str = "Rear prop - working"
                rospy.loginfo(str)
            else:
                str = "Problem with rear prop - average rps = %s" %prop_rps_avg
                rospy.logerr(str)
                return 'aborted'
                
            return 'succeeded'
                
            #return 'preempted'

            #return 'aborted'
