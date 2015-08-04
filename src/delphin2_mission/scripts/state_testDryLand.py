#!/usr/bin/env python

'''
A state to do a run all the actuators and verify if they are functioning properly.

######################################
#Modifications
12/May/2015 use the feedback signal to identify if the actuators is functioning

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
            N = 0
            rpmT0_avg = 0
            rpmT1_avg = 0
            time_ref = time.time()
            while time.time()-time_ref < 3:    
                self.__controller.setArduinoThrusterVertical(-150,-150)
                rpmT0_avg += numpy.abs(self.__controller.getT0rpm())
                rpmT1_avg += numpy.abs(self.__controller.getT1rpm())
                N += 1
            self.__controller.setArduinoThrusterVertical(0,0)
            
            time_ref = time.time()
            while time.time()-time_ref < 3:    
                self.__controller.setArduinoThrusterVertical(150,150)
                rpmT0_avg += numpy.abs(self.__controller.getT0rpm())
                rpmT1_avg += numpy.abs(self.__controller.getT1rpm())
                N += 1
            self.__controller.setArduinoThrusterVertical(0,0)
                
            rpmT0_avg = rpmT0_avg/float(N)
            rpmT1_avg = rpmT1_avg/float(N)
            
            if (rpmT0_avg < 50):
                str = "Problem with thruster 0. Average speed = %s" %rpmT0_avg
                rospy.logerr(str)
                return 'aborted'
            if (rpmT1_avg < 50):
                str = "Problem with thruster 1. Average speed = %s" %rpmT1_avg
                rospy.logerr(str)
                return 'aborted'
            str = "Thruster 0 - working"
            rospy.loginfo(str)
            str = "Thruster 1 - working"
            rospy.loginfo(str)
            
            #### Thruster 2 and 3 ####
            N = 0
            rpmT2_avg = 0
            rpmT3_avg = 0
            time_ref = time.time()
            while time.time()-time_ref < 3:    
                self.__controller.setArduinoThrusterHorizontal(150,150)
                rpmT2_avg += numpy.abs(self.__controller.getT2rpm())
                rpmT3_avg += numpy.abs(self.__controller.getT3rpm())
                N += 1
            self.__controller.setArduinoThrusterHorizontal(0,0)
            
            time_ref = time.time()
            while time.time()-time_ref < 3:    
                self.__controller.setArduinoThrusterHorizontal(-150,-150)
                rpmT2_avg += numpy.abs(self.__controller.getT2rpm())
                rpmT3_avg += numpy.abs(self.__controller.getT3rpm())
                N += 1
            self.__controller.setArduinoThrusterHorizontal(0,0)
                
            rpmT2_avg = rpmT2_avg/float(N)
            rpmT3_avg = rpmT3_avg/float(N)
            
            if (rpmT2_avg < 50):
                str = "Problem with thruster 2. Average speed = %s" %rpmT2_avg
                rospy.logerr(str)
                return 'aborted'
            if (rpmT3_avg < 50):
                str = "Problem with thruster 3. Average speed = %s" %rpmT3_avg
                rospy.logerr(str)
                return 'aborted'
            str = "Thruster 2 - working"
            rospy.loginfo(str)
            str = "Thruster 3 - working"
            rospy.loginfo(str)
            
            ####################################################################
            ### TAIL SECTION ###################################################
            ####################################################################

            angles = [-30.0, 0.0, 30.0]

            for a in angles:
                time_ref = time.time()
                while time.time()-time_ref<3:
                    self.__controller.setControlSurfaceAngle(a,a,a,a)
                
                if numpy.abs(self.__controller.getCS_b() - a) > 10.0:
                    str = "Problem with top control surface. Feedback = %d",self.__controller.getCS_b()
                    rospy.logerr(str)
                    return 'aborted' 
                if numpy.abs(self.__controller.getCS_c() - a) > 10.0:
                    str = "Problem with starboard control surface"
                    rospy.logerr(str)
                    return 'aborted' 
                if numpy.abs(self.__controller.getCS_d() - a) > 10.0:
                    str = "Problem with bottom control surface"
                    rospy.logerr(str)
                    return 'aborted' 
                if numpy.abs(self.__controller.getCS_e() - a) > 10.0:
                    str = "Problem with port control surface"
                    rospy.logerr(str)
                    return 'aborted'
            
            self.__controller.setControlSurfaceAngle(0,0,0,0)
            str = "Control surfaces - working"
            rospy.loginfo(str)
            time.sleep(1) # allow the control surfaces to get back to the neutral position
            
            N = 0
            RPM_avg = 0
            time_ref = time.time()
            while time.time()-time_ref<2:
                self.__controller.setRearProp(10)
                RPM_avg += self.__controller.getPropRPM()
                N += 1
            RPM_avg = RPM_avg/float(N)
            self.__controller.setRearProp(0)

            if RPM_avg < 3: # just an arbitrary integer that is not zero
                str = "Problem with rear prop - average rpm = %s" %RPM_avg
                rospy.logerr(str)
                self.__controller.setRearProp(0)
                return 'aborted'
            else:
                str = "Rear prop - working"
                rospy.loginfo(str)
                self.__controller.setRearProp(0)

            return 'succeeded'
                
            #return 'preempted'

            #return 'aborted'  
            
