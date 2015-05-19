#!/usr/bin/env python

'''
# Dry Land Test:
test of all sensors and actuators before the vehicle is run in water
- run thrusters and propeller at lower RPM

######################################
#Modifications
12/May/2015 use the feedback signal to identify if the actuators is functioning

'''

import rospy
import numpy
import smach
import smach_ros
import time
import cv
#from opencv.highgui import *

class dryLandTest(smach.State):
    def __init__(self, lib):
            smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
            self.__controller = lib
            
    def execute(self, userdata):
            
            ####################################################################
            ### CAMERAS ########################################################
            ####################################################################
#            capture0 = cv.CaptureFromCAM(0)
#            capture1 = cv.CaptureFromCAM(1)
#            cv.SetCaptureProperty(capture0, CV_CAP_PROP_FRAME_WIDTH, 352)
#            cv.SetCaptureProperty(capture0, CV_CAP_PROP_FRAME_HEIGHT, 288)
#            cv.SetCaptureProperty(capture1, CV_CAP_PROP_FRAME_WIDTH, 352)
#            cv.SetCaptureProperty(capture1, CV_CAP_PROP_FRAME_HEIGHT, 288)
            
#            cam0 = cv.QueryFrame(capture0)
#            cam1 = cv.QueryFrame(capture0)
            
#            print 'cam0 size = ',cv.GetSize(cam0)
#            print 'cam1 size = ',cv.GetSize(cam1)
            
#            if (cam0.height == 288 and cam0.width == 352) or (cam0.height == 288*2 and cam0.width == 352*2):
#                str = "Forwards looking camera - working"
#                rospy.loginfo(str)
#            else:
#                str = "Problem with forwards looking camera"
#                rospy.logerr(str)
 #               str = "SET CAMERAS TO PAL!!! (cd ~/ ... sudo ./cameras.sh)"
#                rospy.logerr(str)
                
#            if (cam1.height == 288 and cam1.width == 352) or (cam1.height == 288*2 and cam1.width == 352*2):
#                str = "Forwards looking camera - working"
#                rospy.loginfo(str)
#            else:
#                str = "Problem with forwards looking camera"
#                rospy.loginfo(str)
#                str = "SET CAMERAS TO PAL!!! (cd ~/ ... sudo ./cameras.sh)"
#                rospy.logerr(str)
#                return 'aborted'
                
            
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
                self.__controller.setArduinoThrusterVertical(-600,-600)
                rpmT0_avg += numpy.abs(self.__controller.getT0rpm())
                rpmT1_avg += numpy.abs(self.__controller.getT1rpm())
                N += 1
            self.__controller.setArduinoThrusterVertical(0,0)
            
            time_ref = time.time()
            while time.time()-time_ref < 3:    
                self.__controller.setArduinoThrusterVertical(600,600)
                rpmT0_avg += numpy.abs(self.__controller.getT0rpm())
                rpmT1_avg += numpy.abs(self.__controller.getT1rpm())
                N += 1
            self.__controller.setArduinoThrusterVertical(0,0)
                
            rpmT0_avg = rpmT0_avg/float(N)
            rpmT1_avg = rpmT1_avg/float(N)
            
            if (rpmT0_avg < 100):
                str = "Problem with thruster 0. Average speed = %s" %rpmT0_avg
                rospy.logerr(str)
                return 'aborted'
            if (rpmT1_avg < 100):
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
                self.__controller.setArduinoThrusterHorizontal(600,600)
                rpmT2_avg += numpy.abs(self.__controller.getT2rpm())
                rpmT3_avg += numpy.abs(self.__controller.getT3rpm())
                N += 1
            self.__controller.setArduinoThrusterHorizontal(0,0)
            
            time_ref = time.time()
            while time.time()-time_ref < 3:    
                self.__controller.setArduinoThrusterHorizontal(-600,-600)
                rpmT2_avg += numpy.abs(self.__controller.getT2rpm())
                rpmT3_avg += numpy.abs(self.__controller.getT3rpm())
                N += 1
            self.__controller.setArduinoThrusterHorizontal(0,0)
                
            rpmT2_avg = rpmT2_avg/float(N)
            rpmT3_avg = rpmT3_avg/float(N)
            
            if (rpmT2_avg < 100):
                str = "Problem with thruster 2. Average speed = %s" %rpmT2_avg
                rospy.logerr(str)
                return 'aborted'
            if (rpmT3_avg < 100):
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
            
