#!/usr/bin/env python

import roslib; roslib.load_manifest('delphin2_mission')
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
            self.__controller.switchVerticalThrusters(1)
            self.__controller.setArduinoThrusterVertical(600,600)
            time.sleep(3)
            self.__controller.setArduinoThrusterVertical(-600,-600)
            time.sleep(3)
            
            if (numpy.abs(self.__controller.getT0rpm()) < 400):
                str = "Problem with thruster 0. Speed = %s" %self.__controller.getT0rpm()
                rospy.logerr(str)
                #return 'aborted' #need to uncomment  
            else:
                str = "Thruster 0 - working"
                rospy.loginfo(str)
            
            if (numpy.abs(self.__controller.getT1rpm()) < 400):
                str = "Problem with thruster 1. Speed = %s" %self.__controller.getT1rpm()
                rospy.logerr(str)
                #return 'aborted' #need to comment 
            else:
                str = "Thruster 1 - working"
                rospy.loginfo(str)
                
            self.__controller.switchVerticalThrusters(0)

            
            #### Thruster 2 and 3 ####
            self.__controller.switchHorizontalThrusters(1)
            self.__controller.setArduinoThrusterHorizontal(600,600)
            time.sleep(3)
            self.__controller.setArduinoThrusterHorizontal(-600,-600)
            time.sleep(3)
            
            if (numpy.abs(self.__controller.getT2rpm()) < 400):
                str = "Problem with thruster 2. Speed = %s" %self.__controller.getT2rpm()
                rospy.logerr(str)
                #return 'aborted' #need to uncomment 
            else:
                str = "Thruster 2 - working"
                rospy.loginfo(str)
            
            if (numpy.abs(self.__controller.getT3rpm()) < 400):
                str = "Problem with thruster 3. Speed = %s" %self.__controller.getT3rpm()
                rospy.logerr(str)
                #return 'aborted' #need to uncomment  
            else:
                str = "Thruster 3 - working"
                rospy.loginfo(str)

            self.__controller.switchHorizontalThrusters(0)
            
            ####################################################################
            ### TAIL SECTION ###################################################
            ####################################################################

            angles = [-30.0, 0.0, 30.0]

            for a in angles:
                self.__controller.setControlSurfaceAngle(a,a,a,a)
                time.sleep(3)
                
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
            
            self.__controller.setRearProp(10)
            time.sleep(2)
            
            if self.__controller.getPropRPM() < 190:
                str = "Problem with rear prop"
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
            
