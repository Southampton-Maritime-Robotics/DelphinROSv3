#!/usr/bin/env python

import roslib; roslib.load_manifest('DelphinROSv2')
import rospy
import numpy
import smach
import smach_ros
import time


class windtunnel(smach.State):
    def __init__(self, lib):
            smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
            self.__controller = lib
            
    def execute(self, userdata):
            
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
            
