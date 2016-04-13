#!/usr/bin/env python

'''
Wait for a valid gps fix.

@return: succeeded: a valid gps fix is received
@return: aborted: internal error occur
@return: preempted: when backSeatDriver flag is raised

'''

import rospy
import smach
import smach_ros
import time
from std_msgs.msg import String

class waitForGPS(smach.State):
    def __init__(self, lib, timeout):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])
        self.__controller           = lib
        self.__controlRate          = 5         # [Hz]
        self.__timeout              = timeout   # [sec] timeout to check if the GPS comes online
        
        #Set Up Publisher for Mission Control Log
        self.pubMissionLog = rospy.Publisher('MissionStrings', String)
        
    def execute(self,userdata):
        ######## START OPERATION ################################            
        # Set Up Loop Timing Control
        r = rospy.Rate(self.__controlRate)
        
        timeStartWait = time.time()
        while time.time()-timeStartWait < self.__timeout and not rospy.is_shutdown() and self.__controller.getBackSeatErrorFlag() == 0:
            if not self.__controller.getGPSValidFix(): # If gps fixed is not detected ...
                r.sleep()
            else:
                str = 'reviseWaypoints succeeded'
                rospy.loginfo(str)
                self.pubMissionLog.publish(str)
                return 'succeeded'
                
        if self.__controller.getBackSeatErrorFlag() != 0:
            str = 'reviseWaypoints preempted'
            rospy.loginfo(str)
            self.pubMissionLog.publish(str)
            return 'preempted'
        else:
            str = 'reviseWaypoints aborted'
            rospy.loginfo(str)
            self.pubMissionLog.publish(str)
            return 'aborted'