#!/usr/bin/env python

'''
Usage1: if stable_time is specified
Get the AUV to a desired heading and stay steady for some many seconds.

Usage1: if stable_time is -1
Keep publishing heading demand until a timeout criteria has been reached.

@return: preemped: if the backSeatErrorFlag has been raised
@return: succeeded: if goal has been satisfied
@return: aborted: if the timeout criteria has been reached

'''

import rospy
import smach
import smach_ros
import time
from std_msgs.msg import String

class GoToHeading(smach.State):
    def __init__(self, lib, myUti, demandHeading, stable_time, timeout):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller       = lib
        self.__uti              = myUti
        self.__demandHeading    = demandHeading
        self.__tolerance        = 3                 # [deg] a band that accounts as the AUV is at a desired heading
        self.__stable_time      = stable_time       # AUV must stay at a desired heading for this many seconds
        self.__timeout          = timeout           # [sec] abort criteria
        self.__controlRate      = 5                 # [Hz]
        self.__at_heading_time  = None

    def execute(self,userdata):

        #Set Up Publisher for Mission Control Log
        pubMissionLog = rospy.Publisher('MissionStrings', String)
        
        # Set Up Loop Timing Control
        r = rospy.Rate(self.__controlRate)

        str='Execute GoToHeading State: heading demand = %.3f deg, stable time = %s.' %(self.__demandHeading, self.__stable_time)
        pubMissionLog.publish(str)
        rospy.loginfo(str)

        at_heading = False
        self.__at_heading_time = time.time() # a reference time for heading stady
        timeStart = time.time()              # a reference time for state timeout
        
        while not rospy.is_shutdown() and self.__controller.getBackSeatErrorFlag() == 0 and time.time()-timeStart < self.__timeout:
        
            if stable_time != -1:
                at_heading = check_heading()
                
            if not at_heading:
                self.__controller.setHeading(self.__demandHeading)
            else:
                str= 'goToHeading succeeded at time = %s' %(time.time())
                pubMissionLog.publish(str)
                rospy.loginfo(str)
                return 'succeeded'
                
            r.sleep()

        if self.__controller.getBackSeatErrorFlag() == 1:
            str= 'goForwards preempted at time = %s' %(time.time())
            pubMissionLog.publish(str)
            rospy.loginfo(str)
            return 'preempted'
        else:
            if stable_time == -1:
                str= 'goToHeading succeeded at time = %s' %(time.time())
                pub.publish(str)
                rospy.loginfo(str)
                return 'succeeded'
            else:
                str= 'goForwards aborted at time = %s' %(time.time())
                pubMissionLog.publish(str)
                rospy.loginfo(str)
                return 'aborted'
            
    def check_heading(self):                
            
        heading = self.__controller.getHeading()
        errHeading = self.__uti.computeHeadingError(self.__demandHeading,heading)

        # if the AUV lost a control in heading, reset at_heading_time
        if abs(errHeading)>self.__tolerance:
            self.__at_heading_time = time.time()

        if time.time()-self.__at_heading_time <= self.__stable_time:
            return True
        else:
            return False
