#!/usr/bin/env python

'''
Set heading demand to None to track the current heading; Otherwise, specify the value for the heading demand.

Usage1 - stabilising: if stable_time is specified
Get the AUV to a desired heading and stay steady for some many seconds.
@return: preempted: the backSeatErrorFlag has been raised
@return: succeeded: stable at the heading long enough withing the timeout
@return: aborted: not stable at the heading long enough withing the timeout

Usage2 - reaching: if stable_time is -1
Keep publishing heading demand until a timeout criteria has been reached.
@return: preempted: the backSeatErrorFlag has been raised
@return: succeeded: the heading has been reached withing the timeout
@return: aborted: the heading has not been reached withing the timeout, or if other node that has a higher priority and running in paraller has finished

'''

import rospy
import smach
import smach_ros
import time
from std_msgs.msg import String

class GoToHeading(smach.State):
    def __init__(self, lib, myUti, demandHeading, time_steady, timeout):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller       = lib
        self.__uti              = myUti
        self.__demandHeading    = demandHeading
        self.__tolerance        = rospy.get_param("heading/Tolerance")                 # [deg] a band that accounts as the AUV is at a desired heading
        self.__time_steady      = time_steady       # AUV must stay at a desired heading for this many seconds
        self.__timeout          = timeout           # [sec] abort criteria
        self.__controlRate      = rospy.get_param*"heading/ControlRate")                 # [Hz]
        self.__at_heading_time  = None
        
    def execute(self,userdata):

        #Set Up Publisher for Mission Control Log
        pubMissionLog = rospy.Publisher('MissionStrings', String)
        
        # Set Up Loop Timing Control
        r = rospy.Rate(self.__controlRate)

        # If the heading demand is not specified, use the current heading instead.
        if self.__demandHeading == None: # tracking a current heading
            self.__demandHeading = self.__controller.getHeading()
        # Otherwise ensure that the demand is within [0,360)
        else:
            self.__demandHeading = self.__demandHeading%360

        str='Execute GoToHeading State: heading demand = %.3f deg, stable time = %s, timeout = %s.' %(self.__demandHeading, self.__stable_time, self.__timeout)
        pubMissionLog.publish(str)
        rospy.loginfo(str)
        
        self.__at_heading_time = time.time() # a reference time for heading stady
        timeStart = time.time()              # a reference time for state timeout
        
        while not rospy.is_shutdown() and self.__controller.getBackSeatErrorFlag() == 0 and time.time()-timeStart < self.__timeout:
            if self.preempt_requested():
                str = "Force Exit GoToHeading!!!"
                pubMissionLog.publish(str)
                rospy.loginfo(str)
                self.service_preempt()
                return 'aborted'
                
            at_heading_reached, at_heading_stable = self.check_heading()
            self.__controller.setHeading(self.__demandHeading)
            
            if self.__time_steady != -1 and at_heading_stable:
                str= 'goToHeading - stabilising: succeeded'
                pubMissionLog.publish(str)
                rospy.loginfo(str)
                return 'succeeded'
                
            r.sleep()

        if self.__controller.getBackSeatErrorFlag() == 1:
            str= 'goToHeading preempted'
            pubMissionLog.publish(str)
            rospy.loginfo(str)
            return 'preempted'
        else:
            if self.__time_steady == -1:
                if at_heading_reached:
                    str= 'goToHeading - reaching: succeeded'
                    pubMissionLog.publish(str)
                    rospy.loginfo(str)
                    return 'succeeded'
                else:
                    str= 'goToHeading - reaching: aborted'
                    pubMissionLog.publish(str)
                    rospy.loginfo(str)
                    return 'aborted'
            else:
                str= 'goForwards - stabilising: aborted'
                pubMissionLog.publish(str)
                rospy.loginfo(str)
                return 'aborted'
            
    def check_heading(self):
        
        headingNow = self.__controller.getHeading()
        errHeading = self.__uti.computeHeadingError(self.__demandHeading,headingNow)

        # verify if the heading has been reached
        if abs(errHeading) > self.__tolerance:
            self.__at_heading_time = time.time()
            at_heading_reached = False
        else:
            at_heading_reached = True
            
        # verify if the heading is stable long enough
        if time.time()-self.__at_heading_time <= self.__time_steady:
            at_heading_stable = False
        else:
            at_heading_stable = True
        
        return at_heading_reached, at_heading_stable
