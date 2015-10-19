#!/usr/bin/env python

'''
Usage1: if stable_time is specified
Get the AUV to a desired depth and stay steady for some many seconds.

Usage1: if stable_time is -1
Keep publishing depth demand until a timeout criteria has been reached.

@return: preemped: if the backSeatErrorFlag has been raised
@return: succeeded: if goal has been satisfied
@return: aborted: if the timeout criteria has been reached

'''

import rospy
import smach
import smach_ros
import time
from std_msgs.msg import String

class GoToDepth(smach.State):
    def __init__(self, lib, depth_demand, stable_time, timeout):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller           = lib
        self.__depth_demand         = depth_demand
        self.__tolerance            = 0.2             # [m] a band that accounts as the AUV is at a desired depth
        self.__stable_time          = stable_time     # [sec] AUV must stay at a desired depth for this many seconds
        self.__timeout              = timeout         # [sec] abort criteria
        self.__controlRate          = 5               # [Hz]
        self.__at_depth_time        = None
            
    def execute(self,userdata):
        
        #Set Up Publisher for Mission Control Log
        pubMissionLog = rospy.Publisher('MissionStrings', String)

        # Set Up Loop Timing Control
        r = rospy.Rate(self.__controlRate)
        
        ######## START OPERATION ################################
            
        time_zero = time.time()
        
        str= 'Execute goToDepth State: Depth demand = %s, stable time = %s.' %(self.__depth_demand, self.__stable_time)
        pubMissionLog.publish(str)
        rospy.loginfo(str)
         
        at_depth = False
        self.__at_depth_time = time.time()   # a reference time for depth stady
        timeStart = time.time()              # a reference time for state timeout
        
        ##### Main loop #####
        while (time.time()-time_zero < self.__timeout) and self.__controller.getBackSeatErrorFlag() == 0 and time.time()-timeStart < self.__timeout:
            
            if stable_time != -1:
                at_depth = self.check_depth()
            
            if not at_depth:
                self.__controller.setDepth(self.__depth_demand)
            else:
                str= 'goToDepth succeeded at time = %s' %(time.time())
                pubMissionLog.publish(str)
                rospy.loginfo(str)
                return 'succeeded'
            r.sleep()
            
        if self.__controller.getBackSeatErrorFlag() == 1:
            str= 'goToDepth preempted at time = %s' %(time.time())    
            pubMissionLog.publish(str)
            rospy.loginfo(str)
            return 'preempted'
        else:
            if stable_time == -1:
                str= 'goToDepth succeeded at time = %s' %(time.time())
                pubMissionLog.publish(str)
                rospy.loginfo(str)
                return 'succeeded'
            else:
                str= 'goToDepth timed-out at time = %s' %(time.time())
                pubMissionLog.publish(str)
                rospy.loginfo(str)
                return 'aborted'  
            
    def check_depth(self):                
            
        # if the AUV lost a control in depth, reset at_depth_time
        if (abs(self.__controller.getDepth() - self.__depth_demand) >= self.__tolerance):
            self.__at_depth_time = time.time()
            
        if time.time()-self.__at_depth_time > self.__stable_time:
            return True
        else:
            return False
