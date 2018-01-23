#!/usr/bin/env python

'''
Usage1 - stabilising: if stable_time is specified
Get the AUV to a desired depth and stay steady for some many seconds.
@return: preempted: the backSeatErrorFlag has been raised
@return: succeeded: stable at the depth long enough withing the timeout
@return: aborted: not stable at the depth long enough withing the timeout

Usage2 - reaching: if stable_time is -1
Keep publishing depth demand until a timeout criteria has been reached.
@return: preempted: the backSeatErrorFlag has been raised
@return: succeeded: the depth has been reached withing the timeout
@return: aborted: the depth has not been reached withing the timeout, or if other node that has a higher priority and running in paraller has finished

'''

import rospy
import smach
import smach_ros
import time
from std_msgs.msg import String

class GoToDepth(smach.State):
    def __init__(self, lib, demandDepth, stable_time, timeout):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller           = lib
        self.__depth_demand         = demandDepth
        self.__tolerance            = rospy.get_param("depth/Tolerance")             # [m] a band that accounts as the AUV is at a desired depth
        self.__stable_time          = stable_time     # [sec] AUV must stay at a desired depth for this many seconds
        self.__timeout              = timeout         # [sec] abort criteria
        self.__controlRate          = rospy.get_param("depth/ControlRate")               # [Hz]
        self.__at_depth_time        = None
            
    def execute(self,userdata):
        
        #Set Up Publisher for Mission Control Log
        pubMissionLog = rospy.Publisher('MissionStrings', String)

        # Set Up Loop Timing Control
        r = rospy.Rate(self.__controlRate)
        
        ######## START OPERATION ################################
            
        time_zero = time.time()
        
        text= 'Execute GoToDepth State: Depth demand = %s, stable time = %s, timeout = %s.' %(self.__depth_demand, self.__stable_time, self.__timeout)
        pubMissionLog.publish(text)
        rospy.loginfo(text)
        
        self.__at_depth_time = time.time()   # a reference time for depth stady
        timeStart = time.time()              # a reference time for state timeout
        
        ##### Main loop #####
        at_depth_reached, at_depth_stable = self.check_depth() # initialise, in case timeout = 0
        while (time.time()-time_zero < self.__timeout) and self.__controller.getBackSeatErrorFlag() == 0 and time.time()-timeStart < self.__timeout:
            if self.preempt_requested():
                text = "Force Exit GoToDepth!!!"
                pubMissionLog.publish(text)
                rospy.loginfo(text)
                self.service_preempt()
                return 'aborted'
                
            at_depth_reached, at_depth_stable = self.check_depth()
            self.__controller.setDepth(self.__depth_demand)
            
            if self.__stable_time != -1 and at_depth_stable:
                text = 'goToDepth - stabilising: succeeded'
                pubMissionLog.publish(text)
                rospy.loginfo(text)
                return 'succeeded'
                
            r.sleep()
            
        if self.__controller.getBackSeatErrorFlag() == 1:
            text = 'goToDepth preempted'   
            pubMissionLog.publish(text)
            rospy.loginfo(text)
            return 'preempted'
        else:
            if self.__stable_time == -1: # TODO: add a condition to clarify whether the depth has been reached or not
                if at_depth_reached:
                    text= 'goToDepth - reaching: succeeded'
                    pubMissionLog.publish(text)
                    rospy.loginfo(text)
                    return 'succeeded'
                else:
                    text= 'goToDepth - reaching: aborted'
                    pubMissionLog.publish(text)
                    rospy.loginfo(text)
                    return 'aborted'
            else:
                text= 'goToDepth - stabilising: timed-out'
                pubMissionLog.publish(text)
                rospy.loginfo(text)
                return 'aborted'
            
    def check_depth(self):
        
        depthNow = self.__controller.getDepth()
        errDepth = self.__depth_demand - depthNow
        
        # verify if the depth has been reached
        if abs(errDepth) > self.__tolerance:
            self.__at_depth_time = time.time()
            at_depth_reached = False
        else:
            at_depth_reached = True
            
        # verify if the depth is stable long enough
        if time.time()-self.__at_depth_time <= self.__stable_time:
            at_depth_stable = False
        else:
            at_depth_stable = True
            
        return at_depth_reached, at_depth_stable
