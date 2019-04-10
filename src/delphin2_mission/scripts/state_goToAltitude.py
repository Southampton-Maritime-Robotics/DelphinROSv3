#!/usr/bin/env python

'''
Usage1 - stabilising: if stable_time is specified
Get the AUV to a desired altitude and stay steady for some many seconds.
@return: preempted: the backSeatErrorFlag has been raised
@return: succeeded: stable at the altitude long enough withing the timeout
@return: aborted: not stable at the altitude long enough withing the timeout

Usage2 - reaching: if stable_time is -1
Keep publishing depth demand until a timeout criteria has been reached.
@return: preempted: the backSeatErrorFlag has been raised
@return: succeeded: the altitude has been reached withing the timeout
@return: aborted: the altitude has not been reached withing the timeout, or if other node that has a higher priority and running in paraller has finished

'''

import rospy
import smach
import smach_ros
import time
from std_msgs.msg import String


class GoToAltitude(smach.State):
    def __init__(self, lib, demandAltitude, stable_time, timeout):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller           = lib
        self.__altitude_demand      = demandAltitude
        self.__tolerance            = 0.2             # [m] a band that accounts as the AUV is at a desired altitude
        self.__stable_time          = stable_time     # [sec] AUV must stay at a desired altitude for this many seconds
        self.__timeout              = timeout         # [sec] abort criteria
        self.__controlRate          = 5               # [Hz]
        self.__at_altitude_time        = None
            
    def execute(self,userdata):
        
        #Set Up Publisher for Mission Control Log
        pubMissionLog = rospy.Publisher('MissionStrings', String)

        # Set Up Loop Timing Control
        r = rospy.Rate(self.__controlRate)
        
        ######## START OPERATION ################################
            
        time_zero = time.time()
        
        text= 'Execute GoToAltitude State: Altitude demand = %s, stable time = %s.' %(self.__altitude_demand, self.__stable_time)
        pubMissionLog.publish(text)
        rospy.loginfo(text)
        
        self.__at_altitude_time = time.time()   # a reference time for altitude steady
        timeStart = time.time()              # a reference time for state timeout
        
        ##### Main loop #####
        at_altitude_reached, at_altitude_stable, depth_demand = self.check_Altitude() # initialise, in case timeout = 0
        while (time.time()-time_zero < self.__timeout) and self.__controller.getBackSeatErrorFlag() == 0 and time.time()-timeStart < self.__timeout:
            if self.preempt_requested():
                text = "Force Exit GoToAltitude!!!"
                pubMissionLog.publish(text)
                rospy.loginfo(text)
                self.service_preempt()
                return 'aborted'
                
            # check altitude and update depth demand
            at_altitude_reached, at_altitude_stable, depth_demand = self.check_Altitude()
            self.__controller.setDepth(depth_demand)
            
            if self.__stable_time != -1 and at_altitude_stable:
                text= 'goToAltitude - stabilising: succeeded'
                pubMissionLog.publish(text)
                rospy.loginfo(text)
                return 'succeeded'
                
            r.sleep()
            
        if self.__controller.getBackSeatErrorFlag() == 1:
            text= 'goToAltitude preempted'   
            pubMissionLog.publish(text)
            rospy.loginfo(text)
            return 'preempted'
        else:
            if self.__stable_time == -1: # TODO: add a condition to clarify whether the altitude has been reached or not
                if at_altitude_reached:
                    text= 'goToAltitude - reaching: succeeded'
                    pubMissionLog.publish(text)
                    rospy.loginfo(text)
                    return 'succeeded'
                else:
                    text= 'goToAltitude - reaching: aborted'
                    pubMissionLog.publish(text)
                    rospy.loginfo(text)
                    return 'aborted'
            else:
                text= 'goToAltitude - stabilising: timed-out'
                pubMissionLog.publish(text)
                rospy.loginfo(text)
                return 'aborted'
            
    def check_Altitude(self):
        
        depthNow = self.__controller.getDepth()
        real_altitude = self.__controller.getAltitude()
        # use pseudo altitude for altitude tracking:
        pseudo_altitude = self.__controller.getPseudoAltitude()
        altitudeNow = real_altitude  # use altitude
        # altitudeNow = min(pseudo_altitude, real_altitude) # use pseudo altitude
        errAltitude = altitudeNow - self.__altitude_demand
        depth_demand = depthNow + errAltitude
        
        # verify if the depth has been reached
        if abs(errAltitude) > self.__tolerance:
            self.__at_altitude_time = time.time()
            at_altitude_reached = False
        else:
            at_altitude_reached = True
            
        # verify if the altitude is stable long enough
        if time.time()-self.__at_altitude_time <= self.__stable_time:
            at_altitude_stable = False
        else:
            at_altitude_stable = True
            
        return at_altitude_reached, at_altitude_stable, depth_demand
