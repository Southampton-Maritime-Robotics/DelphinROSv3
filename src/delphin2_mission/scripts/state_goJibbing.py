#!/usr/bin/env python

'''
Get the AUV to turn by using either horizontal thruster or rudder, or using both set of actuators simultaneously.

If the demand for a particular set of actuators is zero, the demand of this set will not be published !!!

@return: preempted: if the backSeatErrorFlag has been raised
@return: succeeded: if the timeout criteria has been reached (for other states, timeout will lead to mission aborted)
@return: aborted: if other node that has a higher priority and running in paraller has finished

'''

from __future__ import division
import rospy
import smach
import smach_ros
import time
from std_msgs.msg import String

class GoJibbing(smach.State):
    def __init__(self, lib, uti, headingMean, headingAmp, demand_th_hor, demand_cs_ver, cycleMax, timeout):
        smach.State.__init__(self, outcomes=['succeeded','continue','aborted','preempted'],
                                   input_keys=['cycleCount_in','dir_in'],
                                   output_keys=['cycleCount_out','dir_out'])
        self.__controller           = lib
        self.__myUti                = uti
        self.__headingMean          = headingMean%360
        self.__headingAmp           = headingAmp
        self.__th_hor               = demand_th_hor
        self.__rudder               = demand_cs_ver
        self.__cycleMax             = cycleMax
        self.__timeout              = timeout
        self.__controlRate          = 5 # [Hz]
        
        # Set Up Loop Timing Control
        self.__r = rospy.Rate(self.__controlRate)    

        #Set Up Publisher for Mission Control Log
        self.__pubMissionLog = rospy.Publisher('MissionStrings', String, queue_size=10)

    def execute(self,userdata):
        
        cycleCount = userdata.cycleCount_in  # a counter to count how many time jibbing has been done.
        dirNow = userdata.dir_in        # direction of turning

        text='Execute GoJibbing State: \n th_hor = %s \n rudder = %s \n heading = %s +- %s \n cycle = %s/%s' %(self.__th_hor, self.__rudder, self.__headingMean, self.__headingAmp, cycleCount, self.__cycleMax)
        self.__pubMissionLog.publish(text)
        rospy.loginfo(text)
        
        timeStart = time.time()
        # determine a heading reference and confine it within [0,360)
        headingRef = (self.__headingMean+dirNow*self.__headingAmp)%360
        while not rospy.is_shutdown() and self.__controller.getBackSeatErrorFlag() == 0 and time.time()-timeStart < self.__timeout:
            if self.preempt_requested():
                text = "Force Exit GoJibbing!!!"
                self.__pubMissionLog.publish(text)
                rospy.loginfo(text)
                self.service_preempt()
                return 'aborted'
            
            heading = self.__controller.getHeading()
            headingErr = self.__myUti.computeHeadingError(headingRef, heading)
            if headingErr*dirNow <= 0:
                dirNow *= -1
                cycleCount += 1
                userdata.dir_out = dirNow
                userdata.cycleCount_out = cycleCount
                
                if cycleCount > self.__cycleMax:
                    self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontVer,RearVer)
                    self.__controller.setRudderAngle(0)
                    text = "goJibbing succeeded"
                    self.__pubMissionLog.publish(text)
                    rospy.loginfo(text)
                    self.service_preempt()
                    return 'succeeded'
                else:
                    text = "goJibbing reached a heading reference: switch a direction and continue"
                    self.__pubMissionLog.publish(text)
                    rospy.loginfo(text)
                    return 'continue'
            else:
                self.__controller.setArduinoThrusterHorizontal(self.__th_hor*dirNow,-self.__th_hor*dirNow)
                self.__controller.setRudderAngle(self.__rudder*dirNow)
                
            self.__r.sleep()

        if self.__controller.getBackSeatErrorFlag() == 1:
            text= 'GoTurning preempted'
            self.__pubMissionLog.publish(text)
            rospy.loginfo(text)
            return 'preempted'
        else:
            text= 'time-out - GoTurning aborted'
            self.__pubMissionLog.publish(text)
            rospy.loginfo(text)
            return 'aborted'
