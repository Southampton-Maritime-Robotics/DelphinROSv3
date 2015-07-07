#!/usr/bin/env python

'''
Get the AUV to a certain heading.

@return: preemped: if the backSeatErrorFlag has been raised
@return: succeeded: if the AUV is stable at the heading for a certain period of time
@return: aborted: if the timeout criteria has been reached

# TODO
- may need to assign the depthDemand at the same time

'''

import rospy
import smach
import smach_ros
import time
from std_msgs.msg import String

class GoToHeading(smach.State):
    def __init__(self, lib, myUti, demandHeading, tolerance, stable_time, timeout, controlRate):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller = lib
        self.__uti = myUti
        self.__demandHeading = demandHeading
        self.__tolerance = tolerance
        self.__stable_time = stable_time
        self.__timeout = timeout
        self.__controlRate = controlRate

    def execute(self,userdata):

        #Set Up Publisher for Mission Control Log
        pubMissionLog = rospy.Publisher('MissionStrings', String)
        
        if self.__controlRate>0:
            r = rospy.Rate(self.__controlRate)

        str='Entered GoToHeading State with a demandHeading = %.3f deg' %(self.__demandHeading)
        pubMissionLog.publish(str)
        rospy.loginfo(str)

        timeStart = time.time()
        while not rospy.is_shutdown() and self.__controller.getBackSeatErrorFlag() == 0 and time.time()-timeStart < self.__timeout:
        
            heading = self.__controller.getHeading()
            errHeading = self.__uti.computeHeadingError(self.__demandHeading,heading)
            if abs(errHeading)>self.__tolerance:
                timeStart = time.time()
            if time.time()-timeStart <= self.__stable_time:
                self.__controller.setHeading(self.__demandHeading)
            else:
                # when the heading is steady, move onto the next step
                self.__controller.setRearProp(0)
                self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
                self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
                str= 'goForwards succeeded at time = %s' %(time.time())
                pubMissionLog.publish(str)
                rospy.loginfo(str)
                return 'succeeded'
                
            if self.__controlRate>0:
                r.sleep()

        self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
        
        if self.__controller.getBackSeatErrorFlag() == 1:
            str= 'goForwards preempted at time = %s' %(time.time())
            pubMissionLog.publish(str)
            rospy.loginfo(str)
            return 'preempted'
        else:
            str= 'goForwards aborted at time = %s' %(time.time())
            pubMissionLog.publish(str)
            rospy.loginfo(str)
            return 'aborted'
