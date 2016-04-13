#!/usr/bin/env python

'''
A state that directly operates the actuators in a certain way for a certain period of time. 

This is very useful when recovering the auv!!!

user needs to specify
-self.__actionHold: how long the action will be held
-self.__controller.setRearProp(0)
-self.__controller.setRudderAngle(0)
-self.__controller.setSternPlaneAngle(0)
-self.__controller.setArduinoThrusterVertical(0,0) # (FrontVer,RearVer)
-self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
-more commands are available in library_highlevel

@return: preempted: if the backSeatErrorFlag has been raised
@return: succeeded: if the timeout criteria has been reached (for other states, timeout will lead to mission aborted)
@return: aborted: not in use

'''

import rospy
import numpy
import smach
import smach_ros
import time
from std_msgs.msg import String

class actions(smach.State):
    def __init__(self, lib, demand_prop, demand_th_ver, demand_th_hor, demand_cs_ver, demand_cs_hor, actionHold):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller  = lib
        self.__actionHold  = actionHold # let the vehicle doing those actions for a period of time (value is specified in second)
        self.__controlRate = 5          # [Hz]
        self.__prop = demand_prop
        self.__th_0 = demand_th_ver[0]
        self.__th_1 = demand_th_ver[1]
        self.__th_2 = demand_th_hor[0]
        self.__th_3 = demand_th_hor[1]
        self.__cs_ver = demand_cs_ver
        self.__cs_hor = demand_cs_hor
        
    def execute(self, userdata):

        #Set Up Publisher for Mission Control Log
        pubMissionLog = rospy.Publisher('MissionStrings', String)

        ####################################################################
        ### Perform actions ################################################
        ####################################################################

        # let the vehicle doing those actions for a period of time
        # and shutdown the actuators once finished
        
        controlPeriod = 1./self.__controlRate
        r = rospy.Rate(self.__controlRate)

        str = 'Execute state actions and hold for %ss' %self.__actionHold
        rospy.loginfo(str)
        pubMissionLog.publish(str)        
        str = 'prop demand: %s' %self.__prop
        rospy.loginfo(str)
        pubMissionLog.publish(str)
        str = 'thruster demand: [%s, %s, %s, %s]' %(self.__th_0,self.__th_1,self.__th_2,self.__th_3)
        rospy.loginfo(str)
        pubMissionLog.publish(str)
        str = 'fins demand: [ver=%s, hor=%s]' %(self.__cs_ver, self.__cs_hor)
        rospy.loginfo(str)
        pubMissionLog.publish(str)
        
        timeStart = time.time()
        while not rospy.is_shutdown() and time.time()-timeStart < self.__actionHold:
            if self.__controller.getBackSeatErrorFlag() == 1:
                str= 'state_actions preempted'   
                rospy.loginfo(str)
                pubMissionLog.publish(str)
                return 'preempted'
            else:
                # Demands may interfere the controllers or other states. Publish only the ones that are required.
                if self.__prop != 0:
                    self.__controller.setRearProp(self.__prop)
                # depth-pitch
                if self.__th_0 != 0 or self.__th_1 != 0:
                    self.__controller.setArduinoThrusterVertical(self.__th_0,self.__th_1) # (FrontVer,RearVer)
                if self.__cs_hor != 0:
                    self.__controller.setSternPlaneAngle(self.__cs_hor)
                # sway-yaw
                if self.__th_2 != 0 or self.__th_3 != 0:
                    self.__controller.setArduinoThrusterHorizontal(self.__th_2,self.__th_3) # (FrontHor,RearHor)
                if self.__cs_ver != 0:
                    self.__controller.setRudderAngle(self.__cs_ver)

            r.sleep()
            
        str= 'state_actions succeeded'  
        rospy.loginfo(str)
        pubMissionLog.publish(str)
        return 'succeeded' # exit with a flag of 'succeeded'
