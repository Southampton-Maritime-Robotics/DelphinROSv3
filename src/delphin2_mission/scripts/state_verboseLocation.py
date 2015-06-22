#!/usr/bin/env python

'''
######################################
# description
determine range and bearing to the specify location
will stop only when terminated

@return: preemped: if the backSeatErrorFlag has been raised

'''

import rospy
import numpy
import smach
import smach_ros
import time
from pylab import *
from math import *
from std_msgs.msg import String

class verboseLocation(smach.State):
    def __init__(self, lib, myUti, wp, controlRate):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller = lib
        self.__uti = myUti
        self.__wp = wp
        self.__controlRate = controlRate

    def execute(self, userdata):
        
        #Set Up Publisher for Mission Control Log
        pubMissionLog = rospy.Publisher('MissionStrings', String)
        
        ####################################################################
        ### Perform actions ################################################
        ####################################################################

        if self.__controlRate>0:
            r = rospy.Rate(self.__controlRate)

        while not rospy.is_shutdown() and self.__controller.getBackSeatErrorFlag() == 0:
            X = self.__controller.getX()
            Y = self.__controller.getY()
            Px = self.__wp[0]
            Py = self.__wp[1]
            rang, bear = self.__uti.rangeBearing([X,Y], [Px, Py])

            str = 'Current location: %s' %[X,Y]
            rospy.loginfo(str)
            pubMissionLog.publish(str)
            
            str = 'Target location: %s' %self.__wp
            rospy.loginfo(str)
            pubMissionLog.publish(str)
            
            str = 'Range to the point: %s deg' %rang
            rospy.loginfo(str)
            pubMissionLog.publish(str)
            
            str = 'Bearing to the point: %s m' %bear
            rospy.loginfo(str)
            pubMissionLog.publish(str)
            
            str = '##################################'
            rospy.loginfo(str)
            pubMissionLog.publish(str)
            
            if self.__controlRate > 0:
                r.sleep()

        if self.__controller.getBackSeatErrorFlag() == 1:
            str= 'manoeuvreSway preempted at time = %s' %(time.time())    
            rospy.loginfo(str)
            pubMissionLog.publish(str)
            return 'preempted'
        else: # this will never be used
            str= 'manoeuvreSway succeed at time = %s' %(time.time())    
            rospy.loginfo(str)
            pubMissionLog.publish(str)
            return 'succeeded'
