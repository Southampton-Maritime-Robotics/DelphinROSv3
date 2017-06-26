#!/usr/bin/env python

'''
######################################
# description
determine range and bearing to the specify location
will stop only when terminated

@return: preempted: if the backSeatErrorFlag has been raised

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
        pubMissionLog = rospy.Publisher('MissionStrings', String, queue_size=10)
        
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

            text = 'Current location: %s' %[X,Y]
            rospy.loginfo(text)
            pubMissionLog.publish(text)
            
            text = 'Target location: %s' %self.__wp
            rospy.loginfo(text)
            pubMissionLog.publish(text)
            
            text = 'Range to the point: %s m' %rang
            rospy.loginfo(text)
            pubMissionLog.publish(text)
            
            text = 'Bearing to the point: %s deg' %bear
            rospy.loginfo(text)
            pubMissionLog.publish(text)
            
            text = '##################################'
            rospy.loginfo(text)
            pubMissionLog.publish(text)
            
            if self.__controlRate > 0:
                r.sleep()

        if self.__controller.getBackSeatErrorFlag() == 1:
            text= 'manoeuvreSway preempted at time = %s' %(time.time())    
            rospy.loginfo(text)
            pubMissionLog.publish(text)
            return 'preempted'
        else: # this will never be used
            text= 'manoeuvreSway succeed at time = %s' %(time.time())    
            rospy.loginfo(text)
            pubMissionLog.publish(text)
            return 'succeeded'
