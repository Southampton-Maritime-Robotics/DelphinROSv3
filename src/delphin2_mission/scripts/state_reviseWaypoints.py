#!/usr/bin/env python

'''
Insert the current position of the AUV into the begining of the path input.

@return: succeeded: when the operation is complete
@return: aborted: when internal error occurs
@return: preempted: not being used

'''

import rospy
import smach
import smach_ros
import time
import numpy as np
from std_msgs.msg import String

class reviseWaypoints(smach.State):
    def __init__(self, lib, wp):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                                   output_keys=['wp_out'])
        self.__controller           = lib
        self.__wp                   = wp
        
        #Set Up Publisher for Mission Control Log
        self.pubMissionLog = rospy.Publisher('MissionStrings', String, queue_size=10)
        
    def displayInput(self):
        ## Display the raw input on the screen
        if self.__wp.size<4:
            text= 'Execute reviseWaypoints State with input: \n [X    :    Y] \n %s' %(self.__wp)
        else:
            text= 'Execute reviseWaypoints State with input: \n [X    :    Y] \n %s' %(self.__wp.T)
        self.pubMissionLog.publish(text)
        rospy.loginfo(text)
        
    def getCurrentLocation(self):
        # Get a current location of the AUV
        X_now = self.__controller.getX()
        Y_now = self.__controller.getY()
        p = np.array([X_now, Y_now])
        
        text= 'Current location: %s' %(p)
        self.pubMissionLog.publish(text)
        rospy.loginfo(text)
        
        return p
        
    def insertCurrentLocationIntoPath(self, p):
        # add a current location into the begining of the path
        if self.__wp.size==2: # If the input is a waypoint
            wp_out = np.vstack(( p , self.__wp)).T
        else: # If the input consists of 2 waypoints or more
            wp_out = np.vstack(( p , self.__wp.T )).T
        
        return wp_out
        
    def execute(self,userdata):
        ######## START OPERATION ################################            
        self.displayInput()

        
        try:
            p = self.getCurrentLocation()
            wp_out = self.insertCurrentLocationIntoPath(p)
            userdata.wp_out = wp_out
            
            # Exit state with a succeeded outcome
            text= 'reviseWaypoints State succeeded'
            self.pubMissionLog.publish(text)
            rospy.loginfo(text)
            return 'succeeded'
        except:
            # Exit state with a succeeded outcome
            text= 'reviseWaypoints State aborted: check path input, it has to be 2 by n numpy array'
            self.pubMissionLog.publish(text)
            rospy.loginfo(text)
            return 'aborted'
