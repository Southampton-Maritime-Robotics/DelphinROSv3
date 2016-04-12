#!/usr/bin/env python

'''
A state to get that analyses a given waypoint(s) and modify in according to the current location of the AUV.

Let's denote a current location of the AUV as p, there are three possible scenarios:
- Given a point (B): current location, p, is used to form a segment, p-->B.
- Given a segment (A-->B): If the auv has already gone beyond point B, point A will be replaced by point p; Otherwise, use the path as it is.
- Given a path: unneccessary parts of the path will be trimmed down, and the remaining part will be revised in a similar manner to scenario 1 or 2 as to find a best way to enter the path.

The outcome could be either segment or path
@return: segment: if the final result consists of two waypoints
@return: path: if the final result consists of three waypoints or more

'''

import rospy
import smach
import smach_ros
import time
import numpy as np
from std_msgs.msg import String

class reviseWaypoints(smach.State):
    def __init__(self, lib, myUti, wp):
        smach.State.__init__(self, outcomes=['segment', 'path'],
                                   output_keys=['wp_out'])
        self.__controller           = lib
        self.__myUti                = myUti
        self.__wp                   = wp
        #Set Up Publisher for Mission Control Log
        self.pubMissionLog = rospy.Publisher('MissionStrings', String)
        
    def displayInput(self):
        ## Display the raw input on the screen
        if self.__wp.size<4:
            str= 'Execute reviseWaypoints State with input: \n [X    :    Y] \n %s' %(self.__wp)
        else:
            str= 'Execute reviseWaypoints State with input: \n [X    :    Y] \n %s' %(self.__wp.T)
        self.pubMissionLog.publish(str)
        rospy.loginfo(str)
    
    def getCurrentLocation(self):    
        # Get a current location of the AUV
        X_now = 7.5 # self.__controller.getX()
        Y_now = -0.5 # self.__controller.getY()
        p = np.array([X_now, Y_now])
        
        str= 'Current location: %s' %(p)
        self.pubMissionLog.publish(str)
        rospy.loginfo(str)
        
        return p

    def trimPath(self,p):
        # If the input is a path, determine a waypoint that is closest to the path and trim down any waypoints before that waypoint.
        # Otherwise, do nothing.
        if self.__wp.size>4: # If the input consists of 3 waypoints or more (NB number of waypoints = size/2)
            # Go through every waypoint, seeking for the one that is closest to the current location ...
            ind_min = None
            rnge_min = np.Inf
            n = self.__wp.size/2
            for idx in range(n):
                rnge, _ = self.__myUti.rangeBearing(p, self.__wp[:,idx])
                if rnge < rnge_min:
                    rnge_min = rnge
                    idx_min = idx
                    
            # Update the input
            if idx_min < n-1:
                self.__wp = self.__wp[:,idx_min:]
            else:
                self.__wp = self.__wp[:,idx_min]
                
    def refine_path(self,p):
        ## modify the path in according to the current location
        # considering the first segment of the path
        p1 = self.__wp[:,0]
        p2 = self.__wp[:,1]
        # determine a closest interception point between a current location and the first segment of the path.
        t, _ = self.__myUti.interPointLine(p1,p2,p)
        
        if t<0: # Interception point is behind the first segment ...
            # insert a current location to the begining of the path.
            wp_out = np.vstack(( p , self.__wp.T )).T
        else: # Interception point will never gets beyond the first line segment since the path has been trimmed down.
            # So, use the path as it is.
            wp_out = self.__wp
            
        return wp_out
        
    def refine_segment(self,p):
        ## modify the segment in according to the current location
        # considering the first segment of the path
        p1 = self.__wp[:,0]
        p2 = self.__wp[:,1]
        # determine a closest interception point between a current location and the segment.
        t, _ = self.__myUti.interPointLine(p1,p2,p)
        if t<0: # Interception point is behind the segment ...
            # insert a current location to the begining of the segment.
            wp_out = np.vstack(( p , self.__wp.T )).T
        elif t>1: # Interception point is beyond the segment ...
            # replace a first waypoint in the segment with the current location.
            wp_out = np.vstack(( p , self.__wp[:,1] )).T
        else: # Interception point is within the segment ...
            # output the segment as it is.
            wp_out = self.__wp
            
        return wp_out
        
    def refine_waypoint(self,p):
        ## Use a current location and a point to form a segment
        wp_out = np.vstack(( p , self.__wp)).T
        
        return wp_out
        
    def execute(self,userdata):
        ######## START OPERATION ################################            
        self.displayInput()
        
        p = self.getCurrentLocation()
        self.trimPath(p)

        # Further refine the input ...
        if self.__wp.size>4: # If the refined input still consists of 3 waypoints or more (NB number of waypoints = size/2)
            wp_out = self.refine_path(p)
        elif self.__wp.size>2: # If the refined input still consists of 2 waypoints (NB number of waypoints = size/2)
            wp_out = self.refine_segment(p)
        else: # Otherwise, the refined input is now a waypoint
            wp_out = self.refine_waypoint(p)
        
        # Exit state with a corresponding outcome
        str= 'Execute reviseWaypoints State with input: \n [X    :    Y] \n %s' %(wp_out.T)
        self.pubMissionLog.publish(str)
        rospy.loginfo(str)
        
        userdata.wp_out = wp_out
        if wp_out.size==4: # If the final output consists of 2 waypoints (NB number of waypoints = size/2)
            return 'segment'
        else:
            return 'path'
