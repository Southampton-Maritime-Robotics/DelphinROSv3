#!/usr/bin/env python

'''
A state for horizontal plane path following algorithm based on line-of-sight technique.

NB, this node should be called after state_reviseWaypoints.py.

execute:
@return: preempted: if the backSeatErrorFlag has been raised
@return: succeeded: when the AUV has reached to the final target
@return: aborted: timeout is raised before reaching the final target

#TODO
-should also consider the side-slip angle when determine heading error

# Note:
- locationWaitTimeout is used to double check if the current location is available

'''

import rospy
import numpy as np
import smach
import smach_ros
import time
import math
from   std_msgs.msg import String

class pathFollowingLOS(smach.State):
    def __init__(self, lib, myUti, timeout):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'], 
                                   input_keys=['wp_in'])
        
        self.__controller           = lib
        self.__uti                  = myUti
        self.__timeout              = timeout
        self.__controlRate          = 5         # [Hz]
        try:
            self.__L_los    = rospy.get_param('LOS_distance')
            self.__uMax     = rospy.get_param('max-speed')
            self.__wp_R     = rospy.get_param('radius_of_acceptance')
        except:
            self.__L_los    = 10    # line of sight distance [m]
            self.__uMax     = 1     # maximum speed [m/s]
            self.__wp_R     = 3     # circle of acceptance [m]
        #Set Up Publisher for Mission Control Log
        self.pubMissionLog = rospy.Publisher('MissionStrings', String)
        
    def getCurrentLocation(self):    
        # Get a current location of the AUV
        X_now = self.__controller.getX()
        Y_now = self.__controller.getY()
        p = np.array([X_now, Y_now])
        
        return p
                
    def check_waypoint_switching(self, p, target, idx_wp, verboseFlag):
        # Determine a range between current location and the targeting waypoint
        rnge, _ = self.__uti.rangeBearing(p, target)
        # If the AUV is withing a radius of acceptance of the targeting waypoint, move the index to the next waypoint.
        if rnge < self.__wp_R:
            idx_wp += 1
            verboseFlag = 1
        # Otherwise, return idx_wp and verboseFlag as they are.
                        
        return idx_wp, verboseFlag
        
    def compute_LOS_parameters(self, p1, p2, p):
        ## compute line-of-sight parameters
        # determine an interception point between current location and the path segment.
        t, p_inter = self.__uti.interPointLine(p1, p2, p)
        
        # determine a cross track error
        vecCross = p_inter-p
        ye = math.sqrt( vecCross[0]**2 + vecCross[1]**2 )
        # determine a range to the targeting waypoint
        rnge, _ = self.__uti.rangeBearing(p, p2)
                    
        # determine line-of-sight position
        if ye>=self.__L_los: # If the path is not within a ling of sight, ...
            los_p = p_inter # move toward the interception point first.
        elif rnge<self.__L_los: # If the targeting waypoint is within a line of sight, ...
            los_p = p2 # directly move toward the target
        else: # Otherwise, determine a line-of-sight position.
            # compute lookahead distance
            xe = math.sqrt( self.__L_los**2 - ye**2 )
            vecAlong = p2-p1
            vecAlongLen = math.sqrt( vecAlong[0]**2 + vecAlong[1]**2 )
            
            if t>1: # If the AUV is already beyond the targeting waypoint, ...
                los_p = p_inter - xe*vecAlong/vecAlongLen
            else:
                los_p = p_inter + xe*vecAlong/vecAlongLen
        
        # determine line-of-sight angle
        _, los_a = self.__uti.rangeBearing(p, los_p)
        
        return los_p, los_a
        
    def execute(self, userdata):
        
        ####################################################################
        ### Perform actions ################################################
        ####################################################################
        # Set Up Loop Timing Control
        r = rospy.Rate(self.__controlRate)
        
        wp = userdata.wp_in     # get waypoints from userdata input
        timeStart = time.time() # reference time for state timeout criteria
        
        ##### Main loop #####
        n = wp.size/2   # number of waypoints
        idx_wp = 1      # waypoint index (index for the first point in the list is denoted as 0)
        verboseFlag = 1 # a flag used for displaying a targeting waypoint
        
        while not rospy.is_shutdown() and self.__controller.getBackSeatErrorFlag() == 0 and time.time()-timeStart < self.__timeout:
            ## Determine AUV state
            heading = self.__controller.getHeading()
            p = self.getCurrentLocation()
            
            # Say out loud what is the current targeting waypoint - do this only once for each target.
            if verboseFlag:
                str = 'targeting waypoint: %s' %wp[:,idx_wp]
                rospy.loginfo(str)
                self.pubMissionLog.publish(str)
                verboseFlag = 0
                
            ## Update the index for the targeting waypoint
            idx_wp, verboseFlag = self.check_waypoint_switching(p, wp[:,idx_wp], idx_wp, verboseFlag)

            ## Check if the final target has been reached
            if idx_wp+1>n: # If arrive to the last waypoint, terminate the mission with outcome succeeded
                str = 'pathFollowingLOS succeeded \n current location: %s' %(p)
                rospy.loginfo(str)
                self.pubMissionLog.publish(str)
                return 'succeeded'
            else: # Otherwise, compute line-of-sight parameters and publish a heading and propeller demand.
                _, los_a = self.compute_LOS_parameters(wp[:,idx_wp-1], wp[:,idx_wp], p)
                
                ## publish heading demand
                self.__controller.setHeading(los_a)
                ## publish heading demand
                self.__controller.setRearProp(22)

            r.sleep()
                        
        if self.__controller.getBackSeatErrorFlag() == 1:
            str= 'pathFollowingLOS preempted'    
            rospy.loginfo(str)
            self.pubMissionLog.publish(str)
            return 'preempted'
        else:
            str= 'time-out: pathFollowingLOS aborted'
            rospy.loginfo(str)
            self.pubMissionLog.publish(str)
            return 'aborted'
