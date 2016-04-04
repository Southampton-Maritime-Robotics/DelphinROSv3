#!/usr/bin/env python

'''
A state for horizontal plane path following algorithm based on line-of-sight technique

Given a path, it will get the AUV follow the path starts from the current location
Given a point, it will get the AUV to the point starts from the current location

execute:
@return: preempted: if the backSeatErrorFlag has been raised
@return: succeeded: when the AUV has arrived to the destination
@return: aborted: not in use

#TODO
-should also consider the side-slip angle when determine heading error

# Note:
- locationWaitTimeout is used to double check if the current location is available

'''

import rospy
import numpy
import smach
import smach_ros
import time
from   pylab        import *
from   math         import *
from   std_msgs.msg import String

class pathFollowingLOS(smach.State):
    def __init__(self, lib, myUti, path):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller           = lib
        self.__uti                  = myUti
        self.__path                 = path      # a list of waypoints
        self.__controlRate          = 5         # [Hz]
        self.__locationWaitTimeout  = 30        # [sec] timeout to check if the GPS comes online
        
        try:
            self.__L_los    = rospy.get_param('LOS_distance')
            self.__uMax     = rospy.get_param('max-speed')
            self.__wp_R     = rospy.get_param('radius_of_acceptance')
        except:
            self.__L_los    = L_los     # line of sight distance
            self.__uMax     = uMax      # maximum speed
            self.__wp_R     = wp_R      # circle of acceptance
        
    def execute(self, userdata):
        
        #Set Up Publisher for Mission Control Log
        pubMissionLog = rospy.Publisher('MissionStrings', String)

        # Set Up Loop Timing Control
        r = rospy.Rate(self.__controlRate)

        ####################################################################
        ### Perform actions ################################################
        ####################################################################
        
        ##### Wait until GPS signal is available #####
        timeStartWait = time.time()
        while True:
            X = self.__controller.getX()
            Y = self.__controller.getY()
            if X!=0 or Y!=0:
                str = 'got the current location: [%s,%s]' %(X,Y)
                rospy.loginfo(str)
                pubMissionLog.publish(str)
                eta_0 = array([X,Y]) # used when creating the path. Bare in mind that the eta_0 is an array while eta is a list
                break # succeeded in getting the current location from gps: move on
            if time.time()-timeStartWait > self.__locationWaitTimeout:
                str = 'GPS signal will not available: mission aborted'
                rospy.loginfo(str)
                pubMissionLog.publish(str)
                return 'aborted'
            r.sleep()
                        
        wpTarget = 1
        self.__path = numpy.vstack((eta_0,self.__path.T)).T # include the current location of the AUV as the first waypoint
        _,pathLen = self.__path.shape
        
        str = 'Execute path following algorithm with a following path'
        rospy.loginfo(str)
        pubMissionLog.publish(str)
        str = 'X = %s' %self.__path[0,:]
        rospy.loginfo(str)
        pubMissionLog.publish(str)
        str = 'Y = %s' %self.__path[1,:]
        rospy.loginfo(str)
        pubMissionLog.publish(str)
        
        flag = 1
        
        ##### Main loop #####
        while not rospy.is_shutdown() and self.__controller.getBackSeatErrorFlag() == 0:
            
            X = self.__controller.getX()
            Y = self.__controller.getY()
            heading = self.__controller.getHeading()
            eta = [X,Y] # state vector denoted following Fossen's convention

            if flag:
                str = 'target waypoint: %s' %self.__path[:,wpTarget]
                rospy.loginfo(str)
                pubMissionLog.publish(str)
                flag = 0
            
            # waypoint switching criteria
            if (self.__uti.waypointSwitching(self.__path[:,wpTarget],eta,self.__wp_R)):
                if wpTarget == pathLen-1:
                    # if arrive to the last waypoint, terminate the mission
                    
                    str = 'arrived to within the circle of acceptance of the destination'
                    rospy.loginfo(str)
                    pubMissionLog.publish(str)
                    str = 'current location: %s' %eta
                    rospy.loginfo(str)
                    pubMissionLog.publish(str)
                    
                    self.__controller.setRearProp(0)
                    self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
                    self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)

                    str= 'pathFollowingLOS succeeded at time = %s' %(time.time())    
                    rospy.loginfo(str)
                    pubMissionLog.publish(str)
                    return 'succeeded'
                else:
                    # if reached the current waypoint, move onto the next line segment
                    wpTarget += 1
                    flag = 1
                    
            # compute line-of-sight parameters
            t,p_inter = self.__uti.interPointLine(self.__path[:,wpTarget-1],self.__path[:,wpTarget],eta)
            vecCross = p_inter-eta # cross track error
            ye = sqrt( vecCross[0]**2 + vecCross[1]**2 )
            if ye>=self.__L_los: # get the AUV perpendicularly moves toward the path
                los_p = p_inter
            else:
                if t==1: # track p_inter which is the ending point of the line segment
                    direction = 0. # zero lookahead distance
                elif t>1: # if the AUV is already beyond the ending point of the line segment, get it moves back to the line segment
                    direction = -1. # negative lookahead distance
                else:
                    direction = 1. # positive lookahead distance
                xe = sqrt( self.__L_los**2 - ye**2 ) # compute lookahead distance
                vecAlong = self.__path[:,wpTarget]-self.__path[:,wpTarget-1]
                vecAlongLen = sqrt( vecAlong[0]**2 + vecAlong[1]**2 )
                los_p = p_inter + direction*xe*vecAlong/vecAlongLen

            los_vec = los_p-eta
            los_a = mod(atan2(los_vec[0],los_vec[1])*180/pi,360) # TODO: should also consider the side slip angle

            # determine heading error
            errHeading = self.__uti.computeHeadingError(los_a,heading)
            u = self.__uti.surgeVelFromHeadingError(self.__uMax,errHeading)
 
            # publish heading demand
            self.__controller.setHeading(los_a)
            # turn speedDemand into propeller demand and publish
            self.__controller.setRearProp(round(u*22.))

            r.sleep()
            
        if self.__controller.getBackSeatErrorFlag() == 1:
            str= 'pathFollowingLOS preempted at time = %s' %(time.time())    
            rospy.loginfo(str)
            pubMissionLog.publish(str)
            return 'preempted'
        else: # not in use
            str= 'pathFollowingLOS timed-out at time = %s' %(time.time())
            pub.publish(str)
            rospy.loginfo(str)
            return 'aborted'
