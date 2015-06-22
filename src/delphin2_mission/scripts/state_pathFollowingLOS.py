#!/usr/bin/env python

'''
A state for horizontal plane path following algorithm based on line-of-sight technique

execute:
@return: preemped: if the backSeatErrorFlag has been raised
@return: succeeded: when the AUV has arrived to the destination

#TODO
-should also consider the side-slip angle when determine heading error

'''

import rospy
import numpy
import smach
import smach_ros
import time
from pylab import *
from math import *
from std_msgs.msg import String

class pathFollowingLOS(smach.State):
    def __init__(self, lib, myUti, path, L_los, uGain, uMax, wp_R, controlRate):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller = lib
        self.__uti = myUti
        self.__path = path
        self.__L_los = L_los
        self.__uGain = uGain
        self.__uMax = uMax
        self.__wp_R = wp_R
        self.__controlRate = controlRate
        
    def execute(self, userdata):
        
        #Set Up Publisher for Mission Control Log
        pubMissionLog = rospy.Publisher('MissionStrings', String)
        
        ####################################################################
        ### Perform actions ################################################
        ####################################################################
        
        X = self.__controller.getX()
        Y = self.__controller.getY()
        eta = [X,Y]
        
        wpTarget = 1
        self.__path = numpy.vstack((eta,self.__path.T)).T # include the current location of the AUV as the first waypoint
        _,pathLen = self.__path.shape
        
        if self.__controlRate>0:
            r = rospy.Rate(self.__controlRate)

        str = 'Execute path following algorithm with a following path'
        rospy.loginfo(str)
        pubMissionLog.publish(str)
        str = 'X = %s' %self.__path[0,:]
        rospy.loginfo(str)
        pubMissionLog.publish(str)
        str = 'Y = %s' %self.__path[1,:]
        rospy.loginfo(str)
        pubMissionLog.publish(str)
        
        while not rospy.is_shutdown() and self.__controller.getBackSeatErrorFlag() == 0:
            
            # reference time to control rate
            timeRef = time.time()
            
            X = self.__controller.getX()
            Y = self.__controller.getY()
            heading = self.__controller.getHeading()
            eta = [X,Y] # state vector denoted following Fossen's convention
            
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
                    str = 'target waypoint: %s' %self.__path[:,wpTarget]
                    rospy.loginfo(str)
                    pubMissionLog.publish(str)

            # compute line-of-sight parameters
            t,p_inter = self.__uti.interPointLine(self.__path[:,wpTarget-1],self.__path[:,wpTarget],eta)
            vecCross = p_inter-eta # cross track error
            ye = sqrt( vecCross[0]**2 + vecCross[1]**2 )
            if ye>=self.__L_los:
                los_p = p_inter
            else:
                xe = sqrt( self.__L_los**2 - ye**2 ) # compute lookahead distance
                vecAlong = self.__path[:,wpTarget]-self.__path[:,wpTarget-1]
                vecAlongLen = sqrt( vecAlong[0]**2 + vecAlong[1]**2 )
                los_p = p_inter + xe*vecAlong/vecAlongLen

            los_vec = los_p-eta
            los_a = mod(atan2(los_vec[0],los_vec[1])*180/pi,360) # TODO: should also consider the side slip angle

            # determine heading error
            errHeading = self.__uti.computeHeadingError(los_a,heading)
            u = self.__uti.surgeVelFromHeadingError(self.__uMax,self.__uGain,errHeading)
 
            # publish heading demand
            self.__controller.setHeading(los_a)
            # turn speedDemand into propeller demand and send
            self.__controller.setRearProp(round(u*22.))
            
            if self.__controlRate>0:
                r.sleep()
                
        if self.__controller.getBackSeatErrorFlag() == 1:
            str= 'pathFollowingLOS preempted at time = %s' %(time.time())    
            rospy.loginfo(str)
            pubMissionLog.publish(str)
            return 'preempted'
