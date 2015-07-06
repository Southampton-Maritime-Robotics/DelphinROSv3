#!/usr/bin/env python

'''
######################################
# description
A state for Zig-Zag manoeuvre (also known as Z manoeuvre).

Given two reference points, the AUV will start from one point then accelerate in a straight line with a certain prop demand to the other point for a few seconds. It will then execute actuator command (either rudders, thrusters or their combination) and perform a five cycle zig-zag path.

Procedure:
    go to reference point
    point to another reference point
    descent to the desired depth
    move forward with a fixed propeller setpoint for a few seconds
    execute the actuator command to zig-zag manoeuvre
    each combination of command will be held until the AUV has completed e.g. five cycles
    stop and ascend to the surface

#Modifications
21/May/2015: make the AUV capable of performing an action at depth
28/May/2015: included the backSeatErrorFlag as a terminating criteria
29/May/2015: keep the AUV heading during descent

@return: preemped: if the backSeatErrorFlag has been raised
@return: succeeded: if the AUV execute all the combination of actuator demands (propeller, thruster, rudder) in a given lists

'''

import rospy
import numpy
import smach
import smach_ros
import time
from pylab import *
from math import *
from std_msgs.msg import String
from delphin2_mission.utilities     import uti

class manoeuvreZigZag(smach.State):
    def __init__(self, lib, myUti, wp, uGain, uMax, errHeadingTol, wp_R, timeDemandHold, timeDelay, depthDemand, depthTol, depthDemandMin, controlRate):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller = lib
        self.__uti = myUti
        self.__wp = wp
        self.__uGain = uGain
        self.__uMax = uMax
        self.__errHeadingTol = errHeadingTol
        self.__wp_R = wp_R
        self.__timeDemandHold = timeDemandHold # demand will be hold for this many second
        self.__timeDelay = timeDelay
        self.__depthDemand = depthDemand # [m].
        self.__depthTol = depthTol # [m]. It is account as the AUV get to the depth if the depth error is less than this.
        self.__depthDemandMin = depthDemandMin # [m] if the depthDemand is less than this, it is accounted as no depth demand specified.
        self.__listProp = [10, 16, 22] # [10,16,22] # list of propeller demand used in experiment
        self.__listThruster = [0, 800, 1500] # [0,800,1500,2200] # TODO [0,800,1500,2000] list of thruster demand used in experiment [+ve yaw CW]
        self.__listRudder = [0, 10, 20] # [0,10,20,30] # TODO [0,10,20,30]] list of rudder angle used in experiment [+ve yaw CW] 
        self.__timeAccelerate = 5 # [sec] delay to let the AUV accelerate
        self.__cycleZigZagDemand = 5 # number of cycle that AUV have to execute in a zig-zag manoeuvre
        self.__amplitude = 20 # [deg]
        self.__controlRate = controlRate
    
    def execute(self, userdata):
        
        #Set Up Publisher for Mission Control Log
        pubMissionLog = rospy.Publisher('MissionStrings', String)

        ### Utility Object ###
        myUti = uti()
        
        ####################################################################
        ### Perform actions ################################################
        ####################################################################

        wpRang,_ = self.__uti.rangeBearing([self.__wp[0][0], self.__wp[1][0]],[self.__wp[0][1], self.__wp[1][1]]) # determine a range between waypoints as a reference
        wpIndex = 0
        
        if self.__controlRate>0:
            r = rospy.Rate(self.__controlRate)

        for demandProp in self.__listProp:
            if rospy.is_shutdown() or self.__controller.getBackSeatErrorFlag() == 1:
                break
            for demandThruster in self.__listThruster:
                if rospy.is_shutdown() or self.__controller.getBackSeatErrorFlag() == 1:
                    break
                for demandRudder in self.__listRudder:
                    if rospy.is_shutdown() or self.__controller.getBackSeatErrorFlag() == 1:
                        break

                    timeStart = time.time()
                    if demandRudder!=0 or demandThruster!=0: # if both actuator commands are zero, skip this combination

                        # go to the waypoint
                        str = 'go to start point: %s' %self.__wp[:,wpIndex]
                        rospy.loginfo(str)
                        pubMissionLog.publish(str)
                        while not rospy.is_shutdown() and self.__controller.getBackSeatErrorFlag() == 0:
                                                
                            X = self.__controller.getX()
                            Y = self.__controller.getY()
                            heading = self.__controller.getHeading()
                            rang, bear = self.__uti.rangeBearing([X,Y], [self.__wp[0][wpIndex], self.__wp[1][wpIndex]])
                            if rang >= self.__wp_R:
                                errHeading = self.__uti.computeHeadingError(bear,heading)
                                u = self.__uti.surgeVelFromHeadingError(self.__uMax,self.__uGain,errHeading)
                                self.__controller.setRearProp(round(u*22.)) # turn speedDemand into propeller demand and send
                                self.__controller.setHeading(bear)                        
                            else:
                                # when the AUV get to the start point, move onto the next step
                                self.__controller.setRearProp(0)
                                self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
                                self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
                                break
                                
                            if self.__controlRate>0:
                                r.sleep()

                        # point toward anoter waypoint
                        timeStart = time.time()
                        str = 'point to the target: %s' %self.__wp[:,1-wpIndex]
                        rospy.loginfo(str)
                        pubMissionLog.publish(str)
                        flag = 1
                        while not rospy.is_shutdown() and self.__controller.getBackSeatErrorFlag() == 0:
                        
                            X = self.__controller.getX()
                            Y = self.__controller.getY()
                            heading = self.__controller.getHeading()
                            rang, bear = self.__uti.rangeBearing([X,Y], [self.__wp[0][1-wpIndex], self.__wp[1][1-wpIndex]])
                            errHeading = self.__uti.computeHeadingError( bear, heading )
                            if abs(errHeading)>self.__errHeadingTol:
                                timeStart = time.time()
                            if time.time()-timeStart <= self.__timeDelay:
                                self.__controller.setHeading(bear)
                                if flag:
                                    str = 'let the heading becomes stady for timeDelay = %s sec' %self.__timeDelay
                                    rospy.loginfo(str)
                                    pubMissionLog.publish(str)
                                    flag = 0
                            else:
                                # when the heading is steady, move onto the next step
                                self.__controller.setRearProp(0)
                                self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
                                self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
                                break
                            if self.__controlRate>0:
                                r.sleep()
                                
                        # bring the AUV to depth if the depth demand is specified
                        if self.__depthDemand>=self.__depthDemandMin:
                            str = 'descend to a depth of %sm' % self.__depthDemand
                            rospy.loginfo(str)
                            pubMissionLog.publish(str)
                            timeStart = time.time()
                            flag = 1
                            while not rospy.is_shutdown() and self.__controller.getBackSeatErrorFlag() == 0:
                            
                                if abs(self.__controller.getDepth()-self.__depthDemand)>self.__depthTol:
                                    timeStart = time.time() # reset the reference time
                                if time.time()-timeStart <= self.__timeDelay:
                                    self.__controller.setHeading(bear)
                                    self.__controller.setDepth(self.__depthDemand)
                                    if flag:
                                        str = 'let the depth becomes stady for timeDelay = %s sec' %self.__timeDelay
                                        rospy.loginfo(str)
                                        pubMissionLog.publish(str)
                                        flag = 0
                                else:
                                    # if the AUV get to the depth and stay there long enough, move onto the next step
                                    str = 'steady at desired depth'
                                    rospy.loginfo(str)
                                    pubMissionLog.publish(str)
                                    break
                                if self.__controlRate>0:
                                    r.sleep()

                        # get the AUV accelerated then apply demand
                        str = "get the AUV accelerate with prop = %s" %(demandProp)
                        rospy.loginfo(str)
                        pubMissionLog.publish(str)
                        timeStart = time.time()
                        while not rospy.is_shutdown() and self.__controller.getBackSeatErrorFlag() == 0 and time.time()-timeStart < self.__timeAccelerate:
                        
                            self.__controller.setRearProp(demandProp)
                            self.__controller.setHeading(bear) # hold at the previous demand
                            self.__controller.setDepth(self.__depthDemand)
                            if self.__controlRate>0:
                                r.sleep()
                                
                        # execute a zig-zag manoeuvre
                        str = 'execute a zig-zag manoeuvre with a demand [prop = %s, thruster = %s, rudder = %s]' %(demandProp, demandThruster, demandRudder)
                        rospy.loginfo(str)
                        pubMissionLog.publish(str)
                        
                        cycleZigZag = 0
                        direction = 1 # 1: yaw right, -1: yaw left
                        headingRef = self.__controller.getHeading()

                        while not rospy.is_shutdown() and self.__controller.getBackSeatErrorFlag() == 0:

                            headingThreshold = mod(headingRef+self.__amplitude*direction, 360)
                            error = myUti.computeHeadingError(headingThreshold,self.__controller.getHeading())
                            
                            if error*direction <= 0:
                                
                                str = 'headingRef: %s, direction: %s, cycle: %s' %(headingRef,direction,cycleZigZag)
                                rospy.loginfo(str)
                                                                
                                direction = -direction # switch direction
                                if direction == 1:
                                    cycleZigZag += 1 #
                            
                            if cycleZigZag <= self.__cycleZigZagDemand:
                                self.__controller.setRearProp(demandProp)
                                self.__controller.setControlSurfaceAngle(direction*demandRudder,0,direction*demandRudder,0) # (VerUp,HorRight,VerDown,HorLeft)
                                self.__controller.setArduinoThrusterHorizontal(direction*demandThruster,-direction*demandThruster) # (FrontHor,RearHor)
                                if self.__depthDemand>=self.__depthDemandMin:
                                    self.__controller.setDepth(self.__depthDemand)
                            else:
                                # if the AUV do the action long enough, move onto the next combination of setpoint
                                self.__controller.setRearProp(0)
                                self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
                                self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
                                if self.__depthDemand>=self.__depthDemandMin:
                                    self.__controller.setDepth(0) # by defult, the depth controller will turn off on its own after 1sec of not reciving new demand
                                break
                            if self.__controlRate>0:
                                r.sleep()
                                        
                        # vehicle will stop for this many second as to let the AUV ascend to the surface
                        if self.__depthDemand>=self.__depthDemandMin:
                            self.__controller.setDepth(0) # by defult, the depth controller will turn off on its own after 1sec of not reciving new demand
                            str = 'rest for timeDelay = %s sec to let the AUV assend' %self.__timeDelay
                            rospy.loginfo(str)
                            pubMissionLog.publish(str)
                            time.sleep(self.__timeDelay)
                                
                        # switch the role of two reference waypoints
                        wpIndex = 1-wpIndex
        
        if self.__controller.getBackSeatErrorFlag() == 1:
            str= 'manoeuvreZigZag preempted at time = %s' %(time.time())    
            rospy.loginfo(str)
            pubMissionLog.publish(str)
            return 'preempted'
        else:
            str= 'manoeuvreZigZag succeed at time = %s' %(time.time())    
            rospy.loginfo(str)
            pubMissionLog.publish(str)
            return 'succeeded'
