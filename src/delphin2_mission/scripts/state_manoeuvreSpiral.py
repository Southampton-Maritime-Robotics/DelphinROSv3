#!/usr/bin/env python

'''
######################################
# description
A state for spiral manoeuvre (also known as Dieudonne manoeuvre).

Given two reference points, the AUV will move in a straight line from one point with a heading bias to the other point. When it is travel far enough and the surge speed is steady, it will execute actuator command (either rudders, thrusters or their combination) and hold until the AUV have performed two circle turns (720 degree).

The sequence of actuator command will be from a hardest turn to lightest turn.
Ideally, the command has to be hardest turn on one side, then slowly decrease pass through the neutral point and continue until it reach the hardest turn on the other side (just google Dieudonne manoeuvre and you will see ^ ^).
Hoever, the space of the Eastleigh is limited and can perform turning on only one side at time.

Procedure:
    go to reference point
    head to another reference point with a heading bias
    descend to a desired depth
    move forward with a fixed propeller setpoint
    when travel far enough, execute the thruster command to start turning
    at this one particular thruster command, vary rudder demand according to a given list 
    each combination of command will be held until the AUV has completed two turns
    stop and ascend to the surface
    
######################################
#TODO
-make the AUV performs action at depth
-include a criterion to let the AUV stop when it have performed 720 degree turn

@return: succeeded: if the AUV execute all the combination of actuator demands (propeller, thruster, rudder) in a given lists

#Modifications

'''

import rospy
import numpy
import smach
import smach_ros
import time
from pylab import *
from math import *
from std_msgs.msg import String

class manoeuvreSpiral(smach.State):
    def __init__(self, lib, myUti, wp, uGain, uMax, errHeadingTol, wp_R, timeDemandHold, timeDelay, depthDemand, depthTol, depthDemandMin, turningAngle):
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
        self.__turningAngle = turningAngle # [deg] the vehicle has to turn this many degree before move to the next step
        self.__listProp = [16] # [10,16,22] # list of propeller demand used in experiment
        self.__listThruster = [0, 800, 1500] # [0,800,1500,2200] # TODO [0,800,1500,2000] list of thruster demand used in experiment [+ve yaw CW]
        self.__listRudder = [0, 10, 20] # [0,10,20,30] # TODO [0,10,20,30]] list of rudder angle used in experiment [+ve yaw CW]
        self.__direction = -1 # 1:cw, -1:ccw 
        self.__headingBias = -self.__direction*25 # TODO [deg, +ve CW] point away from the target just so the vehicle has enough space for turning

    def execute(self, userdata):
        
        ####################################################################
        ### Perform actions ################################################
        ####################################################################

        wpRang,_ = self.__uti.rangeBearing([self.__wp[0][0], self.__wp[1][0]],[self.__wp[0][1], self.__wp[1][1]]) # determine a range between waypoints as a reference
        wpIndex = 0
        
        #Set Up Publisher for Mission Control Log
        pubMissionLog = rospy.Publisher('MissionStrings', String)

        for demandProp in self.__listProp:
            
            if rospy.is_shutdown():
                break
                
            for demandThruster in self.__listThruster:
            
                if rospy.is_shutdown():
                    break
                
                # go to the waypoint
                str = 'go to start point'
                rospy.loginfo(str)
                pubMissionLog.publish(str)
                while not rospy.is_shutdown():
                
                    if self.__controller.getBackSeatErrorFlag() == 1:
                        str= 'manoeuvreSpiral preempted at time = %s' %(time.time())    
                        rospy.loginfo(str)
                        pubMissionLog.publish(str)
                        return 'preempted'
                    
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
                        time.sleep(self.__timeDelay) # vehicle will stop for this many second as to let its motion decay
                        break

                # point toward anoter waypoint with a bias of headingBias
                timeStart = time.time()
                str = 'point to the target'
                rospy.loginfo(str)
                pubMissionLog.publish(str)
                while not rospy.is_shutdown():
                
                    if self.__controller.getBackSeatErrorFlag() == 1:
                        str= 'manoeuvreSpiral preempted at time = %s' %(time.time())    
                        rospy.loginfo(str)
                        pubMissionLog.publish(str)
                        return 'preempted'
                
                    X = self.__controller.getX()
                    Y = self.__controller.getY()
                    heading = self.__controller.getHeading()
                    rang, bear = self.__uti.rangeBearing([X,Y], [self.__wp[0][1-wpIndex], self.__wp[1][1-wpIndex]])
                    headingDemand = mod(bear+self.__headingBias,360) # deviate from the bearing by headingBias
                    errHeading = self.__uti.computeHeadingError( headingDemand, heading )
                    if abs(errHeading)>self.__errHeadingTol:
                        timeStart = time.time()
                    if time.time()-timeStart <= self.__timeDelay:
                        self.__controller.setHeading(headingDemand)
                    else:
                        # when the heading is steady, move onto the next step
                        self.__controller.setRearProp(0)
                        self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
                        self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
                        break
                        
                # bring the AUV to depth if the depth demand is specified
                if self.__depthDemand>=self.__depthDemandMin:
                    str = 'descend to desired depth'
                    rospy.loginfo(str)
                    pubMissionLog.publish(str)
                    timeStart = time.time()
                    while not rospy.is_shutdown():
                    
                        if self.__controller.getBackSeatErrorFlag() == 1:
                            str= 'manoeuvreSpiral preempted at time = %s' %(time.time())    
                            rospy.loginfo(str)
                            pubMissionLog.publish(str)
                            return 'preempted'
                    
                        if abs(self.__controller.getDepth()-self.__depthDemand)>self.__depthTol:
                            timeRef = time.time() # reset the reference time
                        if time.time()-timeStart <= self.__timeDelay:
                            self.__controller.setDepth(self.__depthDemand)
                        else:
                            # if the AUV get to the depth and stay there long enough, move onto the next step
                            break
                
                # get the AUV accelerated then apply demand
                str = "get the AUV accelerate"
                rospy.loginfo(str)
                pubMissionLog.publish(str)
                while not rospy.is_shutdown():
                
                    if self.__controller.getBackSeatErrorFlag() == 1:
                        str= 'manoeuvreSpiral preempted at time = %s' %(time.time())    
                        rospy.loginfo(str)
                        pubMissionLog.publish(str)
                        return 'preempted'
                
                    X = self.__controller.getX()
                    Y = self.__controller.getY()
                    heading = self.__controller.getHeading()
                    rang, bear = self.__uti.rangeBearing([X,Y], [self.__wp[0][wpIndex], self.__wp[1][wpIndex]])
                    if rang <= wpRang*0.75:
                        self.__controller.setRearProp(demandProp)
                        self.__controller.setHeading(headingDemand) # hold at the previous demand
                        self.__controller.setDepth(self.__depthDemand)
                    else:
                        break
                # activate the actuator when AUV is far enough from the start location
                for demandRudder in self.__listRudder:
                    if rospy.is_shutdown():
                        break
                    timeStart = time.time()
                    if demandRudder!=0 or demandThruster!=0: # if both actuator commands are zero, skip this combination
                        str = 'demand is [prop = %s, thruster = %s, rudder = %s] = ' %(demandProp, demandThruster, demandRudder)
                        rospy.loginfo(str)
                        pubMissionLog.publish(str)
                        headingAccu = 0.
                        headingOld = self.__controller.getHeading()
                        while not rospy.is_shutdown():
                        
                            if self.__controller.getBackSeatErrorFlag() == 1:
                                str= 'manoeuvreSpiral preempted at time = %s' %(time.time())    
                                rospy.loginfo(str)
                                pubMissionLog.publish(str)
                                return 'preempted'
                        
                            # compute the accumulate heading displacement
                            headingNow = self.__controller.getHeading()
                            headingDis = self.__uti.computeHeadingError(headingOld,headingNow)
                            headingAccu = headingAccu+headingDis
                            headingOld = headingNow
                            # apply the demand
                            if abs(headingAccu)<self.__turningAngle:
                                self.__controller.setRearProp(demandProp)
                                self.__controller.setControlSurfaceAngle(self.__direction*demandRudder,0,self.__direction*demandRudder,0) # (VerUp,HorRight,VerDown,HorLeft)
                                self.__controller.setArduinoThrusterHorizontal(self.__direction*demandThruster,-self.__direction*demandThruster) # (FrontHor,RearHor)
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
                                
                # vehicle will stop for this many second as to let the AUV ascend to the surface
                if self.__depthDemand>=self.__depthDemandMin:
                    self.__controller.setDepth(0) # by defult, the depth controller will turn off on its own after 1sec of not reciving new demand
                    time.sleep(self.__timeDelay)
                        
                # switch the role of two reference waypoints
                wpIndex = 1-wpIndex
        
        return 'succeeded' # exit with a flag of 'succeeded'
