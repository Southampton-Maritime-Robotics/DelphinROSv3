#!/usr/bin/env python

'''
A state that directly operates the actuators in a certain way for a period of time.

user needs to specify
-self.delay_action: how long the action will be held
-self.__controller.setRearProp(0)
-self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
-self.__controller.setArduinoThrusterVertical(0,0) # (FrontVer,RearVer)
-self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
-more commands are available in library_highlevel

'''

import rospy
import numpy
import smach
import smach_ros
import time
from std_msgs.msg import String

class actions(smach.State):
    def __init__(self, lib):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller = lib
        self.delay_thruster = 0 # allow the vehicle to gain a speed (value is specified in second) 
        self.delay_action = self.delay_thruster+12 # let the vehicle doing those actions for a period of time (value is specified in second)
            
    def execute(self, userdata):

        #Set Up Publisher for Mission Control Log
        pubMissionLog = rospy.Publisher('MissionStrings', String)

        ####################################################################
        ### Perform actions ################################################
        ####################################################################

        # apply a setpoint to a relevant actuator
        self.__controller.setRearProp(0)
        self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
        self.__controller.setArduinoThrusterVertical(0,0) # (FrontVer,RearVer)
        self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
        time.sleep(self.delay_thruster) # allow the vehicle to gain a speed (delay is specified in second)
        
        # let the vehicle doing those actions for a period of time
        # and shutdown the actuators once finished
        
        controlRate = 10. # [Hz]
        controlPeriod = 1./controlRate
        r = rospy.Rate(controlRate)
        
        time.sleep(5)
    
####        # check relationship between u_th and rpm
####        thList = [-250,-500,-1000,-1500,-2000]
####        for thDemand in thList:
####            timeStart = time.time()
####            while not rospy.is_shutdown() and time.time()-timeStart < self.delay_action:
####                if self.__controller.getBackSeatErrorFlag() == 1:
####                    str= 'state_actions preempted at time = %s' %(time.time())    
####                    rospy.loginfo(str)
####                    pubMissionLog.publish(str)
####                    return 'preempted'
####                else:
####                    rospy.loginfo('thruster demand is %s' % thDemand)
####                    self.__controller.setArduinoThrusterHorizontal(thDemand,0) # (FrontHor,RearHor)
####                    r.sleep()
####                    pass
####            self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
####            time.sleep(5)
            
        # check transcient input with square wave
        thList = [-500,-1000,-1500,-2000]
        for thDemand in thList:
        
            for i in range(3):
            
                timeStart = time.time()
                while not rospy.is_shutdown() and time.time()-timeStart < self.delay_action:
                    if self.__controller.getBackSeatErrorFlag() == 1:
                        str= 'state_actions preempted at time = %s' %(time.time())    
                        rospy.loginfo(str)
                        pubMissionLog.publish(str)
                        return 'preempted'
                    else:
                        rospy.loginfo('thruster demand is %s' % thDemand)
#                        self.__controller.setArduinoThrusterVertical(thDemand,0) # (FrontVer,RearVer)
#                        self.__controller.setArduinoThrusterHorizontal(thDemand,0) # (FrontHor,RearHor)
                        r.sleep()
                        pass
                        
                self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
                time.sleep(3)
        
        # stop all the actuators
        self.__controller.setRearProp(0)
        self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft)
        self.__controller.setArduinoThrusterVertical(0,0) # (FrontVer,RearVer)
        self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)
        return 'succeeded' # exit with a flag of 'succeeded'
