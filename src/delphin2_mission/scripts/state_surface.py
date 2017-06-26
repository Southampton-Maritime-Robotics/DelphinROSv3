#!/usr/bin/env python

'''
Possibly, a state to get the AUV back to the surface and have a gps fixed.

May not functioning.

'''

import rospy
import smach
import smach_ros
import time
from std_msgs.msg import String

class Surface(smach.State):
    def __init__(self, lib, timeout):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller           = lib
        self.__timeout              = timeout
        self.__controlRate          = 1         # [Hz] gps driver works at 1Hz, no need for this state to hurry
            
    def execute(self,userdata):

        #Set Up Publisher for Mission Control Log
        pubMissionLog = rospy.Publisher('MissionStrings', String, queue_size=10)

        # Set Up Loop Timing Control
        r = rospy.rate(self.__controlRate)
        
        ######## START OPERATION ################################
            
        time_zero = time.time()
        
        text= 'Entered Surface State, started at time = %s' %(time_zero)
        pubMissionLog.publish(text)
        rospy.loginfo(text)
        time.sleep(0.1)
        
        #Set Depth Demand and Speed Demand To Zero and turn off heading control
        self.__controller.setDepth(0.0)
        time.sleep(0.1)
        self.__controller.setSpeed(0.0)
        time.sleep(0.1)
        text= 'Setting depth demand and foward speed to zero and turning heading control off' 
        pubMissionLog.publish(text)
        rospy.loginfo(text)            
        
        timer=time.time()
        
        ##### Main loop #####
        while (time.time()-time_zero < self.__timeout) and not rospy.is_shutdown():
                
            text= 'Waiting For Valid GPS FIX' 
            pubMissionLog.publish(text)
            rospy.loginfo(text)  
            
            fix=self.__controller.getGPSValidFix()
            
            if fix ==0:        #If no valid GPS fix reset timer
                timer=time.time()
                                
            if fix == 1 and time.time()-timer > 5: #Had a valid GPS fix for more than 5s
            
                X=self.__controller.getX()
                Y=self.__controller.getY()
                text = 'Surface Position is X=%sm and Y=%sm' %(X,Y)
                rospy.loginfo(text)
                pubMissionLog.publish(text)
                if self.__controller.getBackSeatErrorFlag() == 1:
                    text ='Surface State has aquired new GPS fix, but backseat driver flag is set to 1'
                    pubMissionLog.publish(text)
                    rospy.loginfo(text)
                    return 'preempted'
                else:
                    text ='Surface State has succesfully aquired a new GPS fix'
                    pubMissionLog.publish(text)
                    rospy.loginfo(text)                            
                    return 'succeeded'
                
            r.sleep()
            
        text= 'Surface State Failed to get a GPS fix within the timeout of %s s' %(self.__timeout)
        pubMissionLog.publish(text)
        rospy.loginfo(text)
        return 'aborted'
