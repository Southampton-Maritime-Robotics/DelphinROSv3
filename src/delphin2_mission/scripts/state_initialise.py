#!/usr/bin/env python

'''
A state to check if 
-all the required actuators and sensors come online within a certain time span.
-battery charged state is good

Initialise class
@extends: smach.State

__init__ - initialises an Initialise object
@param: lib - instance of library_highlevel.py
@param: timeout - time state will wait for sensors/actuators to come online
execute - waits until all sensors/actuators are online and checks motor voltage. 
@return: aborted: if timeout is reached
@return: preempted: if battery voltage is less than 20,000mV
@return: succeeded: if all systems came online within the time and battery is above 20,000mV

'''

import rospy
import smach
import smach_ros
import time
from std_msgs.msg import String

class Initialise(smach.State):
    def __init__(self, lib, timeout):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller = lib
        self.__timeout = timeout
  		
    def execute(self,userdata):

        #parameters to utilize watchdog
        try: 
            voltage_min = rospy.get_param('min-motor-voltage')
        except:
            voltage_min = 19000 # [mV]
        	
        #Set Up Publisher for Mission Control Log
        pub = rospy.Publisher('MissionStrings', String)
                        
        time_zero = time.time()
        all_online = False
        text= 'Entered State Initialise'
        pub.publish(text)

        r = rospy.Rate(2) # [Hz] for controlling the loop timing
        while (time.time()-time_zero < self.__timeout) and not(all_online) and not rospy.is_shutdown():
           all_online = (self.__controller.getThrusterStatus() 
                and self.__controller.getTailStatus()   
                and True # self.__controller.getAltimeterStatus() # FIXME: remove "True" and uncomment me when loading the altimeter
                and self.__controller.getGPSStatus()
                and self.__controller.getDepthTransducerStatus()
                and self.__controller.getXsensStatus()
                and self.__controller.getHeadingCtrlStatus()
                and self.__controller.getDepthCtrlStatus()
                and self.__controller.getDeadreckonerStatus()
                and self.__controller.getLoggerStatus()
                and self.__controller.getBackSeatDriverStatus()
                and self.__controller.getEnergyMonitorStatus())
           r.sleep()

        text = "\n ############################################################################### \n ############################################### CRITICAL NODE STATUS ########## \n ###############################################################################"
        rospy.loginfo(text)
        pub.publish(text)
        text = 'thruster status = %r' % self.__controller.getThrusterStatus()
        pub.publish(text)
        rospy.loginfo(text)
        text= 'tail status = %r' % self.__controller.getTailStatus()
        pub.publish(text)
        rospy.loginfo(text)
        #text= 'alt status = unloaded!!!!!!!!!!' # FIXME: load the altimeter when needed
        text= 'alt status = %r' % self.__controller.getAltimeterStatus()
        pub.publish(text)
        rospy.loginfo(text)
        text= 'gps status = %r' % self.__controller.getGPSStatus()
        pub.publish(text)
        rospy.loginfo(text)
        text='depthTranducer status = %r' %self.__controller.getDepthTransducerStatus()
        pub.publish(text)
        rospy.loginfo(text)
        text='xsens status = %r' %self.__controller.getXsensStatus()
        pub.publish(text)
        rospy.loginfo(text)
        text = "###########################################"
        rospy.loginfo(text)
        pub.publish(text)
        text='heading ctrl status = %r' %self.__controller.getHeadingCtrlStatus()
        pub.publish(text)
        rospy.loginfo(text)
        text='depth ctrl status = %r' %self.__controller.getDepthCtrlStatus()
        pub.publish(text)
        rospy.loginfo(text)
        text='deadreckoner status = %r' %self.__controller.getDeadreckonerStatus()
        pub.publish(text)
        rospy.loginfo(text)
        text='logger status = %r' %self.__controller.getLoggerStatus()
        pub.publish(text)
        rospy.loginfo(text)
        text='backSeatDriver status = %r' %self.__controller.getBackSeatDriverStatus()
        pub.publish(text)
        rospy.loginfo(text)
        text='energyMonitor status = %r' %self.__controller.getEnergyMonitorStatus()
        pub.publish(text)
        rospy.loginfo(text)
        text = "###########################################"
        rospy.loginfo(text)
        pub.publish(text)
        
        #if timeout...                
        if all_online == False:    
            text="One or more critical systems have not come online within the timeout (%ss)" %self.__timeout 
            rospy.logerr(text)
            pub.publish(text)          
            text='Initialise State Aborted' 
            pub.publish(text)                      
            return 'aborted'

        voltage = self.__controller.getVoltage()
        text = "Battery voltage: %smV" %(voltage)
        rospy.loginfo(text)
        pub.publish(text)
        text='time elapsed = %s s' %(time.time()-time_zero)
        pub.publish(text)
        rospy.loginfo(text)
        text = "\n ################################################################################# \n #################################################################################"
        rospy.loginfo(text)
        pub.publish(text)
        
        if voltage < voltage_min:
            "Initial battery voltage, %smV < 20,000mV" %voltage
            rospy.logerr(text)  
            pub.publish(text)       
            text='Initialise State Preempted' 
            pub.publish(text)                             
            return 'preempted'
        else:
            text="All critical systems have come online within the timeout (%ss)" %self.__timeout
            rospy.loginfo(text) 
            text='Initialise State Succeded' 
            pub.publish(text)  
            return 'succeeded'
