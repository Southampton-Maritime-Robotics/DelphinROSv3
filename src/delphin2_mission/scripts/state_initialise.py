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
        str= 'Entered State Initialise'
        pub.publish(str)

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

        str = "\n ############################################################################### \n ############################################### CRITICAL NODE STATUS ########## \n ###############################################################################"
        rospy.loginfo(str)
        pub.publish(str)
        str = 'thruster status = %r' % self.__controller.getThrusterStatus()
        pub.publish(str)
        rospy.loginfo(str)
        str= 'tail status = %r' % self.__controller.getTailStatus()
        pub.publish(str)
        rospy.loginfo(str)
        str= 'alt status = unloaded!!!!!!!!!!' # FIXME: load the altimeter when needed
##      str= 'alt status = %r' % self.__controller.getAltimeterStatus()
        pub.publish(str)
        rospy.loginfo(str)
        str= 'gps status = %r' % self.__controller.getGPSStatus()
        pub.publish(str)
        rospy.loginfo(str)
        str='depthTranducer status = %r' %self.__controller.getDepthTransducerStatus()
        pub.publish(str)
        rospy.loginfo(str)
        str='xsens status = %r' %self.__controller.getXsensStatus()
        pub.publish(str)
        rospy.loginfo(str)
        str = "###########################################"
        rospy.loginfo(str)
        pub.publish(str)
        str='heading ctrl status = %r' %self.__controller.getHeadingCtrlStatus()
        pub.publish(str)
        rospy.loginfo(str)
        str='depth ctrl status = %r' %self.__controller.getDepthCtrlStatus()
        pub.publish(str)
        rospy.loginfo(str)
        str='deadreckoner status = %r' %self.__controller.getDeadreckonerStatus()
        pub.publish(str)
        rospy.loginfo(str)
        str='logger status = %r' %self.__controller.getLoggerStatus()
        pub.publish(str)
        rospy.loginfo(str)
        str='backSeatDriver status = %r' %self.__controller.getBackSeatDriverStatus()
        pub.publish(str)
        rospy.loginfo(str)
        str='energyMonitor status = %r' %self.__controller.getEnergyMonitorStatus()
        pub.publish(str)
        rospy.loginfo(str)
        str = "###########################################"
        rospy.loginfo(str)
        pub.publish(str)
        
        #if timeout...                
        if all_online == False:    
            str="One or more critical systems have not come online within the timeout (%ss)" %self.__timeout 
            rospy.logerr(str)
            pub.publish(str)          
            str='Initialise State Aborted' 
            pub.publish(str)                      
            return 'aborted'

        voltage = self.__controller.getVoltage()
        str = "Battery voltage: %smV" %(voltage)
        rospy.loginfo(str)
        pub.publish(str)
        str='time elapsed = %s s' %(time.time()-time_zero)
        pub.publish(str)
        rospy.loginfo(str)
        str = "\n ################################################################################# \n #################################################################################"
        rospy.loginfo(str)
        pub.publish(str)
        
        if voltage < voltage_min:
            "Initial battery voltage, %smV < 20,000mV" %voltage
            rospy.logerr(str)  
            pub.publish(str)       
            str='Initialise State Preempted' 
            pub.publish(str)                             
            return 'preempted'
        else:
            str="All critical systems have come online within the timeout (%ss)" %self.__timeout
            rospy.loginfo(str) 
            str='Initialise State Succeded' 
            pub.publish(str)  
            return 'succeeded'
