#!/usr/bin/env python

import roslib; roslib.load_manifest('delphin2_mission')
import rospy
import smach
import smach_ros
import time
from std_msgs.msg import String

# Initialise class
# @extends: smach.State
#
# __init__ - initialises an Initialise object
# @param: lib - instance of library_highlevel.py
# @param: timeout - time state will wait for sensors/actuators to come online
# execute - waits until all sensors/actuators are online and checks motor voltage. 
# @return: aborted: if timeout is reached
# @return: preempted: if battery voltage is less than 20,000mV
# @return: succeeded: if all systems came online within the time and battery is above 20,000mV

class Initialise(smach.State):
	def __init__(self, lib, timeout):
		smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
                self.__controller = lib
                self.__timeout = timeout
  		
	def execute(self,userdata):
                global pub
                #Set Up Publisher for Mission Control Log
                pub = rospy.Publisher('MissionStrings', String)
		time_zero = time.time()
                all_online = False
                str= 'Entered State Initialise'
                pub.publish(str)
                
                while (time.time()-time_zero < self.__timeout) and not(all_online):
                   all_online = (self.__controller.getThrusterStatus() 
                        and self.__controller.getTailStatus()   
                        and self.__controller.getAltimeterStatus()
                        and self.__controller.getGPSStatus()
                        and self.__controller.getCompassStatus())
####                   print 'thruster status = ', self.__controller.getThrusterStatus()
####                   print 'tail status = ', self.__controller.getTailStatus()
####                   print 'alt status = ', self.__controller.getAltimeterStatus()
####                   print 'gps status = ', self.__controller.getGPSStatus()
####                   print 'compass status = ', self.__controller.getCompassStatus(), '\n'
                   time.sleep(0.5)

                str= 'thruster status = %r' % self.__controller.getThrusterStatus()
                pub.publish(str)
                str= 'tail status = %r' % self.__controller.getTailStatus()
                pub.publish(str)
                str= 'alt status = %r' % self.__controller.getAltimeterStatus()
                pub.publish(str)
                str= 'gps status = %r' % self.__controller.getGPSStatus()
                pub.publish(str)
                str='compass status = %r' %self.__controller.getCompassStatus()
                pub.publish(str)
                str= 'all online =  = %r' %all_online
                pub.publish(str)
                str='time elapsed = %s s' %(time.time()-time_zero)
                pub.publish(str)

                print 'thruster status = ', self.__controller.getThrusterStatus()
                print 'tail status = ', self.__controller.getTailStatus()
                print 'alt status = ', self.__controller.getAltimeterStatus()
                print 'gps status = ', self.__controller.getGPSStatus()
                print 'compass status = ', self.__controller.getCompassStatus()
                print 'all online = ', all_online
                print 'time elapsed = ', time.time()-time_zero
                #if timeout...                
                if all_online == False:    
                    str="One or more critical systems have not come online within the timeout (%ss)" %self.__timeout 
                    rospy.logerr(str)
                    pub.publish(str)          
                    str='Initialise State Aborted' 
                    pub.publish(str)                      
                    return 'aborted'
                
                time.sleep(0.1) #give motor control board time to return a voltage measurement
                str = "Motor voltage: %smV" %(self.__controller.getVoltage())
                rospy.loginfo(str)
                pub.publish(str) 
                
                voltage=self.__controller.getVoltage()
                if voltage < 20*1000:
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
