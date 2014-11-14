#!/usr/bin/env python

import roslib; roslib.load_manifest('delphin2_mission')
import rospy
import smach
import smach_ros
import time
import matplotlib.pyplot as plt;

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

class ImportWaypoints(smach.State):
	def __init__(self, lib, pathAndFile1,pathAndFile2,plot):
		smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
                self.__controller = lib
  		self.__pathAndFile1 = pathAndFile1
  		self.__pathAndFile2 = pathAndFile2
  		self.__plot = plot
	def execute(self,userdata):
		global longitude
		global latitude
				
		(longitude, latitude, Load1)=self.__controller.loadWaypoints(self.__pathAndFile1)
		(longitudeBoundary, latitudeBoundary, Load2)=self.__controller.loadWaypoints(self.__pathAndFile2)
		if Load1 ==1 and Load2==1:
			if self.__plot==True:
				pl=plt.plot(longitude,latitude)
				pl=plt.plot(longitudeBoundary,latitudeBoundary,'r')
				plt.show()
                	return 'succeeded'
		else:
			return 'aborted'	

               
                
