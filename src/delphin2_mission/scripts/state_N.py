#!/usr/bin/env python

import roslib; roslib.load_manifest('delphin2_mission')
import rospy
import numpy
import smach
import smach_ros
import time

class N(smach.State):
    def __init__(self, lib, T2, T3, timeout):
            smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
            self.__controller = lib
            self.__T2 = T2
            self.__T3 = T3
            self.__timeout = timeout
            
    def execute(self,userdata):
            
            self.__controller.switchHeadingOnOff(0)
            self.__controller.switchHorizontalThrusters(1)
            time_zero = time.time()
            
            ##### Main loop #####
            while (time.time() - time_zero < self.__timeout) and (self.__controller.getBackSeatErrorFlag() == 0):
                
                
                T2 = self.__T2
                T3 = self.__T3
                
                T2setpoint = int(numpy.sign(T2)*(60*(numpy.abs(T2)/(1000*0.46*0.07**4))**0.5))
                T3setpoint = int(numpy.sign(T3)*(60*(numpy.abs(T3)/(1000*0.46*0.07**4))**0.5))

                self.__controller.setTSLHorizontal(T2setpoint, T3setpoint)
              
            if (time.time() - time_zero > self.__timeout):
		self.__controller.switchHorizontalThrusters(0)
                time.sleep(45)
                return 'succeeded'
                
            ##### Main loop #####

            if self.__controller.getBackSeatErrorFlag() == 1:
                return 'preempted'
            else:
                return 'aborted'  
            
