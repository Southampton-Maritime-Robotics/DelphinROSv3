#!/usr/bin/env python

import roslib; roslib.load_manifest('delphin2_mission')
import rospy
import numpy
import smach
import smach_ros
import time

class terminalZ(smach.State):
    def __init__(self, lib, thrust, depthlimit, timeout):
            smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
            self.__controller = lib
            self.__thrust = thrust
            self.__depthlimit = depthlimit
            self.__timeout = timeout
            
    def execute(self,userdata):
            
            self.__controller.switchDepthOnOff(0)
            self.__controller.switchVerticalThrusters(1)
            time_zero = time.time()
            
            ##### Main loop #####
            while (time.time() - time_zero < self.__timeout) and self.__controller.getBackSeatErrorFlag() == 0:
                
                bouyancy = 6.79
                
                #Pitch controller
                pitch_error = 0 - self.__controller.getPitch()
                pitch_moment_error = 0.52*numpy.sin(numpy.deg2rad(pitch_error))
                
                T0 = 0.91*(self.__thrust + bouyancy/2) - (pitch_moment_error/(2*0.55))
                T1 = 1.11*(self.__thrust + bouyancy/2) + (pitch_moment_error/(2*0.49))
                
                T0setpoint = int(numpy.sign(T0)*(60*(numpy.abs(T0)/(1000*0.46*0.07**4))**0.5))
                T1setpoint = int(numpy.sign(T1)*(60*(numpy.abs(T1)/(1000*0.46*0.07**4))**0.5))

                
                self.__controller.setTSLVertical(T0setpoint, T1setpoint)
              
                if self.__controller.getDepth() > self.__depthlimit:
                    return 'succeeded'
                
            ##### Main loop #####

            if self.__controller.getBackSeatErrorFlag() == 1:
                return 'preempted'
            else:
                return 'aborted'  
            
