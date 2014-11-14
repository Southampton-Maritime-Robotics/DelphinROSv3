#!/usr/bin/env python

import roslib; roslib.load_manifest('delphin2_mission')
import rospy
import smach
import smach_ros
import time


class setTail(smach.State):
    def __init__(self, lib, b, c, d, e, prop, time):
            smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
            self.__controller = lib
            self.__b = b
            self.__c = c
            self.__d = d
            self.__e = e
            self.__prop = prop  
            self.__time = time            
            
    def execute(self,userdata):
            time_zero = time.time()
            a = 1
            
            #self.__controller.setRearProp(self.__prop)
           
            self.__controller.setControlSurfaceAngle(0,0,0,0)
            time.sleep(2*self.__time)
            print 'ACQUIRE!!!!!'
            time.sleep(15)
            
            self.__controller.setControlSurfaceAngle(0,20,0,20)
            time.sleep(self.__time)
            print 'ACQUIRE!!!!!'
            time.sleep(15)
            
            self.__controller.setControlSurfaceAngle(0,15,0,15)
            time.sleep(self.__time)
            print 'ACQUIRE!!!!!'
            time.sleep(15)
            
            self.__controller.setControlSurfaceAngle(0,10,0,10)
            time.sleep(self.__time)
            print 'ACQUIRE!!!!!'
            time.sleep(15)
            
            self.__controller.setControlSurfaceAngle(0,5,0,5)
            time.sleep(self.__time)
            print 'ACQUIRE!!!!!'
            time.sleep(15)
            
            self.__controller.setControlSurfaceAngle(0,0,0,0)
            time.sleep(self.__time)
            print 'ACQUIRE!!!!!'
            time.sleep(15)
            
            self.__controller.setControlSurfaceAngle(0,-5,0,-5)
            time.sleep(self.__time)
            print 'ACQUIRE!!!!!'
            time.sleep(15)
            
            self.__controller.setControlSurfaceAngle(0,-10,0,-10)
            time.sleep(self.__time)
            print 'ACQUIRE!!!!!'
            time.sleep(15)
            
            self.__controller.setControlSurfaceAngle(0,-15,0,-15)
            time.sleep(self.__time)
            print 'ACQUIRE!!!!!'
            time.sleep(15)
            
            self.__controller.setControlSurfaceAngle(0,-20,0,-20)
            time.sleep(self.__time)
            print 'ACQUIRE!!!!!'
            time.sleep(15)
            
            self.__controller.setControlSurfaceAngle(0,-25,0,-25)
            time.sleep(self.__time)
            print 'ACQUIRE!!!!!'
            time.sleep(15)
            
            self.__controller.setControlSurfaceAngle(0,0,0,0)
            time.sleep(15)

            ##### Main loop #####
            if (time.time()-time_zero > self.__time):
                return 'succeeded'
            
            ##### Main loop #####

            if self.__controller.getBackSeatErrorFlag() == 1:
                return 'preempted'
            else:
                return 'aborted'  
