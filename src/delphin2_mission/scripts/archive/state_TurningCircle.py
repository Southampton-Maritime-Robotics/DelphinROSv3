#!/usr/bin/env python

import roslib; roslib.load_manifest('delphin2_mission')
import rospy
import smach
import smach_ros
import time
from std_msgs.msg import String


class TurningCircle(smach.State):
    def __init__(self, lib, prop_speed, time1stPhase, rudderAngle, time2ndPhase):
            smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
            self.__controller = lib
            self.__prop_speed = prop_speed
            self.__time1stPhase = time1stPhase
            self.__rudderAngle= rudderAngle
            self.__time2ndPhase = time2ndPhase

    def execute(self,userdata):
            global pub
            global error
            #Set Up Publisher for Mission Control Log
            pub = rospy.Publisher('MissionStrings', String)
            time_zero=time.time()
            str='Entered TurningCircle State at %s' %time_zero
            pub.publish(str)
            rospy.loginfo(str)
            str='Rudder Angle =%.3f deg' %(self.__rudderAngle) 
            pub.publish(str)
            rospy.loginfo(str)
            str='PropDemand =%.3f' %(self.__prop_speed) 
            pub.publish(str)
            rospy.loginfo(str)            
            
            #######First Phase
            self.__controller.switchHorizontalThrusters(0)
            self.__controller.switchHeadingOnOff(0)   
            self.__controller.switchDepthOnOff(0)
            
            self.__controller.switchHeadingOnOff(0)
            self.__controller.setControlSurfaceAngle(0,0,0,0)  #top: %s, sb: %s, bottom: %s, p: %s deg"
            time_zero = time.time()
            self.__controller.switchHorizontalThrusters(0)
            self.__controller.setRearProp(self.__prop_speed)            
            while (time.time()-time_zero < self.__time1stPhase) and self.__controller.getBackSeatErrorFlag() == 0:
            
                pass
                
                
            #######Second Phase
            self.__controller.setControlSurfaceAngle(self.__rudderAngle,0,self.__rudderAngle,0)  

            time_zero = time.time()
         
            while (time.time()-time_zero < self.__time2ndPhase) and self.__controller.getBackSeatErrorFlag() == 0:
            
                pass                


            if self.__controller.getBackSeatErrorFlag() == 1:
                return 'preempted'
            else:
                return 'succeeded' 
                
            
  
                
