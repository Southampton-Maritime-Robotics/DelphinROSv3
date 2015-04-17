#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from std_msgs.msg import String


class GoToDepth(smach.State):
    def __init__(self, lib, depth_demand, depth_tolerance, stable_time, timeout, heading_demand, speed_demand):
            smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
            self.__controller           = lib
            self.__depth_demand         = depth_demand
            self.__tolerance            = depth_tolerance
            self.__stable_time          = stable_time
            self.__timeout              = timeout
            self.__at_depth_time        = time.time()
            self.__heading_demand       = heading_demand
            self.__speed_demand         = speed_demand
            
    def execute(self,userdata):
        
        ######## SETUP PUBLISHER FOR MISSION LOG ################
            global pub
            #Set Up Publisher for Mission Control Log
            pub = rospy.Publisher('MissionStrings', String)
        
        ######## START OPERATION ################################
            
            time_zero = time.time()
            
            str= 'Entered goToDepth State Initialise, started at time = %s' %(time_zero)
            pub.publish(str)
            
            str= 'Depth demand = %s, Heading demand = %s, Speed demand = %s'  %(self.__depth_demand, self.__heading_demand, self.__speed_demand)
            pub.publish(str)
        
            at_depth = False
            self.__at_depth_time = time.time()
            
            self.__controller.setDepth(self.__depth_demand)
            self.__controller.setHeading(self.__heading_demand)
            self.__controller.setSpeed(self.__speed_demand)
            
            ##### Main loop #####
            while (time.time()-time_zero < self.__timeout) and self.__controller.getBackSeatErrorFlag() == 0:
                
                at_depth = self.check_depth()
                
                
                if at_depth == True:
                    str= 'goToDepth succeeded at time = %s' %(time.time())
                    pub.publish(str)
                    return 'succeeded'
                
            ##### Main loop #####
            str= 'backSeatErrorFlag = %s' %(self.__controller.getBackSeatErrorFlag())
            pub.publish(str)
            
            if self.__controller.getBackSeatErrorFlag() == 1:
                str= 'goToDepth preempted at time = %s' %(time.time())    
                pub.publish(str)
                return 'preempted'
            else:
                str= 'goToDepth timed-out at time = %s' %(time.time())
                pub.publish(str)
                return 'aborted'  
            
    def check_depth(self):                
            
            if (abs(self.__controller.getDepth() - self.__depth_demand) >= self.__tolerance):
                self.__at_depth_time = time.time()
                
            if time.time()-self.__at_depth_time > self.__stable_time:
                return True
            else:
                return False
                
