#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from std_msgs.msg import String


class GoToAltitude(smach.State):
    def __init__(self, lib, altitude_demand, altitude_tolerance, stable_time, timeout, heading_demand, speed_demand):
            smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
            self.__controller           = lib
            self.__altitude_demand      = altitude_demand
            self.__tolerance            = altitude_tolerance
            self.__stable_time          = stable_time
            self.__timeout              = timeout
            self.__at_altitude_time     = time.time()
            self.__heading_demand       = heading_demand
            self.__speed_demand         = speed_demand
            
    def execute(self,userdata):
        
        ######## SETUP PUBLISHER FOR MISSION LOG ################
            global pub
            #Set Up Publisher for Mission Control Log
            pub = rospy.Publisher('MissionStrings', String)
        
        ######## START OPERATION ################################
            
            time_zero = time.time()
            
            str= 'Entered goTo Altitude State Initialise, started at time = %s' %(time_zero)
            pub.publish(str)
            rospy.loginfo(str)
            
            str= 'Altitude demand = %s, Heading demand = %s, Speed demand = %s'  %(self.__altitude_demand, self.__heading_demand, self.__speed_demand)
            pub.publish(str)
            rospy.loginfo(str)
            
            altitude = self.__controller.getAltitude() 

            str= 'Current Altitude =%sm' %altitude
            pub.publish(str)
            rospy.loginfo(str)            
        
            at_altitude = False
            self.__at_altitude_time = time.time()
        
            self.__controller.setHeading(self.__heading_demand)
            self.__controller.setSpeed(self.__speed_demand)
            
            ##### Main loop #####
            while (time.time()-time_zero < self.__timeout) and self.__controller.getBackSeatErrorFlag() == 0:
                
                #get current depth and altitude
                depth= self.__controller.getDepth()
                altitude = self.__controller.getAltitude() 
                
                #determine new depth demand
                if altitude> 0.5:                    
                    alt_error       = altitude - self.__altitude_demand
                    depth_demand    = depth + alt_error
                else:
                    depth_demand=depth-0.1
                
                self.__controller.setDepth(depth_demand)
                #str= 'Altitude demand = %s, Altitude =%s, Depth Demand=%s, Depth = %s'  %(self.__altitude_demand, altitude, depth_demand,depth)
                #pub.publish(str)
                #rospy.loginfo(str) 
                
                at_altitude = self.check_altitude()
                
                
                if at_altitude == True:
                    str= 'goToAltitude succeeded at time = %s' %(time.time())
                    pub.publish(str)
                    return 'succeeded'
                
                time.sleep(0.05)
                
            ##### Main loop #####
            str= 'backSeatErrorFlag = %s' %(self.__controller.getBackSeatErrorFlag())
            pub.publish(str)
            
            if self.__controller.getBackSeatErrorFlag() == 1:
                str= 'goToAltitude preempted at time = %s' %(time.time())    
                pub.publish(str)
                return 'preempted'
            else:
                str= 'goToAltitude timed-out at time = %s' %(time.time())
                pub.publish(str)
                return 'aborted'  
            
    def check_altitude(self):                
            
            if (abs(self.__controller.getAltitude() - self.__altitude_demand) >= self.__tolerance):
                self.__at_altitude_time = time.time()
                
            if time.time()-self.__at_altitude_time > self.__stable_time:
                return True
            else:
                return False
                
