#!/usr/bin/env python

'''
Possibly, a state to get the AUV to a certain heading.

May not functioning!

'''

import rospy
import smach
import smach_ros
import time
from std_msgs.msg import String

class GoToHeading(smach.State):
    def __init__(self, lib, demand, tolerance, stable_time, timeout):
            smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
            self.__controller = lib
            self.__demand = demand
            self.__tolerance = tolerance
            self.__stable_time = stable_time
            self.__timeout = timeout
            self.__at_heading_time = time.time()


    def execute(self,userdata):
            global pub
            global error
            #Set Up Publisher for Mission Control Log
            pub = rospy.Publisher('MissionStrings', String)
            time_zero=time.time()
            str='Entered GoToHeading State at %s' %time_zero
            pub.publish(str)
            rospy.loginfo(str)
            str='Desired Heading =%.3f deg' %(self.__demand) 
            pub.publish(str)
            rospy.loginfo(str)
            
            time_zero = time.time()
            at_heading = False
            
            
            self.__controller.setHeading(self.__demand)
            
            ##### Main loop #####
            while (time.time()-time_zero < self.__timeout) and self.__controller.getBackSeatErrorFlag() == 0:
            
            
                    demand = (self.__demand)%360

                    error  = demand - self.__controller.getHeading()
    
                    if error <-180:
                           error =   error%360
                    if error > 180:
                           error= -(-error%360)
                
                    at_heading = self.check_heading()
                
                
                    if at_heading == True:
                           return 'succeeded'
                
            ##### Main loop #####

            if self.__controller.getBackSeatErrorFlag() == 1:
                return 'preempted'
            else:
                return 'aborted'  
            
    def check_heading(self):                
            
            if (abs(error) >= self.__tolerance):
                self.__at_heading_time = time.time()
                
            if time.time()-self.__at_heading_time > self.__stable_time:
                return True
            else:
                return False
                
