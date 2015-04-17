#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from std_msgs.msg import String


class GoForwards(smach.State):
    def __init__(self, lib, depth_demand, timeout, prop_demand):
            smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
            self.__controller           = lib
            self.__depth_demand         = depth_demand
            self.__timeout              = timeout
            self.__prop_demand          = prop_demand
            
    def execute(self,userdata):
        
        ######## SETUP PUBLISHER FOR MISSION LOG ################
            global pub
            #Set Up Publisher for Mission Control Log
            pub = rospy.Publisher('MissionStrings', String)
        
        ######## START OPERATION ################################
            
            time_zero = time.time()
            
            str= 'Entered goForwards State, started at time = %s' %(time_zero)
            pub.publish(str)
            

            heading_zero=self.__controller.getHeading()
            str= 'Depth demand = %s, Initial Heading = %s, Prop demand = %s'  %(self.__depth_demand, heading_zero, self.__prop_demand)
            pub.publish(str)
        
            self.__at_depth_time = time.time()
            
            self.__controller.setDepth(self.__depth_demand)
            self.__controller.setHeading(heading_zero)
            self.__controller.setRearProp(self.__prop_demand)
            
            ##### Main loop #####
            count = 1
            rpm_sum = 0
            while (time.time()-time_zero < self.__timeout) and self.__controller.getBackSeatErrorFlag() == 0:
                self.__controller.setRearProp(self.__prop_demand)
                
                #self.__controller.setRudderAngle(30)

		rpm=self.__controller.getPropRPM()
                rpm_sum += rpm

                count += 1

                #str = 'Current rpm = %s'%rpm
                #rospy.loginfo(str)




                time.sleep(0.05)
	    
            rpm_avg = rpm_sum/count

            str = 'Averaged rpm = %s'%rpm_avg
            rospy.loginfo(str)
                
            ##### Main loop #####
            str= 'backSeatErrorFlag = %s' %(self.__controller.getBackSeatErrorFlag())
            pub.publish(str)
            
            if (time.time()-time_zero < self.__timeout):
                str= 'goForwards succeeded at time = %s' %(time.time())
                pub.publish(str)
                return 'succeeded'
                

            if self.__controller.getBackSeatErrorFlag() == 1:
                str= 'goForwards preempted at time = %s' %(time.time())    
                pub.publish(str)
                return 'preempted'
            else:
                str= 'goForwards timed-out at time = %s' %(time.time())
                pub.publish(str)
                return 'aborted'  

