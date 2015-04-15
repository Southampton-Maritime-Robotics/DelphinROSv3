#!/usr/bin/env python

import roslib; roslib.load_manifest('delphin2_mission')
import rospy
import smach
import smach_ros
import time
from std_msgs.msg import String


class TestHeading(smach.State):
    def __init__(self, lib, depth_demand, timeout):
            smach.State.__init__(self, outcomes=['succeeded'])
            self.__controller           = lib
            self.__depth_demand         = depth_demand
            self.__timeout              = timeout
            
    def execute(self,userdata):
        
        ######## SETUP PUBLISHER FOR MISSION LOG ################
            global pub
            #Set Up Publisher for Mission Control Log
            pub = rospy.Publisher('MissionStrings', String)
            
            time_zero = time.time()

            str= 'Entered TestHeading State at time %s' %(time_zero)
            pub.publish(str)
            
            str= 'Depth demand = %s'  %(self.__depth_demand)
            pub.publish(str)            
            rospy.loginfo(str)
        
            self.__controller.setDepth(self.__depth_demand)
            
            heading_demand=0
            str= 'heading demand = %s'  %(heading_demand)
            pub.publish(str)            
            rospy.loginfo(str)            
            self.__controller.setHeading(heading_demand) 
            time.sleep(40)

            heading_demand=90
            str= 'heading demand = %s'  %(heading_demand)
            pub.publish(str)            
            rospy.loginfo(str)            
            self.__controller.setHeading(heading_demand) 
            time.sleep(40)
            
            heading_demand=180
            str= 'heading demand = %s'  %(heading_demand)
            pub.publish(str)            
            rospy.loginfo(str)            
            self.__controller.setHeading(heading_demand) 
            time.sleep(40)
            
                        
            heading_demand=270
            str= 'heading demand = %s'  %(heading_demand)
            pub.publish(str)            
            rospy.loginfo(str)            
            self.__controller.setHeading(heading_demand) 
            time.sleep(40)                        


            return 'succeeded'
