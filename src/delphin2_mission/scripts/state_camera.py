#!/usr/bin/env python

import roslib; roslib.load_manifest('delphin2_mission')
import rospy
import numpy
import smach
import smach_ros
import time
from std_msgs.msg import String


class camera(smach.State):
    def __init__(self, lib, cam, rec, filename, light):
            smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
            self.__controller = lib
            self.__cam = cam
            self.__rec = rec
            self.__filename = filename
            self.__light = light
            
    def execute(self, userdata):
            pub = rospy.Publisher('MissionStrings', String)
            str='Entered Camera State' 
            pub.publish(str)
            rospy.loginfo(str)
            
#            time.sleep(10)
            
            self.__controller.camera(self.__cam,self.__rec,self.__filename)
            
            str='Camera = %s, Record = %s, Filename = %s.' %(self.__cam,self.__rec,self.__filename)
            pub.publish(str)
            rospy.loginfo(str)
            
            if self.__light:
                self.__controller.lightOnOff(True)
                str='Downwards Lighting On' 
                pub.publish(str)
                rospy.loginfo(str)
                
            if self.__light==False:
                self.__controller.lightOnOff(False)
                str='Downwards Lighting Off' 
                pub.publish(str)
                rospy.loginfo(str)
                
            
                

            return 'succeeded'
                
            #return 'preempted'

            #return 'aborted'  
            
