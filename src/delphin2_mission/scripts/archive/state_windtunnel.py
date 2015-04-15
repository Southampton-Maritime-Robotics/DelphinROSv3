#!/usr/bin/env python

import roslib; roslib.load_manifest('delphin2_mission')
import rospy
import numpy
import smach
import smach_ros
import time


class windtunnel(smach.State):
    def __init__(self, lib):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.__controller = lib
            
    def execute(self, userdata): 
            
        ####################################################################
        ### TAIL SECTION ###################################################
        ####################################################################
        
        time_zero=time.time()
        time.sleep(10)
        print "zero control surfaces"
        self.__controller.setControlSurfaceAngle(0,0,0,0)
        time.sleep(60)
        print "Aquire NOW!!!"
        time.sleep(20)
        
        #comment out from here

#        while not rospy.is_shutdown():
#         
#            print time.time()-time_zero
#            if (time.time()-time_zero)<1: # in second
#                rpm=rpm+self.__controller.getPropRPM()
#                counter=counter+1.0
#				 pass

# to here!


        #self.__controller.setRearProp(15.3)
        #time.sleep(1) # in seconds, original value of 60 sec
        print "Setting Rudder Angle 5 deg"
        self.__controller.setControlSurfaceAngle(0,5,0,5)
        time.sleep(60)
        print "Aquire NOW!!!"
        time.sleep(20)
        
        print "Setting Rudder Angle 10 deg"
        self.__controller.setControlSurfaceAngle(0,10,0,10)
        time.sleep(60)
        print "Aquire NOW!!!"
        time.sleep(20)
        
        
        print "Setting Rudder Angle 15 deg"
        self.__controller.setControlSurfaceAngle(0,15,0,15)
        time.sleep(60)
        print "Aquire NOW!!!"
        time.sleep(20)
        
        print "Setting Rudder Angle 20 deg"
        self.__controller.setControlSurfaceAngle(0,20,0,20)
        time.sleep(60)
        print "Aquire NOW!!!"
        time.sleep(20)      
        
        print "Setting Rudder Angle 25 deg"
        self.__controller.setControlSurfaceAngle(0,25,0,25)
        time.sleep(60)
        print "Aquire NOW!!!"
        time.sleep(20)                         

        print "Setting Rudder Angle 0 deg"
        self.__controller.setControlSurfaceAngle(0,0,0,0)
        time.sleep(60)
        print "Aquire NOW!!!"
        time.sleep(20)


        print "Setting Rudder Angle -5 deg"
        self.__controller.setControlSurfaceAngle(0,-5,0,-5)
        time.sleep(60)
        print "Aquire NOW!!!"
        time.sleep(20)
        
        print "Setting Rudder Angle -10 deg"
        self.__controller.setControlSurfaceAngle(0,-10,0,-10)
        time.sleep(60)
        print "Aquire NOW!!!"
        time.sleep(20)
        
        
        print "Setting Rudder Angle -15 deg"
        self.__controller.setControlSurfaceAngle(0,-15,0,-15)
        time.sleep(60)
        print "Aquire NOW!!!"
        time.sleep(20)
        
        print "Setting Rudder Angle -20 deg"
        self.__controller.setControlSurfaceAngle(0,-20,0,-20)
        time.sleep(60)
        print "Aquire NOW!!!"
        time.sleep(20)   
        
        print "Setting Rudder Angle -25 deg"
        self.__controller.setControlSurfaceAngle(0,-25,0,-25)
        time.sleep(60)
        print "Aquire NOW!!!"
        time.sleep(20)                       

        time.sleep(5)
        while not rospy.is_shutdown():
#         
#            print time.time()-time_zero
            if (time.time()-time_zero)<100: # in second
#                rpm=rpm+self.__controller.getPropRPM()
#                counter=counter+1.0
				 pass
#      
            else:
#                rpm=rpm/counter
#                str="RPM= %s" %rpm
#                print str 
#                rospy.loginfo(str)
#                self.__controller.setRearProp(0)
#                time.sleep(2)
		    return 'succeeded'
                    
                #return 'preempted'

                #return 'aborted'  
