#!/usr/bin/env python

import roslib; roslib.load_manifest('delphin2_mission')
import rospy
import numpy
import math
import smach
import smach_ros
import time
from std_msgs.msg import String


class GoToXYZ(smach.State):
    def __init__(self, lib, WpXnew, WpYnew, depthDemand, XYtolerance, Ztolerance, XYstable_time, Zstable_time, timeout):
            smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
            self.__controller = lib
            self.__WpXnew = WpXnew
            self.__WpYnew = WpYnew
            self.__depthDemand=depthDemand
            self.__XYtolerance   = XYtolerance
            self.__Ztolerance    = Ztolerance
            self.__XYstable_time = XYstable_time
            self.__Zstable_time  = Zstable_time
            self.__timeout       = timeout
            self.__at_XY_time    = time.time()
            self.__at_Z_time     = time.time()
            
    def execute(self,userdata):
        global pub
        #Set Up Publisher for Mission Control Log
        pub = rospy.Publisher('MissionStrings', String)
        
        message = 'Starting goToXYZ state. \n' + \
                  'Current position: X=%s Y=%s Z=%s \n' %(self.__controller.getX(), self.__controller.getY(), self.__controller.getDepth()) + \
                  'Going to position: X=%s Y=%s Z=%s ' %(self.__WpXnew, self.__WpYnew,self.__depthDemand) 
        
        print 'Message = ',message
        
        self.__controller.sendSMS(message)
        
        time.sleep(2.5)
        time_zero = time.time()
        at_depth_time = 0
        at_depth = False
        # FIXME kantapon: uncomment detDepth
#        self.__controller.setDepth(self.__depthDemand)
        dt = 0
        
        str='Entered GoToXYZ State at %s' %time_zero
        pub.publish(str)
        rospy.loginfo(str)
        str='Desired Position X=%.3f m, Y=%.3f m and depth=%.3f m' %(self.__WpXnew, self.__WpYnew,self.__depthDemand) 
        pub.publish(str)
        rospy.loginfo(str)
        str='Initial Location is X=%.3f, Y=%.3f' %(self.__controller.getX(), self.__controller.getY())
        pub.publish(str)
        rospy.loginfo(str)
        

        dx = self.__WpXnew - self.__controller.getX()
        dy = self.__WpYnew - self.__controller.getY()
        Range = (dx**2 + dy**2)**0.5
        str = "Initial Range to Waypoint %.3f" %Range
        pub.publish(str)
        rospy.loginfo(str)
            
           
        ##### Main loop #####           
        while (not rospy.is_shutdown()) and (time.time()-time_zero < self.__timeout) and self.__controller.getBackSeatErrorFlag() == 0:
            loop_time_zero = time.time()
            
            #Get Current Position in X and Y and Current State
            X=self.__controller.getX()
            Y=self.__controller.getY()
            heading=self.__controller.getHeading()
            depth=self.__controller.getDepth()

            #Check if AUV has meet stable time and tolerance in horizontal and vertical positions
            at_Z  = self.check_Z()
            at_XY = self.check_XY()
            
            if (at_XY == True) and (at_Z == True):
                str = 'WayPoint Reached'
                pub.publish(str)
                return 'succeeded'
            
            #Calculate Heading Demand
            #Track follow algorithm based on Mcphail and Pebody, Navigation and Control of an Autonomous Underwater
            #Vehicle, Underwater Technology, Vol 23 No,1 1998
            
            DX = (self.__WpXnew-X)
            DY = (self.__WpYnew-Y)
            Range=numpy.sqrt(DX**2+DY**2)
            
            headingDemand = math.atan2(DY,DX)*180/numpy.pi # (-pi,pi]
            if headingDemand<0:
                headingDemand = 360+headingDemand

            #Set Heading demand!: FIXME Kantapon: uncomment
#            self.__controller.setHeading(headingDemand)
            print "headingDemand = ", headingDemand

            #Calculate Propeller Setpoint
            #Prop Setpoint is graduated depending on magnitude of heading error
    
            #Calculate Heading Error
            headingError  = headingDemand - heading  
            if headingError <-180:
                headingError =   headingError%360
            if headingError > 180:
                headingError= -(-headingError%360)
            
            #Calculate Propeller Setpoint
            #Prop Setpoint is graduated depending on magnitude of heading error
            

            if (abs(headingError) <= 10 and (range > self.__XYtolerance)):
                propDemand = 15
            elif (abs(headingError) <= 40 and (range > self.__XYtolerance)):
                propDemand = 12
            elif (abs(headingError) > 90):
                propDemand = 0 
                
            print "propDemand = ", propDemand
            #FIXME kantapon: uncomment
#            self.__controller.setRearProp(propDemand)                
            
            #Main Loop Calculations Performed                
            str='GoToXYZ: Range=%.3f m, Heading Dem=%.1f deg, Curr Heading=%.1f deg, Heading Error =%.1f deg, Prop Dem=%.1f, Curr Depth=%.2f, Depth Dem=%1.f' %(Range, headingDemand, heading, headingError, propDemand, depth,self.__depthDemand)  
            pub.publish(str)
            print str
            
            #Only want to update demands every 0.5s hence following wait statements
            dt = time.time() - loop_time_zero
            while dt < 0.5:
                dt = time.time() - loop_time_zero

        ##### Exited Main loop #####

        if self.__controller.getBackSeatErrorFlag() == 1:
            rospy.logerr("BackSeatDriver Identified Fault") 
            str = 'Backseat Driver Identified Fault GoToXYZ State preempted'
            pub.publish(str)
            return 'preempted'
        else:
            rospy.logerr("Go To Waypoint Timed Out")  
            str='GoToXYZ state timed out, state aborted'
            pub.publish(str)              
        return 'aborted'  
		
    #Function to check if sub has meet criteria in the vertical plane
    def check_Z(self):                

        if (abs(self.__controller.getDepth() - self.__depthDemand) >= self.__Ztolerance):
            self.__at_Z_time = time.time()
            
        if time.time()-self.__at_Z_time > self.__Zstable_time:
            return True
        else:
            return False         
        
    #Function to check if sub has meet criteria in the horizontal plane
    def check_XY(self):                
            
        if (abs(self.__controller.getX() - self.__WpXnew) >= self.__XYtolerance) or (abs(self.__controller.getY() - self.__WpYnew) >= self.__XYtolerance):
            self.__at_XY_time = time.time()
            
        if time.time()-self.__at_XY_time > self.__XYstable_time:
            return True
        else:
            return False 
    
