#!/usr/bin/env python

import rospy
import numpy
import smach
import smach_ros
import time
from std_msgs.msg import String


class GoToHome(smach.State):
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
            
            message = 'Starting goHOME state. \n' + \
                      'Current position: X=%s Y=%s Z=%s \n' %(self.__controller.getX(), self.__controller.getY(), self.__controller.getDepth()) + \
                      'Going to position: X=%s Y=%s Z=%s ' %(self.__WpXnew, self.__WpYnew,self.__depthDemand) 
            
            print 'Message = ',message
            
            self.__controller.sendSMS(message)
            
            time.sleep(2.5)
            time_zero = time.time()
            at_depth_time = 0
            at_depth = False
            self.__controller.setDepth(self.__depthDemand)
            dt = 0
            
            str='Entered GoHOME State at %s' %time_zero
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
            while (not rospy.is_shutdown()) and (time.time()-time_zero < self.__timeout):
                loop_time_zero = time.time()
                
                #Get Current Position in X and Y
                
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
                
                Range=numpy.sqrt((X-self.__WpXnew)**2+(Y-self.__WpYnew)**2)
                TX=self.__WpXnew
                TY=self.__WpYnew
 
                #Find heading demand from current position to target location
                if (TY-Y)>0:
                    dummy=(TX-X)/(TY-Y)
                    headingDemand=2*numpy.pi+numpy.arctan(dummy)
                    headingDemand=headingDemand%(2*numpy.pi)
                elif (TY-Y)<0:
                    dummy=(TX-X)/(TY-Y)
                    headingDemand=numpy.pi+numpy.arctan(dummy)
                elif ((TY-Y)==0 and (TX-X)>0):
                    headingDemand=numpy.pi
                else:
                    headingDemand=3/4*numpy.pi
                
                #Convert headingDemand to degrees
                headingDemand=headingDemand/numpy.pi*180
                
                #Set Heading demand!
                self.__controller.setHeading(headingDemand)

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
                
                if (abs(headingError) > 90):
                    speedDemand = 0.6
                    self.__controller.setSpeed(speedDemand)


                if (abs(headingError) <= 90 and (range > self.__XYtolerance)):
                    speedDemand = 0.8 
                    self.__controller.setSpeed(speedDemand)


#                if (abs(headingError) < 20): Makes system unstable?
#                    propDemand = 15
        
                #Set Propeller Setpoint
                
                
                #Main Loop Calculations Performed                
                str='GoToXYZ: Range=%.3f m, Heading Dem=%.1f deg, Curr Heading=%.1f deg, Heading Error =%.1f deg, Speed Dem=%.1f, Curr Depth=%.2f, Depth Dem=%1.f' %(Range, headingDemand, heading, headingError, speedDemand,depth,self.__depthDemand)  
                pub.publish(str)
                print str
                
                #Only want to update demands every 0.5s hence following wait statements
                dt = time.time() - loop_time_zero
                while dt < 0.5:
                    dt = time.time() - loop_time_zero

               
            ##### Exited Main loop #####


            rospy.logerr("GoHOME Timed Out")  
            str='GoHOME state timed out, state aborted'
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
    
