#!/usr/bin/env python

import roslib; roslib.load_manifest('delphin2_mission')
import rospy
import smach
import smach_ros
import time
import numpy as np
from std_msgs.msg import String


class trackAltitude(smach.State):
    def __init__(self, lib, altitude_demand, heading_demand, speed_demand, timeout):
            smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
            self.__controller           = lib
            self.__altitude_demand      = altitude_demand
            self.__heading_demand       = heading_demand
            self.__speed_demand         = speed_demand
            self.__timeout              = timeout
            
            
    def execute(self,userdata):
            ####################################################################
            #global pub
            pub = rospy.Publisher('MissionStrings', String)
            time_zero = time.time()
            
            ####################################################################
            altitude = self.__controller.getAltitude()
            str='Entered trackAltitude State at %s' %time_zero
            pub.publish(str)
	    rospy.loginfo(str)
            str='Desired Altitude =%.3f m, heading =%.3f deg, speed=%.3f m/s and timout= %s s' %(self.__altitude_demand, self.__heading_demand,self.__speed_demand, self.__timeout) 
            pub.publish(str)
            rospy.loginfo(str)
            str='Initial Altitude =%.3f m' %(altitude) 
            pub.publish(str)
            rospy.loginfo(str)
            
            ####################################################################
            #Set speed demand
            self.__controller.setSpeed(self.__speed_demand)
            
            #Set heading demand
            self.__controller.setHeading(self.__heading_demand)
            ####################################################################
            
            count           = time.time()
            samples_in_zone = 0
            
            ####################################################################
            
            while time.time() - time_zero < self.__timeout and self.__controller.getBackSeatErrorFlag() == 0 and not rospy.is_shutdown():
                
                ### Get latest feedback ###
                depth    = self.__controller.getDepth()
                alt_old  = altitude
                altitude = self.__controller.getAltitude() 
                
                ### Filter altitude data ###
                if altitude > 0.5:
                    altitude = altitude
                else:
                    altitude = alt_old
                    
                pitch =  self.__controller.getPitch()
                alt_correction = 1.2*np.tan(pitch*np.pi/180.0)
                altitude_corrected = altitude + alt_correction
                
                ### In critical zone? ###
                if (altitude_corrected < 2.0) and (altitude_corrected > 0.4):
                    samples_in_zone = samples_in_zone + 1
                    
                ### If in critical zone do the following ###
                if samples_in_zone > 5:
                    print 'In critical altitude zone!'
                    
                    if abs(altitude_corrected - alt_old) > 3:
                        str = 'Vehicle too close to the bottom, pulling up'
                        print str
                        pub.publish(str)    
                        depth_demand = depth - 1.0
                        time.sleep(5.0)
                                            
                    elif (altitude_corrected - self.__altitude_demand) < 1.0 and (altitude_corrected - self.__altitude_demand) > 0:
                        str = 'Slow approach to setpoint'
                        print str
                        pub.publish(str)    
                        alt_error  = altitude - self.__altitude_demand
                        if alt_error > 0.2:
                            alt_error = 0.2
                        depth_demand = depth + alt_error
                                        
                else:
                    str = 'Setting large change in depth demand'
                    print str
                    pub.publish(str)    
                    alt_error       = altitude_corrected - self.__altitude_demand
                    depth_demand    = depth + alt_error
                

                ### Set depth ###
                self.__controller.setDepth(depth_demand)
            
                #publish altitude control settings at frquency of 4Hz
                
                print 'time = ',time.time() - count
                if time.time() - count > 5.0:
                    str = 'trackAltitude: Altitude=%.2f m, Altitude Demand=%.2f m, Depth=%.2f m, Depth Demand =%.2f m' %(altitude, self.__altitude_demand, depth, depth_demand)  
                    pub.publish(str)    
                    rospy.loginfo(str)
                    count = time.time()
                    
                #Sleep command    
                time.sleep(0.1)
            
            
            if self.__controller.getBackSeatErrorFlag()==1:
                return 'preempted'
            else:
                return 'succeeded'
                         
