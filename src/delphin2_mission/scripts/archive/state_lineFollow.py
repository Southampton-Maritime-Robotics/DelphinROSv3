#!/usr/bin/env python

import roslib; roslib.load_manifest('delphin2_mission')
import rospy
import smach
import smach_ros
import time
import numpy as np
from std_msgs.msg import String


class lineFollow(smach.State):
    def __init__(self, lib, timeout):
            smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
            self.__controller           = lib
            self.__timeout              = timeout
            
            
    def execute(self,userdata):
            ####################################################################
            #global pub
            pub = rospy.Publisher('MissionStrings', String)
            time_zero = time.time()
            
            ####################################################################
            str='Entered line follow state %s' %time_zero
            pub.publish(str)
	    rospy.loginfo(str)
            
            ####################################################################
            #Set speed demand
#            self.__controller.setSpeed(self.__speed_demand)
            
            #Set heading demand
#            self.__controller.setHeading(self.__heading_demand)
            ####################################################################
            
#            count           = time.time()
#            samples_in_zone = 0
            
            ####################################################################
            
            
            ##define constants
            d1 = 10	#in percentage of frame width
            d2 = 25	#in percentage of frame width
            a1 = 5	#in degree
            a2 = 25	#in degree
            line_status=False
            d=0
            a=0
            frame_width=352
	    a_keep = -1

            #Main Loop
            while time.time() - time_zero < self.__timeout and self.__controller.getBackSeatErrorFlag() == 0 and not rospy.is_shutdown():
                
                
                #Get line details from line_info_msg
                d = self.__controller.getd()			#pixels
                a = self.__controller.getAngle()			#degree
                line_status = self.__controller.getLineState()
                
		#keep the information of the heading. will be used if the vehicle lose the line


                if line_status==1:
                    lines_status=True
                else:
                    line_status=False
                
                frame_width = 352  	#pixels
                speed1 = 10				#Unit

                str = 'LineFollow: LineStatus=%s, d=%s, a=%s, Frame_width=%s' %(line_status, d, a, frame_width)  
                pub.publish(str)    
                rospy.loginfo(str)
                
                
                #########################################################################################################################
                if line_status == True:

		    a_adjust = a-90
		    d_direction = d/np.abs(d)	#unit vector of d

                    a_keep = a
                    d_keep = d

                    #calculate d_com and a_com
                    d_com = (np.abs(d)*100.0)/frame_width
                    a_com = np.abs(float(a_adjust))

		    #determine the condition of sub_case ( "1" is line in front and "2" is line behind )
                    if (d >= 0 and a < 90) or (d < 0 and a >= 90 ):
			sub_case = 1
                    elif(d >= 0 and a >= 90) or (d < 0 and a < 90 ): 
			sub_case = 2

		    sub_case_keep = sub_case
		    
                    if(a1>a_com): #start of classification for case 1, 2 and 3

			if(d1>d_com):	#case 1
				if(sub_case == 1):
                                        speedDemand = speed1
					swayDemand = 0.0
					changeHeadingBy = a_adjust
				elif(sub_case == 2):
                                        speedDemand = speed1
					swayDemand = 0.0
					changeHeadingBy = a_adjust

			if(d2>d_com and d_com >=d1): #case 2
				if(sub_case == 1):
                                        speedDemand = speed1
					swayDemand = 0.0
					changeHeadingBy = a_adjust
				elif(sub_case == 2):
                                        speedDemand = speed1
					swayDemand = 0.0
					changeHeadingBy = a_adjust

			if(d_com>=d2):	#case 3
				if(sub_case == 1):
                                        speedDemand = speed1
					swayDemand = 0.0
					if (d >= 0 and a < 90):
						changeHeadingBy = a_adjust + 2				
					elif (d < 0 and a >= 90 ): 
						changeHeadingBy = a_adjust - 2
				elif(sub_case == 2):
                                        speedDemand = 0.0
					swayDemand = 0.0
					if (d < 0 and a < 90 ):
						changeHeadingBy = a_adjust - 2				
					elif (d >= 0 and a >= 90): 
						changeHeadingBy = a_adjust + 2

                    if(a2>a_com and a_com>=a1): #start of classification for case 4, 5 and 6
			if(d1>d_com):	#case 4
				if(sub_case == 1):
                                        speedDemand = 0.0
					swayDemand = 0.0
					if (d >= 0 and a < 90):
						changeHeadingBy = a_adjust + 2				
					elif (d < 0 and a >= 90 ): 
						changeHeadingBy = a_adjust - 2
				elif(sub_case == 2):
                                        speedDemand = 0.0
					swayDemand = 0.0
					if (d < 0 and a < 90 ):
						changeHeadingBy = a_adjust - 2				
					elif (d >= 0 and a >= 90): 
						changeHeadingBy = a_adjust + 2
			if(d2>d_com and d_com >=d1): #case 5
				if(sub_case == 1):
                                        speedDemand = 0.0
					swayDemand = 0.0
					if (d >= 0 and a < 90):
						changeHeadingBy = a_adjust + 2				
					elif (d < 0 and a >= 90 ): 
						changeHeadingBy = a_adjust - 2
				elif(sub_case == 2):
                                        speedDemand = 0.0
					swayDemand = 0.0
					if (d < 0 and a < 90 ):
						changeHeadingBy = a_adjust - 2				
					elif (d >= 0 and a >= 90): 
						changeHeadingBy = a_adjust + 2
			if(d_com>=d2):	#case 6
				if(sub_case == 1):
                                        speedDemand = 0.0
					swayDemand = 0.0
					if (d >= 0 and a < 90):
						changeHeadingBy = a_adjust + 2				
					elif (d < 0 and a >= 90 ): 
						changeHeadingBy = a_adjust - 2
				elif(sub_case == 2):
                                        speedDemand = 0.0
					swayDemand = 0.0
					if (d < 0 and a < 90 ):
						changeHeadingBy = a_adjust - 2				
					elif (d >= 0 and a >= 90): 
						changeHeadingBy = a_adjust + 2

                    if(a_com>=a2): #start of classification for case 7, 8 and 9
			if(d1>d_com):	#case 7
				if(sub_case == 1):
                                        speedDemand = 0.0
					swayDemand = 0.0
					if (d >= 0 and a < 90):
						changeHeadingBy = a_adjust + 2				
					elif (d < 0 and a >= 90 ): 
						changeHeadingBy = a_adjust - 2
				elif(sub_case == 2):
                                        speedDemand = 0.0
					swayDemand = 0.0
					if (d < 0 and a < 90 ):
						changeHeadingBy = a_adjust - 2				
					elif (d >= 0 and a >= 90): 
						changeHeadingBy = a_adjust + 2
			if(d2>d_com and d_com>=d1): #case 8
				if(sub_case == 1):
                                        speedDemand = 0.0
					swayDemand = 0.0
					if (d >= 0 and a < 90):
						changeHeadingBy = a_adjust + 2				
					elif (d < 0 and a >= 90 ): 
						changeHeadingBy = a_adjust - 2
				elif(sub_case == 2):
                                        speedDemand = 0.0
					swayDemand = 0.0
					if (d < 0 and a < 90 ):
						changeHeadingBy = a_adjust - 2				
					elif (d >= 0 and a >= 90): 
						changeHeadingBy = a_adjust + 2
			if(d_com>=d2):	#case 9
				if(sub_case == 1):
                                        speedDemand = 0.0
					swayDemand = 0.0
					if (d >= 0 and a < 90):
						changeHeadingBy = a_adjust + 2				
					elif (d < 0 and a >= 90 ): 
						changeHeadingBy = a_adjust - 2
				elif(sub_case == 2):
                                        speedDemand = 0.0
					swayDemand = 0.0
					if (d < 0 and a < 90 ):
						changeHeadingBy = a_adjust - 2				
					elif (d >= 0 and a >= 90): 
						changeHeadingBy = a_adjust + 2

                #if there is no line being detected, turn off all actuators to make the vehicle stay still
                else:
			if (a_keep <= 0):
                                a_com = 0.0
				speedDemand = 0.0
				swayDemand = 0.0
				changeHeadingBy = 0.0     
			elif (a_keep >=0):


				#Get line information from the previous iteration
				d = d_keep		#only sight is needed to compare with zero
				a = a_keep		#degree

				a_adjust = a-90

				#determine the condition of sub_case ( "1" is line in front and "2" is line behind )
				if (d >= 0 and a < 90) or (d < 0 and a >= 90 ):
					sub_case = 1
				elif(d >= 0 and a >= 90) or (d < 0 and a < 90 ): 
					sub_case = 2
				
				#calculate a_com
				a_com = np.abs(float(a_adjust))

				if (a1>a_com):
					if (sub_case == 1):
						speedDemand = speed1
						swayDemand = 0.0
						if (a < 90):
							changeHeadingBy = a_adjust + 2				
						elif (a >= 90 ): 
							changeHeadingBy = a_adjust - 2
					elif (sub_case == 2):
						speedDemand = 0.0
						swayDemand = 0.0
						if (a < 90):
							changeHeadingBy = a_adjust - 2				
						elif (a >= 90): 
							changeHeadingBy = a_adjust + 2
                                        
				if (a2 > a_com and a_com >= a1):
					if (sub_case == 1):
						speedDemand = 0.0
						swayDemand = 0.0
						if (a < 90):
							changeHeadingBy = a_adjust + 2				
						elif (a >= 90 ): 
							changeHeadingBy = a_adjust - 2
					elif (sub_case == 2):
						speedDemand = 0.0
						swayDemand = 0.0
						if (a < 90):
							changeHeadingBy = a_adjust - 2				
						elif (a >= 90): 
							changeHeadingBy = a_adjust + 2
				if (a_com >= a2):
					if (sub_case == 1):
						speedDemand = 0.0
						swayDemand = 0.0
						if (a < 90):
							changeHeadingBy = a_adjust + 2				
						elif (a >= 90): 
							changeHeadingBy = a_adjust - 2
					elif (sub_case == 2):
						speedDemand = 0.0
						swayDemand = 0.0
						if (a < 90):
							changeHeadingBy = a_adjust - 2				
						elif (a >= 90): 
							changeHeadingBy = a_adjust + 2
                                        
    		changeHeadingBy=changeHeadingBy
                
                if changeHeadingBy >10:
                    changeHeadingBy =10
                if changeHeadingBy <-10:
                    changeHeadingBy =-10

                if (changeHeadingBy > 0 and changeHeadingBy < 5.1):
                    changeHeadingBy = 5.1
                if (changeHeadingBy < 0 and changeHeadingBy > -5.1):
                    changeHeadingBy = -5.1
                                 
                #publish demands
                
                #self.__controller.setRearProp(speedDemand)
                #self.__controller.sway(swayDemand)
		time.sleep(0.01)
                #self.__controller.changeHeadingBy(changeHeadingBy)
                

                str = 'LineFollow: SpeedDemand=%.2f m/s, SwayDemand=%.2f, ChangeHeadingBy=%.2f deg' %(speedDemand, swayDemand, changeHeadingBy)  
                pub.publish(str)    
                rospy.loginfo(str)

                    
                #Sleep command    
                time.sleep(0.05)
            
            
            if self.__controller.getBackSeatErrorFlag()==1:
                return 'preempted'
            else:
                return 'succeeded'
                         
