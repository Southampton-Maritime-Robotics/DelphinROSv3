#!/usr/bin/env python

import roslib; roslib.load_manifest('delphin2_mission')
import rospy
import smach
import smach_ros
import time
import numpy as np
from std_msgs.msg import String


class terminalZ2(smach.State):
    def __init__(self, lib, depth_demand1, depth_demand2, depth_tolerance, thrust, timeout):
            smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
            self.__controller           = lib
            self.__depth_demand1        = depth_demand1
            self.__depth_demand2        = depth_demand2
            self.__depth_tolerance      = depth_tolerance
            self.__thrust               = thrust
            self.__timeout              = timeout
            
    def execute(self,userdata):
        
        
        ######## START OPERATION ################################
            
            time_zero = time.time()
                    
            at_depth1 = False
            iter = 0
            T0   = 0.0
            T1   = 0.0
                       
################################################################################
################# Main loop ####################################################
################################################################################
            while (time.time()-time_zero < self.__timeout) and self.__controller.getBackSeatErrorFlag() == 0:
                
            ######## FIRST DEPTH ########
                self.__at_depth_time = time.time()
                self.__controller.setDepth(self.__depth_demand1)
                print 'Going to depth1'
                
                while not self.check_depth1():
                    time.sleep(0.1)
                    if (time.time()-time_zero > self.__timeout):
                        print 'Timed out!',time.time()-time_zero
                        return 'aborted'
                    
                print 'Stabilised at depth1'
                at_depth = True
                
                self.__at_depth_time = time.time()
                
                ## GET AVG T0 and T1 ##
                if at_depth == True:
                    while iter < 50:
                        data = self.__controller.getDepthandpitchMPC()
                        T0   = data.T0 + T0
                        T1   = data.T1 + T1
                        time.sleep(0.1)
                        iter = iter + 1
                iter = 0
                                    
                ## SET EXTRA THRUST ##
                T0 = T0/50.0 + self.__thrust
                T1 = T1/50.0 + self.__thrust
                thruster0 =  int(1.13*np.sign(T0)*(60*(np.abs(T0)/(1000*0.46*0.07**4))**0.5))
                thruster1 =  int(1.13*np.sign(T1)*(60*(np.abs(T1)/(1000*0.46*0.07**4))**0.5))
                self.__controller.switchDepthOnOff(0)
                self.__controller.setTSLVertical(thruster0, thruster1)
                
            ######## CHECK TO SEE IF SECOND DEPTH HAS BEEN REACHED ####
                while self.__controller.getDepth() < self.__depth_demand2:
                    print 'reached depth 2'
                    time.sleep(0.1)
                    
                return 'succeeded'

                self.__at_depth_time = time.time()
                self.__controller.setDepth(self.__depth_demand2)
                print 'Going to depth2'

                
                while not self.check_depth2():
                    time.sleep(0.1)
                    if (time.time()-time_zero > self.__timeout):
                        return 'aborted'
                
                print 'Stabilised at depth2'
                at_depth = True
                self.__at_depth_time = time.time()
                
                ## GET AVG T0 and T1 ##
                T0 = 0.0
                T1 = 0.0
                if at_depth == True:
                    while iter < 50:
                        data = self.__controller.getDepthandpitchMPC()
                        T0   = data.T0 + T0
                        T1   = data.T1 + T1
                        time.sleep(0.1)
                        iter = iter + 1
                iter = 0

                            
                ## SET EXTRA THRUST ##
                T0 = T0/50.0 - self.__thrust
                T1 = T1/50.0 - self.__thrust
                thruster0 =  int(1.13*np.sign(T0)*(60*(np.abs(T0)/(1000*0.46*0.07**4))**0.5))
                thruster1 =  int(1.13*np.sign(T1)*(60*(np.abs(T1)/(1000*0.46*0.07**4))**0.5))
                self.__controller.switchDepthOnOff(0)
                self.__controller.setTSLVertical(thruster0, thruster1)
                
            ######## STABILISE AT DEPTH 1 ####
                while self.__controller.getDepth() > self.__depth_demand1: 
                    time.sleep(0.1)
                
                
            
                
                return 'succeeded'
                    
                
                
                
            ##### Main loop #####
            str= 'backSeatErrorFlag = %s' %(self.__controller.getBackSeatErrorFlag())
            pub.publish(str)
            
            if self.__controller.getBackSeatErrorFlag() == 1:
                str= 'goToDepth preempted at time = %s' %(time.time())    
                pub.publish(str)
                return 'preempted'
            else:
                str= 'goToDepth timed-out at time = %s' %(time.time())
                pub.publish(str)
                return 'aborted'  

################################################################################            
################################################################################
    def check_depth1(self):                
            
            if (abs(self.__controller.getDepth() - self.__depth_demand1) >= self.__depth_tolerance):
                self.__at_depth_time = time.time()
                
            if time.time()-self.__at_depth_time > 30.0:
                return True
            else:
                return False
            
################################################################################            
################################################################################
    def check_depth2(self):                
            
            if (abs(self.__controller.getDepth() - self.__depth_demand2) >= self.__depth_tolerance):
                self.__at_depth_time = time.time()
                
            if time.time()-self.__at_depth_time > 30.0:
                return True
            else:
                return False
                
