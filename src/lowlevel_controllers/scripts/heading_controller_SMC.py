#!/usr/bin/python

"""
A heading controller for the AUV when moving on a horizontal plane based-on Sliding Mode Control (SMC) approach.

"""

import rospy
import serial
import time
import numpy
from hardware_interfaces.msg    import tsl_setpoints
from hardware_interfaces.msg    import tail_setpoints
from hardware_interfaces.msg    import compass
from lowlevel_controllers.msg   import heading_control
from std_msgs.msg               import Float32
from std_msgs.msg               import Int8
from std_msgs.msg               import Bool
from std_msgs.msg               import String
from hardware_interfaces.msg import status

from delphin2_mission.utilities     import uti

################################################################################
#### CONTROLLER PARAMETERS #####################################################
################################################################################

class controller_SMC(object):
    def __init__(self):
        ### loop timing control ###
        self.controlRate = 5. # [Hz]
        self.dt = 1./self.controlRate
        
        ### ros communication ###
        # create publishers and messages to be published

        # create subscribers and parameters for callback functions

################################################################################
######## AUV MODEL #############################################################
################################################################################
    def some_method(self,u,_nu):
        F_prop = self.propeller_model(u[0],_nu)
        X_cs, Y_cs, N_cs = self.rudder_model(u[1],_nu)
        F_th_fh, self.thruster_rpm_fh = self.thruster_model(u[2],self.thruster_rpm_fh,_nu)
        F_th_ah, self.thruster_rpm_ah = self.thruster_model(u[3],self.thruster_rpm_ah,_nu)
        
        f = np.array([F_prop, X_cs, Y_cs, N_cs, F_th_fh, F_th_ah])
        
        return f

    def spin(self):
        r = rospy.Rate(self.controlRate)
        
        # to control a timing for status publishing
        timeZero_status = time.time()
        try:
            dt_status = rospy.get_param('status_timing')
        except:
            dt_status = 2.
            
        while not rospy.is_shutdown():
            # to control a timing for status publishing
            if time.time()-timeZero_status > dt_status:
                timeZero_status = time.time()
                self.pubStatus.publish(nodeID = 7, status = True)
                
            timeRef = time.time()

            ## control law
            
            ## control allocation
                    
            ## pack the information into the message and publish to the topic
            
            ## Verify and maintain loop timing
            timeElapse = time.time()-timeRef
            if timeElapse < self.dt:
                r.sleep()
            else:
                str = "heading_controller rate does not meet the desired value of %.2fHz: actual control rate is %.2fHz" %(self.controlRate,1/timeElapse)
                rospy.logwarn(str)
                self.pubMissionLog.publish(str)

################################################################################
######## SATURATION AND UPDATE PARAMETERS FROM TOPICS ##########################
################################################################################
    def demand_prop_cb(self, newDemand_th):
        self.demand_prop = newDemand_th.data
        self.timeLastDemand_prop = time.time()
        
if __name__ == '__main__':
    time.sleep(1) #Allow System to come Online
    rospy.init_node('Heading_controller')
    controller = controller_SMC()
    controller.spin()
