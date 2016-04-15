#!/usr/bin/python

"""
A heading controller for the AUV when moving on a horizontal plane.

A generalised moment is determined based on PI-D control strategy.
This is then allocate onto the horizontal thrusters and rudders according to forward speed.

"""

from __future__ import division
import rospy
import time
import numpy as np
from hardware_interfaces.msg    import tsl_setpoints
from hardware_interfaces.msg    import tail_setpoints
from hardware_interfaces.msg    import compass
from navigation.msg             import position
from lowlevel_controllers.msg   import heading_control_PID_ctrlAllo
from std_msgs.msg               import Float32
from std_msgs.msg               import Int8
from std_msgs.msg               import String
from hardware_interfaces.msg    import status

from delphin2_mission.utilities     import uti

################################################################################
#### CONTROLLER PARAMETERS #####################################################
################################################################################

class controller_PID(object):
    def __init__(self):
        ### loop timing control ###
        self.controlRate = 20. # [Hz]
        self.dt = 1./self.controlRate
        
        ## controller parameters
        self.P_gain = 0.4 # TODO: tune me
        self.I_gain = 0 # This must always be set to zero as to avoid an integral windup phenomenon.
        self.D_gain = -0.8 # TODO: tune me
        self.I_term_lim = 10 # TODO tume me [N.m]
        
        ## actuator parameter
        self.rho    = 1000  # water density [kg/m^3]
        # rudder
        self.u_R_lim    = 30                    # hard limit on the rudder demand 
        self.N_uu_delta =  0.32536336           # gain for the moment produced by thruster in according to forward velocity and rudder angle
        # thruster parameters
        self.u_th_lim           = 2500          # hard limit on the horizontal thrusters demand
        self.deadband_th        = 145           # deadband of the thruster demand
        self.c1_th              = 0.35          # thruster transient model parameter
        self.c2_th              = 1.5           # thruster transient model parameter
        self.D_th               = 0.070         # thruster diameter [m]
        self.K_T_th             = 1.2870e-04    # thrust coefficient [-]
        self.L_th_h             = 1.245         # distance between horizontal thrusters [m]
        
        ### ros communication ###
        # create publishers and messages to be published
        self.pub_tsl  = rospy.Publisher('TSL_setpoints_horizontal', tsl_setpoints)
        self.pub_tail = rospy.Publisher('tail_setpoints_vertical', tail_setpoints)
        self.pub_HC   = rospy.Publisher('Heading_controller_values_PID_ctrlAllo', heading_control_PID_ctrlAllo)
        self.pubMissionLog = rospy.Publisher('MissionStrings', String)
        self.pubStatus = rospy.Publisher('status', status)
        self.HC = heading_control_PID_ctrlAllo()
        
        # create subscribers and parameters for callback functions
        rospy.Subscriber('heading_demand', Float32, self.heading_demand_cb)
        rospy.Subscriber('compass_out', compass, self.compass_cb)
        rospy.Subscriber('position_dead', position, self.position_dead_cb)
        self.timeLastCallback = time.time()
        self.timeLastCallback_max = 1
        self.th_rpm_fh = 0
        
        self.__myUti = uti()
        
################################################################################
######## MAIN LOOP #############################################################
################################################################################
    def spin(self):
        r = rospy.Rate(self.controlRate)
        # to control a timing for status publishing
        timeZero_status = time.time()
        try:
            dt_status = rospy.get_param('status_timing')
        except:
            dt_status = 2.
        
        # initialise recuring parameters
        I_term = 0
        dt = 1./self.controlRate # [sec]
        
        while not rospy.is_shutdown():
            # to control a timing for status publishing
            if time.time()-timeZero_status > dt_status:
                timeZero_status = time.time()
                self.pubStatus.publish(nodeID = 7, status = True)
                
            timeRef = time.time()
            
            heading         = self.HC.heading            # [deg]
            heading_demand  = self.HC.heading_demand     # [deg]
            yawRate_deg     = self.HC.yawRate*180./np.pi # [deg/s]
            surgeVel        = self.HC.forwardVel
            swayVel         = self.HC.swayVel
            th_rpm          = self.th_rpm_fh # assumed front and rear thruster are operating at the same speed
            
            if time.time()-self.timeLastCallback < self.timeLastCallback_max:
                self.HC.controller_onOff = True
                
                ## generalised moment control law
                err = self.__myUti.computeHeadingError(heading_demand, heading) # [deg]
                P_term = self.P_gain*err
                I_term = I_term + self.I_gain*err*self.controlRate # This should be used only when needed as to avoid an integral windup phenomenon.
                I_term = self.__myUti.limits(I_term, -self.I_term_lim, self.I_term_lim)
                D_term = self.D_gain*yawRate_deg
                N_summed = P_term + I_term + D_term
                
                ## control allocation
                # allocate to rudder
                if surgeVel > 0.4:
                    u_R = N_summed/self.N_uu_delta/surgeVel**2
                    # apply rudder limit
                    if np.abs(u_R) > self.u_R_lim:
                        u_R = np.sign(u_R)*self.u_R_lim
                        N_res = N_summed - (self.N_uu_delta)*surgeVel**2*u_R
                    else:
                        u_R = u_R
                        N_res = 0
                else:
                    u_R = 0
                    N_res = N_summed
                    

                # allocate to horizontal thrusters
                if N_res != 0:
                    K_1 = np.exp(-self.c1_th*surgeVel**2)
                    # assume front and rear horizontal thruster are operating at the same speed
                    if th_rpm == 0:
                        K_2 = 1
                    else:
                        K_2 = 1-np.abs(self.c2_th*swayVel/th_rpm/self.D_th)
                        if K_2<=0.1:
                            K_2 = 0.1
                        
                    u_th = np.sign(N_res)*np.sqrt(np.abs(N_res)/self.L_th_h/self.rho/self.D_th**4/self.K_T_th/K_1/K_2)
                    u_th = self.__myUti.limits(u_th, -self.u_th_lim, self.u_th_lim)
                    
                    if np.abs(u_th) < self.deadband_th:
                        u_th = 0
                else:
                    u_th = 0

                self.pub_tail.publish(cs0 = u_R, cs1 = u_R)
                self.pub_tsl.publish(thruster0 = u_th, thruster1 = -u_th)
                
            else:
                # TODO: correct this section
                self.HC.controller_onOff = False
                err = 0
                P_term = 0
                # I_term: it is what is was in last iteration
                D_term = 0
                u_R = 0
                u_th = 0
            
            ## publishing
            self.HC.error = err
            self.HC.N_Pterm = P_term
            self.HC.N_Iterm = I_term
            self.HC.N_Dterm = D_term
            self.HC.CS_demand = u_R
            self.HC.thruster0 = u_th
            self.HC.thruster1 = -u_th
            self.pub_HC.publish(self.HC)
            
            ## Verify and maintain loop timing
            timeElapse = time.time()-timeRef
            if timeElapse < self.dt:
                r.sleep()
            else:
                str = "heading_controller rate does not meet the desired value of %.2fHz: actual control rate is %.2fHz" %(self.controlRate,1/timeElapse)
                rospy.logwarn(str)
                self.pubMissionLog.publish(str)

################################################################################
######## UPDATE PARAMETERS FROM TOPICS #########################################
################################################################################
    def heading_demand_cb(self, headingd):
        self.HC.heading_demand = headingd.data
        self.timeLastCallback = time.time()

    def compass_cb(self, compass):
        self.HC.heading = compass.heading
        self.HC.yawRate = compass.angular_velocity_z
        
    def position_dead_cb(self, newPositionInfo):
        self.HC.forwardVel = newPositionInfo.forward_vel
        self.HC.swayVel  = newPositionInfo.sway_vel
        self.th_rpm_fh = newPositionInfo.th_rpm_fh
    
################################################################################
######## INITIALISATION ########################################################
################################################################################
if __name__ == '__main__':
    time.sleep(1) #Allow System to come Online
    rospy.init_node('Heading_controller')
    controller = controller_PID()
    controller.spin()
