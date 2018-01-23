#!/usr/bin/python

"""
A heading controller for the AUV when moving on a horizontal plane based-on Sliding Mode Control (SMC) approach.

"""

from __future__ import division
import rospy
import time
import numpy as np
import copy
from hardware_interfaces.msg    import tsl_setpoints
from hardware_interfaces.msg    import tail_setpoints
from hardware_interfaces.msg    import compass
from navigation.msg             import position
from lowlevel_controllers.msg   import heading_control_SMC
from std_msgs.msg               import Float32
from std_msgs.msg               import String
from hardware_interfaces.msg    import status

from delphin2_mission.utilities     import uti

class controller_SMC(object):
################################################################################
#### CONTROLLER PARAMETERS #####################################################
################################################################################
    def __init__(self):
        ### loop timing control ###
        self.controlRate = rospy.get_param("heading/ControlRate") # typically 5 [Hz]
        self.dt = 1./self.controlRate
        
        ### controller parameter ###
        self.timelastDemand_max = 1 # [sec] if there is no new demand available for this many seconds, the controller will be off.
        self.yawRateDemand_slope = 0.1*0  # to control a shape of yaw rate demand [deg/s per deg of heading error]
        self.yawRateDemand_sat = 30     # to control a shape of yaw rate demand [deg/s]
        h1 = 1                          # gain to compute sliding variable
        h2 = 0.01*h1                    # gain to compute sliding variable
        self.h = np.array([h1,h2])      # gain to compute sliding variable
        self.k_s_1 = rospy.get_param("control/heading/SMC/K1") # gain for a sliding term
        self.k_s_2 = rospy.get_param("control/heading/SMC/K2") # gain for an integral sliding term
        self.bound_int = 1.2*self.dt   # bound on the integral sliding term
        self.sw_bl = rospy.get_param("control/heading/SMC/Sigma") # boundary layer thickness for tanh function
        
        ### AUV model parameters ###
        ## rigid-gody parameter
        self.L_AUV  = 1.96  # length of the AUV [m]
        self.Izz    = 35    # moment of inertia about z-axis [kg.m^2]
        self.rho    = 1000  # water density [kg/m^3]
        
        ## hydrodynamic parameters
        # added masses and added inertia
        self.X_u_dot = -2.4102  # [kg]
        self.Y_v_dot = -65.4693 # [kg]
        self.N_r_dot = -14.1749 # [kg.m^2/rad]
        # damping: nonlinear
        self.N_vv = 59.032  # [kg];
        self.N_rr = -82     # [kg.m^2/rad^2]
        
        # added inertia matrix
        M = np.array([[self.Izz-self.N_r_dot,  0.],
                      [0.,                     1.]])
        self.M_inv = np.linalg.inv(M)
        
        ## actuator parameter
        # rudder
        self.u_R_lim    = 30                    # hard limit on the rudder demand 
        self.N_uu_delta =  0.32536336           # gain for the moment produced by thruster in according to forward velocity and rudder angle
        # thruster parameters
        self.u_th_lim           = rospy.get_param("thruster/SetpointMax") # hard limit on the horizontal thrusters demand
        self.deadband_th        = 145           # deadband of the thruster demand
        self.c1_th              = 0.35          # thruster transient model parameter
        self.c2_th              = 1.5           # thruster transient model parameter
        self.D_th               = 0.070         # thruster diameter [m]
        self.K_T_th             = 1.2870e-04    # thrust coefficient [-]
        self.L_th_h             = 1.245         # distance between horizontal thrusters [m]

        
        # Transformation matrix for a generalised moment
        b = np.array([1,  0.])
        self.B = np.dot(self.M_inv,b)
        self.inv_hB = 1./np.dot(self.h,self.B) # may need "np.linalg.inv" when doing an inverse of matrix
        
        ## utility functions
        self.__myUti = uti()
        
        ### ros communication ###
        # create publishers and messages to be published
        self.pub_tsl  = rospy.Publisher('TSL_setpoints_horizontal', tsl_setpoints)
        self.pub_tail = rospy.Publisher('tail_setpoints_vertical', tail_setpoints)
        self.pub_HC   = rospy.Publisher('Heading_controller_values_SMC', heading_control_SMC)
        self.pubMissionLog = rospy.Publisher('MissionStrings', String)
        self.pubStatus = rospy.Publisher('status', status)
        self.HC = heading_control_SMC()
        
        # create subscribers and parameters for callback functions
        rospy.Subscriber('heading_demand', Float32, self.heading_demand_cb)
        rospy.Subscriber('compass_out', compass, self.compass_cb)
        rospy.Subscriber('position_dead', position, self.position_dead_cb)

        self.timeLastCallback_headingDemand = time.time()
        self.th_rpm_fh = 0
        
################################################################################
######## CONTROLLER ############################################################
################################################################################
    def compute_yawRateDemand(self,errHeading):
        yawRateDemand = -errHeading*self.yawRateDemand_slope # [deg/s]
        yawRateDemand = self.__myUti.limits(yawRateDemand, -self.yawRateDemand_sat, self.yawRateDemand_sat)
        return yawRateDemand # [deg/s]
        
#    def generalisedMomentControlLaw
#    def controlAllocation

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
        headingDemand_old = 0
        yawRateDemand_old = 0
        s_old = 0
        sw_int = 0
        
        while not rospy.is_shutdown():
            # to control a timing for status publishing
            if time.time()-timeZero_status > dt_status:
                timeZero_status = time.time()
                self.pubStatus.publish(nodeID = 7, status = True)
                
            timeRef = time.time()
            
            heading     = self.HC.heading
            surgeVel    = self.HC.forwardVel
            swayVel     = self.HC.swayVel
            yawRate     = self.HC.yawRate # [rad/s]
            th_rpm      = self.th_rpm_fh # assumed front and rear thruster are operating at the same speed
            
            if time.time()-self.timeLastCallback_headingDemand < self.timelastDemand_max:
                self.HC.controller_onOff = True
                
                ## update controller parameters
                headingDemand = self.HC.headingDemand # [deg]
                heading_error = self.__myUti.computeHeadingError(heading,headingDemand) # [deg]
                
                yawRateDemand = self.compute_yawRateDemand(heading_error)*np.pi/180. # [rad/s]
                yawRate_error = yawRate-yawRateDemand # [rad/s]
                
                headingDemand_der = (headingDemand-headingDemand_old)/self.dt
                yawRateDemand_der = (yawRateDemand-yawRateDemand_old)/self.dt
                headingDemand_old = headingDemand
                yawRateDemand_old = yawRateDemand
                
                x = np.array([yawRate, heading]) # [rad/s, deg]
                x_err = np.array([yawRate_error, heading_error]) # [rad/s, deg]
                x_demand_der = np.array([yawRateDemand_der, headingDemand_der]) # [rad/s, deg]
                
                ## update coefficient matrices
                # update damping: linear (set B - pre-multiplied with 1000)
                N_v = 4.539*0.5*self.L_AUV**3*surgeVel  # [kg.m/s]
                N_r = -5.348*0.5*self.L_AUV**4*surgeVel # [kg.m^2/rad/s]
                D = np.array([[-N_r-self.N_rr*np.abs(yawRate),  0.],
                              [-180./np.pi,                     0.]])
                f = np.array([(self.X_u_dot-self.Y_v_dot)*swayVel*surgeVel-(N_v+self.N_vv*np.abs(swayVel))*swayVel,  0.])
                
                A = np.dot(-self.M_inv,D)
                F = np.dot(-self.M_inv,f)
                
                s = np.dot(self.h,x_err)
                s_dot = (s-s_old)/self.dt
                s_old = s
                
                ## generalise yaw moment control law
                # equivalent control law
                N_eq = self.inv_hB * ( np.dot(self.h,x_demand_der) - np.dot(self.h,F) - np.dot(self.h,np.dot(A,x)) )
                # switching control law
                sw_int = self.__myUti.limits(sw_int + self.k_s_2*np.tanh(s/self.sw_bl), -self.bound_int, self.bound_int)
                sw_term = self.k_s_1*np.tanh(s/self.sw_bl) + sw_int
                N_sw = -self.inv_hB * sw_term

                # total genaralised yaw moment required
                N_summed = N_eq + N_sw
                
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
                sw_int = 0
                heading_error = 0
                yawRateDemand = 0
                self.HC.controller_onOff = False
                s = 0
                s_dot = 0
                N_eq = 0
                N_sw = 0
                u_R = 0
                u_th = 0
            
            self.HC.heading_error = heading_error
            self.HC.yawRateDemand = yawRateDemand
            self.HC.s       = s
            self.HC.s_dot   = s_dot
            self.HC.N_eq    = N_eq
            self.HC.N_sw    = N_sw
            self.HC.u_R     = u_R
            self.HC.u_th    = u_th
            self.HC.sw_int     = sw_int
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
    def heading_demand_cb(self, newHeadingDemand):
        self.HC.headingDemand = newHeadingDemand.data
        self.timeLastCallback_headingDemand = time.time()

    def compass_cb(self, newCompassInfo):
        self.HC.heading = newCompassInfo.heading # [deg]
        self.HC.yawRate = newCompassInfo.angular_velocity_z # available in rad/s
        
    def position_dead_cb(self, newPositionInfo):
        self.HC.forwardVel = newPositionInfo.forward_vel
        self.HC.swayVel  = newPositionInfo.sway_vel
        self.th_rpm_fh = newPositionInfo.th_rpm_fh
        
if __name__ == '__main__':
    time.sleep(1) #Allow System to come Online
    rospy.init_node('Heading_controller')
    controller = controller_SMC()
    time.sleep(2) #Allow the heading demand flag to be set off
    controller.spin()
