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
from lowlevel_controllers.msg   import heading_control_SMC # TODO: create this message
from std_msgs.msg               import Float32
from std_msgs.msg               import Int8
from std_msgs.msg               import Bool
from std_msgs.msg               import String
from hardware_interfaces.msg    import status

from delphin2_mission.utilities     import uti
from delphin2_mission.library_highlevel     import library_highlevel

class controller_SMC(object):
################################################################################
#### CONTROLLER PARAMETERS #####################################################
################################################################################
    def __init__(self):
        ### loop timing control ###
        self.controlRate = 5. # [Hz]
        self.dt = 1./self.controlRate
        
        ### controller parameter ###
        self.timelastDemand_heading = 1 # [sec] if there is no new demand available for this many seconds, the controller will be off.
        self.yawRateDemand_slop = 0.1   # to control a shape of yaw rate demand [deg/s per deg of heading error]
        self.yawRateDemand_sat = 30     # to control a shape of yaw rate demand [deg/s]
        h1 = 1                          # gain to compute sliding variable
        h2 = 0.01*h1                    # gain to compute sliding variable
        self.h = np.array([h1,h2])      # gain to compute sliding variable
        self.k_s_1 = 0.5                # gain for a sliding term
        self.k_s_2 = 0.1                # gain for an integral sliding term
        self.bound_int = 0.35*self.dt   # bound on the integral sliding term
        self.sw_bl = 0.1                # boundary layer thicknerr for tanh function
        
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
        self.u_th_lim           = 2500          # hard limit on the horizontal thrusters demand
        self.deadband_th        = 145           # deadband of the thruster demand
        self.c1_th              = 0.35          # thruster transient model parameter
        self.c2_th              = 1.5           # thruster transient model parameter
        self.D_th               = 0.070         # thruster diameter [m]
        self.K_T_th             = 1.2870e-04    # thrust coefficient [-]
        self.L_th_h             = 1.245         # distance between horizontal thrusters [m]

        
        # Transformation matrix for a generalised moment
        b = np.array([1,  0.])
        self.B = np.dot(self.M_inv,b)
        self.hB = np.dot(self.h,self.B)
        
        ## utility functions
        self.__myUti = uti()
        
        ### ros communication ###
        # create publishers and messages to be published
        self.pub_tsl  = rospy.Publisher('TSL_setpoints_horizontal', tsl_setpoints)
        self.pub_tail = rospy.Publisher('tail_setpoints_vertical', tail_setpoints)
        self.pub_HC   = rospy.Publisher('Heading_controller_values_SMC', heading_control_SMC)
        self.pubMissionLog = rospy.Publisher('MissionStrings', String)
        self.pubStatus = rospy.Publisher('status', status)
        
        # create subscribers and parameters for callback functions
        rospy.Subscriber('heading_demand', Float32, self.heading_demand_cb)
        rospy.Subscriber('compass_out', compass, self.compass_cb)
        rospy.Subscriber('position_dead', position, self.position_dead_cb)
        self.HC = heading_control_SMC()
        self.timeLastCallback_headingDemand = time.time()
        self.controller_onOff = False
        
################################################################################
######## CONTROLLER ############################################################
################################################################################
    def compute_yawRateDemand(self,errHeading):
        yawRateDemand = -errHeading*self.yawRateDemand_slop # [deg/s]
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
        s_old = np.array([0,0])
        sw_int = 0
            
        while not rospy.is_shutdown():
            # to control a timing for status publishing
            if time.time()-timeZero_status > dt_status:
                timeZero_status = time.time()
                self.pubStatus.publish(nodeID = 7, status = True)
                
            timeRef = time.time()
            if time.time()-self.timeLastCallback_headingDemand < self.timelastDemand_heading:
                heading     = self.HC.heading
                surgeVel    = self.HC.forwardVel
                swayVel     = self.HC.swayVel
                yawRate     = self.HC.yawRate
                th_rpm      = self.HC.th_rpm_fh # assumed front and rear thruster are operating at the same speed
                
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
                              [-180./np.pi,                     1.]])
                f = np.array([(self.X_u_dot-self.Y_v_dot)*swayVel*surgeVel-(N_v+self.N_vv*np.abs(swayVel))*swayVel,  0.])
                
                A = np.dot(-self.M_inv,D)
                F = np.dot(-self.M_inv,f)
                
                s = np.dot(self.h,x_err)
                s_dot = (s-s_old)/self.dt
                
                ## generalise yaw moment control law
                # equivalent control law
                N_eq = self.hB * ( np.dot(self.h,x_demand_der) - np.dot(self.h,F) - np.dot(self.h,np.dot(A,x)) )
                # switching control law
                sw_int = self.__myUti.limits(sw_int + self.k_s_2*np.tanh(s/self.sw_bl), -self.bound_int, self.bound_int)
                sw_term = self.k_s_1*np.tanh(s/self.sw_bl) + sw_int
                N_sw = self.hB * sw_term
                # total genaralised yaw moment required
                N_summed = N_eq + N_sw
                
                ## control allocation
                # allocate to rudder
                if surgeVel > 0.4:
                    u_R = N_summed/self.N_uu_delta/surgeVel**2
                    N_res = N_summed - (self.N_uu_delta)*surgeVel**2*u_R
                else:
                    u_R = 0
                    N_res = 0
                
                # allocate to horizontal thrusters
                if N_res != 0:
                    K_2 = np.exp(-self._c1_th*surgeVel**2)
                    # assume front and rear horizontal thruster are operating at the same speed
                    if th_rpm == 0:
                        K_2 = 1
                    else:
                        K2 = 1-np.abs(self.c2_th*swayVel/th_rpm/self.D_th)
                    u_th = np.sign(N_res)*np.sqrt(np.abs(N_res)/self.L_th_h/self.rho/self.D_th**4/self.K_T_th)
                    if np.abs(u_th) < self.deadband_th:
                        u_th = 0
                else:
                    u_th = 0
                    
            else:
                pass
                # TODO: construct a message for when the controller is off
            
            print '>>>>>>>>>:', [u_R, u_th]
#            print 'heading:', self.HC.heading
#            print 'yaw rate:', self.HC.yawRate
#            print 'heading demand:', self.HC.headingDemand
#            print 'forward velocity:', self.HC.forwardVel
#            print 'sway velocity:', self.HC.swayVel
#            print '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>'
            
            ## control law
            # compute generalised control moment required
            
            ## control allocation
            # distribute the generalised moment to the available actuators
            
            ## pack the information into the message and publish to the topic
            # putlish to actuator demand topics
            # publish to heading control values topics
            
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
        self.controller_onOff = True
        self.timeLastCallback_headingDemand = time.time()

    def compass_cb(self, newCompassInfo):
        self.HC.heading = newCompassInfo.heading # [deg]
        self.HC.yawRate = newCompassInfo.angular_velocity_z # available in rad/s. TODO: check if it has to be deg/s or rad/s
        
    def position_dead_cb(self, newPositionInfo):
        self.HC.forwardVel = newPositionInfo.forward_vel
        self.HC.swayVel  = newPositionInfo.sway_vel
        self.HC.th_rpm_fh = newPositionInfo.th_rpm_fh
        
if __name__ == '__main__':
    time.sleep(1) #Allow System to come Online
    rospy.init_node('Heading_controller')
    controller = controller_SMC()
    controller.spin()
