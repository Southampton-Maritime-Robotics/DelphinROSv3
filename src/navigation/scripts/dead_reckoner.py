#!/usr/bin/env python

'''
This 3DOF mathematical model includes surge sway and yaw dynamics. It is used as an observer to estimate the AUV state - denoted as nu - in according to the known actuator demands.
Drag components due to wave making and thruster operation are included.

Position will be updated using gpsInfo at when the reliable gps measurement connection is available; Otherwise, the deadreckoning technique will be used instead.
During the abscense of gps: lat-long will be back calculated from the X_pos and Y_pos that is updated from deadreckoner

note1: actual depth of the AUV will not be considered in the mathematical model - a constant depth is used instead.
note2: this node requires either the mtdevice.py or mtdevice_dummy.py to update the AUV heading.
note3: the tf currently goes with the ros convention: the z axis is going upwards, against direction of the depth!

'''

# TODO:
# - get altitude from altimater and publish to the position topic
# - get depth from the pressure sensor and publish to the posotion topic (it is already used in tf)
# - may implement kalman filter to fuse the gps-reading with the position estimation from deadreckoner
# - fix too strong drift in y direction

from __future__ import division
import numpy as np
import math
import copy
import time

import rospy
import tf
import tf2_ros
import geometry_msgs.msg

from hardware_interfaces.msg    import compass
from hardware_interfaces.msg    import depth
from navigation.msg             import position
from std_msgs.msg               import Int8
from std_msgs.msg               import String
from hardware_interfaces.msg    import status
from hardware_interfaces.msg    import tsl_setpoints
from hardware_interfaces.msg    import tail_setpoints
from hardware_interfaces.msg    import gps
from delphin2_mission.utilities import uti

class delphin2_AUV(object):
    def __init__(self):
        ### loop timing control ###
        self.controlRate = 20. # [Hz]
        self.dt = 1./self.controlRate
        
        ### AUV model parameters ###
        ## initial state of the AUV
        self.nu = np.array([0,0,0]) # [u: surge vel (m/s), v: sway vel (m/s), r: yaw rate (rad/s) ]
        self.headingNow = 0.0 # [deg]
        
        ## constants
        self.rho = 1000. # water density [kg/m^3]
        self.depthNow = 0 # [m] a depth of the AUV in this particular moment
        self.depthSat = 0.5 # [m] a criteria to switch between deep and shallow water condition
        
        ## rigid-body parameters (estimated from solidworks)
        self.V_AUV  = 0.08                          # volume of the AUV [m^3]
        self.L_AUV  = 1.96                          # length of the AUV [m]
        self.m      = self.rho*self.V_AUV*0.992355  # mass including water inside the flaring [kg]
        self.Izz    = 35.                           # moment of inertia about z-axis expressed in b-frame [kg.m^2]
        
        ## frame of reference
        O_b         = np.array([0.8, 0, 0])
        r_g_nose    = np.array([0.8, 0, -0.06])
        self.r_g    = O_b - r_g_nose

        ## actuators models
        # propeller parameters
        self.Kt0_prop       = 0.094554711647155 # K_t-J relation
        self.a_prop         = 0.699859170492637 # K_t-J relation
        self.b_prop         = 1.520485949644561 # K_t-J relation
        self.w_t_prop       = 0.36              # wake fraction [-]
        self.t_prop         = 0.11              # thrust deduction [-]
        self.D_prop         = 0.305             # peopeller diameter [m]
        self.deadband_prop  = 10                # deadband on propeller setpoint 

        # thruster parameters
        self.Kn_th              = 2.3483    # thruster transient model parameter
        self.Ku_th              = 2.3553    # thruster transient model parameter
        self.c1_th              = 0.35      # thruster transient model parameter
        self.c2_th              = 1.5       # thruster transient model parameter
        self.D_th               = 0.070 # thruster diameter [m]
        self.K_T_th             = 1.2870e-04 # thrust coefficient [-]
        self.thruster_rpm_fh    = 0.    # initial thruster rpm
        self.thruster_rpm_ah    = 0.    # initial thruster rpm
        self.deadband_th        = 145   # deadband on thruster setpoint
        l_th_frt_nose           = 0.205 # location of the front horizontal thruster w.r.t. AUV nose [m]
        l_th_aft_nose           = 1.45  # location of the aft horizontal thruster w.r.t. AUV nose [m]
        l_th_fh                 = O_b[0]-l_th_frt_nose  # location of the front horizontal thruster wrt. origin of b-frame [m]
        l_th_ah                 = O_b[0]-l_th_aft_nose  # location of the aft horizontal thruster wrt. origin of b-frame [m]
        self.a_th_drag          = 0.565993917435650     # coefficient for extradrad due to thruster operation
        self.b_th_drag          = -7.608906885747783    # coefficient for extradrad due to thruster operation
        self.c_th_drag          = 0.056535966126787     # coefficient for extradrad due to thruster operation
        self.d_th_drag          = -0.896791505996002    # coefficient for extradrad due to thruster operation

        # rudders parameters
        self.X_uu_delta_delta    = -0.003632126810901
        self.Y_uu_delta          = -0.324087707662035
        self.N_uu_delta          =  0.325363360000000

        # actuator configuration matrix
        t_prop      = np.array([1, 0, 0])
        t_cs_drag   = np.array([1, 0, 0])
        t_cs_lift   = np.array([0, 1, 0])
        t_cs_moment = np.array([0, 0, 1])
        t_th_fh     = np.array([0, 1, l_th_fh])
        t_th_ah     = np.array([0, 1, l_th_ah])
        self.t      = np.vstack((t_prop,t_cs_drag,t_cs_lift,t_cs_moment,t_th_fh,t_th_ah)).T

        ## hydrodynamic coefficients
        # added masses and inertia
        self.X_u_dot = -2.4102  # [kg]
        self.Y_v_dot = -65.4693 # [kg]
        self.N_r_dot = -14.1749 # [kg.m^2/rad]

        # damping coefficients: linear (set B - pre-multiplied with 1000)
        self.X_u    = 0         # [kg.m/s]
        self.Y_v_p  = -28.45    # [-*1000]
        self.Y_r_p  = 12.64     # [-*1000]
        self.N_v_p  = 4.539     # [-*1000]
        self.N_r_p  = -5.348    # [-*1000]

        # damping coefficients: nonlinear
        self.Y_vv = -183.2206   # [kg/m]
        self.Y_rr = 0           # [kg/m/rad^2]
        self.N_vv = 59.032      # [kg]
        self.N_rr = -82         # [kg.m^2/rad^2]

        # rigid-body inertia matrix
        M_RB = np.array([[self.m,   0.,             0.],
                         [0.,  self.m,              self.m*self.r_g[0]],
                         [0.,  self.m*self.r_g[0],  self.Izz]])

        # added inertia matrix
        M_A = -np.array([[self.X_u_dot, 0.,             0.],
                         [0.,           self.Y_v_dot,   0.],
                         [0.,           0,              self.N_r_dot]])

        # total inertia matrix and its inverse
        M = M_RB+M_A
        self.M_inv = np.linalg.inv(M)

        ### ros communication ###
        # create publishers and messages to be published
        self.pubPosition = rospy.Publisher('position_dead', position, queue_size=10)
        self.pubStatus = rospy.Publisher('status', status, queue_size=10)
        self.pubMissionLog = rospy.Publisher('MissionStrings', String, queue_size=10)
        self.posOut = position()

        # tf broadcaster
        self.tfBroadcast = tf2_ros.TransformBroadcaster()
        self.delphintf = geometry_msgs.msg.TransformStamped()
        self.delphintf_swapped = geometry_msgs.msg.TransformStamped()
        # initialise y value, since this one is not always updated
        self.delphintf.transform.translation.y = 0
        self.delphintf_swapped.transform.translation.z = 0


        # create subscribers and parameters for callback functions
        rospy.Subscriber('gps_out', gps, self.gps_callback)
        rospy.Subscriber('prop_demand', Int8, self.demand_prop_cb)
        rospy.Subscriber('TSL_setpoints_horizontal', tsl_setpoints, self.demand_th_hor_cb)
        rospy.Subscriber('tail_setpoints_vertical', tail_setpoints, self.demand_rudder_cb)
        rospy.Subscriber('compass_out', compass, self.compass_cb)
        rospy.Subscriber('depth_out', depth, self.depth_cb)
        self.gpsInfo = gps()
        self.comInfo = compass()
        self.roll_rad = 0
        self.pitch_rad = 0
        self.yaw_rad = 0
        self.depthInfo = depth()
        self.timeLastDemand_prop = time.time()
        self.timeLastDemand_th_hori = time.time()
        self.timeLastDemand_cs_vert = time.time()
        self.timeLastDemand_max = 1  # [sec]
        self.demand_prop = 0
        self.demand_th_fh = 0
        self.demand_th_ah = 0
        self.demand_rudder = 0
        
################################################################################
######## AUV MODEL #############################################################
################################################################################
    def actuator_models(self,u,_nu):
        F_prop = self.propeller_model(u[0],_nu)
        X_cs, Y_cs, N_cs = self.rudder_model(u[1],_nu)
        F_th_fh, self.thruster_rpm_fh = self.thruster_model(u[2],self.thruster_rpm_fh,_nu)
        F_th_ah, self.thruster_rpm_ah = self.thruster_model(u[3],self.thruster_rpm_ah,_nu)
        
        f = np.array([F_prop, X_cs, Y_cs, N_cs, F_th_fh, F_th_ah])
        
        return f
        
    def propeller_model(self,u_prop,_nu):
        if np.abs(u_prop)<self.deadband_prop:   # apply deadband to the propeller demand
            F_prop = 0
        else:            
            rps = -0.005505952380952*u_prop**2 + 0.413585946573745*u_prop -1.789459930313526 # compute propeller speed [rps]
            
            if _nu[0] < 0:
                J = 0
            else:
                J = _nu[0]*(1-self.w_t_prop)/rps/self.D_prop
                
            Kt = self.Kt0_prop*(1-(J/self.a_prop)**self.b_prop)
            F_prop = self.rho*rps**2*self.D_prop**4*Kt*(1-self.t_prop)

        return F_prop

    def rudder_model(self,csAng,_nu):
        # only the forward velocity is concidered
        U_square = _nu[0]**2

        X_cs = (self.X_uu_delta_delta)*U_square*csAng**2
        Y_cs = (self.Y_uu_delta)*U_square*csAng
        N_cs = (self.N_uu_delta)*U_square*csAng
        
        return X_cs, Y_cs, N_cs
        
    def thruster_model(self,u_th,rpm_old,_nu):     
        # apply thruster deadband to the demand
        if np.abs(u_th) < self.deadband_th:
            u_th = 0
        
        # update rpm using Runge-Kutta 4th order method
        k1 = self.Ku_th*u_th - self.Kn_th*rpm_old
        k2 = self.Ku_th*u_th - self.Kn_th*(rpm_old+self.dt/2.*k1)
        k3 = self.Ku_th*u_th - self.Kn_th*(rpm_old+self.dt/2.*k2)
        k4 = self.Ku_th*u_th - self.Kn_th*(rpm_old+self.dt*k3)
        rpm = rpm_old + self.dt/6.*(k1+2*k2+2*k3+k4)
        
        # determine force due to thruster
        if np.abs(rpm) < self.deadband_th/20:
            F_thruster = 0
        else:
            
            K_1 = np.exp(-self.c1_th*(_nu[0]**2))
            K_2 = 1-np.abs(self.c2_th*_nu[1]/rpm/self.D_th)
            
            F_thruster = self.rho*self.D_th**4*self.K_T_th*rpm**2*np.sign(rpm)*K_1*K_2

        return F_thruster, rpm
    
    def rigidbodyDynamics(self,_nu,_tau):
        ## update nonlinear time-varying coefficient matrices
        # centripetal and coriolis matrix of rigid-body (assumed y_g = 0)
        C_RB = np.array([[0,                                   0,                  -self.m*(self.r_g[0]*_nu[2]+_nu[1])],
                         [0,                                   0,                  self.m*_nu[0]],
                         [self.m*(self.r_g[0]*_nu[2]+_nu[1]),  -self.m*_nu[0],     0]])
        # centripetal and coriolis matrix of added-mass
        C_A = np.array([[0,                       0,                      self.Y_v_dot*_nu[1]],
                        [0,                       0,                      -self.X_u_dot*_nu[0]],
                        [-self.Y_v_dot*_nu[1],    self.X_u_dot*_nu[0],    0]])

        # linear damping matrix
        Y_v = self.Y_v_p*0.5*self.L_AUV**2*_nu[0]   # [kg/s]
        Y_r = self.Y_r_p*0.5*self.L_AUV**3*_nu[0]   # [kg.m/rads]
        N_v = self.N_v_p*0.5*self.L_AUV**3*_nu[0]   # [kg.m/s]
        N_r = self.N_r_p*0.5*self.L_AUV**4*_nu[0]   # [kg.m^2/rad/s]
        D_l = -np.array([[self.X_u,    0,      0],
                         [0,           Y_v,    Y_r],
                         [0,           N_v,    N_r]])
        # nonlinear damping matrix (assumed no coupling between sway and yaw)
        if self.depthNow <= self.depthSat:
            self.X_uu = -2.788 # [kg/m]
        else:
            self.X_uu = -6.4907 # -10.5; % [kg/m]
        D_q = -np.array([[self.X_uu*np.abs(_nu[0]),  0,                         0],
                         [0,                         self.Y_vv*np.abs(_nu[1]),  self.Y_rr*np.abs(_nu[2])],
                         [0,                         self.N_vv*np.abs(_nu[1]),  self.N_rr*np.abs(_nu[2])]])
        
        # extra drag:
        # 1) drag due to wave making
        if self.depthNow <= self.depthSat:
            uMax = 1.6
            if _nu[0] <= uMax:
                u = _nu[0]
            else:
                u = uMax
            X_wave = -(0.034908341541532*np.exp(4.864911009750655*u)-0.034908341541532)
            _tau_extra_wave = -np.array([X_wave, 0, 0])
        else:
            _tau_extra_wave = -np.array([0, 0, 0])
        
        # 2) drag due to vertical thruster operation when deeply submerge
        if self.depthNow > self.depthSat:
            uj_ver = 0.883 # jet velocity through vertical thrusters [m/s]
            ratio_u_uj = _nu[0]/float(uj_ver)
            
            Cd_th_ver = self.a_th_drag*np.exp(self.b_th_drag*ratio_u_uj) + self.c_th_drag*np.exp(self.d_th_drag*ratio_u_uj);
            X_th_ver = -0.5*self.rho*self.V_AUV**(2./3.)*Cd_th_ver*np.abs(_nu[0])*_nu[0]*2  # time 2 vertical thrusters
            
            # actuator weighting function
            U_star_th = 0.9
            w_delta_th = 0.03
            w_th = 1-0.5*(np.tanh((_nu[0]-U_star_th)/w_delta_th)+1)
            X_th_ver = X_th_ver*w_th
            
            _tau_extra_th_ver = -np.array([X_th_ver, 0, 0])

        else:
            _tau_extra_th_ver = -np.array([0, 0, 0])
        
        # extra drag due to horizontal thruster operation
        uj_hor = np.sqrt(4*self.thruster_rpm_fh**2*self.D_th**2*self.K_T_th/np.pi) # jet velocity through horizontal thruster [m/s]
        if uj_hor == 0:
            _tau_extra_th_hor = -np.array([0, 0, 0])
        else:
            ratio_u_uj = _nu[0]/float(uj_hor)
            Cd_th_hor = self.a_th_drag*np.exp(self.b_th_drag*ratio_u_uj) + self.c_th_drag*np.exp(self.d_th_drag*ratio_u_uj)
            X_th_hor = -0.5*self.rho*self.V_AUV**(2./3.)*Cd_th_hor*np.abs(_nu[0])*_nu[0]*2 # time 2 horizontal thrusters
            _tau_extra_th_hor = -np.array([X_th_hor, 0, 0])
        
        # compute acceleration based on AUV's current state subjected to the actuating forces and moments
        C = C_RB + C_A
        D = D_l + D_q
        M_inv_nu_dot = -np.dot(( C + D ), _nu) + _tau - _tau_extra_wave - _tau_extra_th_ver - _tau_extra_th_hor
        nu_dot = np.dot(self.M_inv, M_inv_nu_dot )
        
        return nu_dot
    
    def verify_actuator_demand(self):        
        if time.time()-self.timeLastDemand_prop>self.timeLastDemand_max:
            self.demand_prop = 0
        # verify horizontal thruster demand
        if time.time()-self.timeLastDemand_th_hori>self.timeLastDemand_max:
            self.demand_th_fh = 0
            self.demand_th_ah = 0
        # verify rudder demand
        if time.time()-self.timeLastDemand_cs_vert>self.timeLastDemand_max:
            self.demand_rudder = 0
        
        u = [self.demand_prop, self.demand_rudder, self.demand_th_fh, self.demand_th_ah] # demands: [u_prop, u_cs, u_th_frt, u_th_aft]
        return u

    def spin(self):
        r = rospy.Rate(self.controlRate)
        
        # initialise the initial position
        X_pos = 0
        Y_pos = 0
        try: 
            lat_orig  = rospy.get_param('lat_orig')
            long_orig = rospy.get_param('long_orig')
        except:
            lat_orig  = 50.9567
            long_orig = -1.36735
        R = 6367500 #Radius of the earth in m
        latitude  = lat_orig
        longitude = long_orig
        
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
                self.pubStatus.publish(nodeID = 9, status = True)
                
            timeRef = time.time()
            
            ## update the velocity vector with mathematical model
            # create a copy of the state vector: if copy is not being used, change on nuO will affect self.nu
            nu0 = copy.copy(self.nu)
            
            # get actuator demand
            u = self.verify_actuator_demand()
            
            # compute force due to actuators
            f = self.actuator_models(u,nu0)
            tau = np.dot(self.t,f) # generalised forces expressed in b-frame
            
            # form the state vector and implement Runge-Kutta 4th order
            k1 = self.rigidbodyDynamics(nu0,tau)
            k2 = self.rigidbodyDynamics(nu0+self.dt/2.*k1,tau)
            k3 = self.rigidbodyDynamics(nu0+self.dt/2.*k2,tau)
            k4 = self.rigidbodyDynamics(nu0+self.dt*k3,tau)
            
            delta_nu0 = self.dt/6.*(k1+2*k2+2*k3+k4)
            self.nu = nu0 + delta_nu0
            if self.nu[0] < 0:
                self.nu[0] = 0
            
            # update heading: measured from xsens or dummy xsens
            self.headingNow = self.comInfo.heading
            self.headingNow = np.mod(self.headingNow,360)
            
            ## update position with either gps or dead reckoning
            if not self.gpsInfo.fix == 0 and self.gpsInfo.number_of_satelites >= 5:
                X_pos = self.gpsInfo.x
                Y_pos = self.gpsInfo.y
                latitude  = np.float64(self.gpsInfo.latitude)
                longitude = np.float64(self.gpsInfo.longitude)
            else:
                h_rad = self.headingNow/180*np.pi
                
                # velocity in earth frame [m/s] - X:east, Y:north
                X_vel = self.nu[0]*np.sin(h_rad)+self.nu[1]*np.cos(h_rad)
                Y_vel = self.nu[0]*np.cos(h_rad)-self.nu[1]*np.sin(h_rad)
                
                # position in earth frame [m] - X:east, Y:north                
                X_pos = X_pos + X_vel*self.dt
                Y_pos = Y_pos + Y_vel*self.dt
                
                # lat long estimation with known position from dead reckoning technique
                try:
                    brng=math.atan2(X_pos,Y_pos)
                    d=math.sqrt(X_pos**2+Y_pos**2)
                    lat1 = math.radians(lat_orig)
                    lon1 = math.radians(long_orig)
                    lat2 = math.asin(math.sin(lat1)*math.cos(d/R) + math.cos(lat1)*math.sin(d/R)*math.cos(brng))
                    lon2 = lon1 + math.atan2(math.sin(brng)*math.sin(d/R)*math.cos(lat1), math.cos(d/R)-math.sin(lat1)*math.sin(lat2))
                    latitude    = np.float64(math.degrees(lat2))
                    longitude   = np.float64(math.degrees(lon2))
                except:
                    str = "dead_reckoner fails to compute lat-long from position"
                    rospy.logwarn(str)
                    self.pubMissionLog.publish(str)
                    pass
                    
            ## pack the information into the message and publish to the topic
            self.posOut.X = X_pos
            self.posOut.Y = Y_pos
            self.posOut.forward_vel = self.nu[0]
            self.posOut.sway_vel = self.nu[1]
            self.posOut.lat         = latitude
            self.posOut.long        = longitude
            self.posOut.ValidGPSfix = self.gpsInfo.fix
            self.posOut.th_rpm_fh   = self.thruster_rpm_fh
            self.posOut.th_rpm_ah   = self.thruster_rpm_ah
            self.pubPosition.publish(self.posOut)

            # pack information into the tf broadcaster
            self.delphintf.header.stamp = rospy.Time.now()
            self.delphintf.header.frame_id = "world"
            self.delphintf.child_frame_id = "delphin2"
            self.delphintf.transform.translation.x = X_pos
            # the dead reckoner estimate alone experiences too much drift, so unless a gps fix is available,
            # the y position is not changed
            if not self.gpsInfo.fix == 0 and self.gpsInfo.number_of_satelites >= 5:
                self.delphintf.transform.translation.y = Y_pos
            self.delphintf.transform.translation.z = - self.depthInfo.depth_filt

            q = tf.transformations.quaternion_from_euler(self.roll_rad, self.pitch_rad, self.yaw_rad)
            self.delphintf.transform.rotation.x = q[0]
            self.delphintf.transform.rotation.y = q[1]
            self.delphintf.transform.rotation.z = q[2]
            self.delphintf.transform.rotation.w = q[3]

            self.tfBroadcast.sendTransform(self.delphintf)

            self.delphintf_swapped.header.stamp = rospy.Time.now()
            self.delphintf_swapped.header.frame_id = "world"
            self.delphintf_swapped.child_frame_id = "delphin2_zy_swapped"
            self.delphintf_swapped.transform.translation.x = X_pos
            # the dead reckoner estimate alone experiences too much drift, so unless a gps fix is available,
            # the y position is not changed
            if not self.gpsInfo.fix == 0 and self.gpsInfo.number_of_satelites >= 5:
                self.delphintf_swapped.transform.translation.z = Y_pos
            self.delphintf_swapped.transform.translation.y = - self.depthInfo.depth_filt
            q = tf.transformations.quaternion_from_euler(self.roll_rad, self.yaw_rad, self.pitch_rad)
            self.delphintf_swapped.transform.rotation.x = q[0]
            self.delphintf_swapped.transform.rotation.y = q[1]
            self.delphintf_swapped.transform.rotation.z = q[2]
            self.delphintf_swapped.transform.rotation.w = q[3]

            self.tfBroadcast.sendTransform(self.delphintf_swapped)
            
            ## Verify and maintain loop timing
            timeElapse = time.time()-timeRef
            if timeElapse < self.dt:
                r.sleep()
            else:
                str = "dead_reckoner rate does not meet the desired value of %.2fHz: actual control rate is %.2fHz" %(self.controlRate,1/timeElapse)
                rospy.logwarn(str)
                self.pubMissionLog.publish(str)

################################################################################
######## SATURATION AND UPDATE PARAMETERS FROM TOPICS ##########################
################################################################################
    def demand_prop_cb(self, newDemand_prop):
        self.demand_prop = newDemand_prop.data
        self.timeLastDemand_prop = time.time()
        
    def demand_th_hor_cb(self, newDemand_th):
        self.demand_th_fh = newDemand_th.thruster0
        self.demand_th_ah = newDemand_th.thruster1
        self.timeLastDemand_th_hori = time.time()
        
    def demand_rudder_cb(self, newDemand_rudder):
        self.demand_rudder = newDemand_rudder.cs0 # assume the command on top and bottom surface are identical
        self.timeLastDemand_cs_vert = time.time()
        
    def compass_cb(self, newComInfo):
        self.comInfo = newComInfo
        self.yaw_rad = np.deg2rad(self.comInfo.heading)
        self.pitch_rad = np.deg2rad(self.comInfo.pitch)
        self.roll_rad = np.deg2rad(self.comInfo.roll)
        
    def gps_callback(self, gps):
        self.gpsInfo = gps

    def depth_cb(self, new_depth):
        self.depthInfo = new_depth
    
if __name__ == '__main__':
    time.sleep(1) #Allow System to come Online
    rospy.init_node('auvsim')
    delphin2 = delphin2_AUV()
    delphin2.spin()
