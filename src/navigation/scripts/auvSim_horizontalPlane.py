'''
This 3DOF mathematical model includes surge sway and yaw dynamics. It is used as an observer to estimate the AUV state - denoted as nu - in according to the known actuator demands.
Drag components due to wave making and thruster operation are included.
'''

# get actuator demands from the topics and compact them into a vector u
# use Runge-Kutta method to update the state vector accordingly
# publish the AUV state to the corresponding topics

from __future__ import division
import numpy as np
import math
from copy import deepcopy
import time
import pylab
from mpl_toolkits.mplot3d import Axes3D

class delphin2_AUV(object):
    def __init__(self):
        ## initial state of the AUV
        self.nu = np.array([0,0,0]) # [u: surge vel (m/s), v: sway vel (m/s), r: yaw rate (rad/s) ]
        
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
        
        ## actuator parameters
        # propeller Turnock2010 e.q. 16.19 
        self.Kt0_prop       = 0.094554711647155 # K_t-J relation
        self.a_prop         = 0.699859170492637 # K_t-J relation
        self.b_prop         = 1.520485949644561 # K_t-J relation
        self.w_t_prop       = 0.36              # wake fraction [-]
        self.t_prop         = 0.11              # thrust deduction [-]
        self.D_prop         = 0.305             # peopeller diameter [m]
        self.deadband_prop  = 10                # deadband on propeller setpoint
        
        # thruster
        self.Kn_th              = 2.3483
        self.Ku_th              = 2.3553
        self.c1_th              = 0.35
        self.c2_th              = 1.5
        self.D_th               = 0.070 # thruster diameter [m]
        self.K_T_th             = 1.2870e-04
        self.thruster_rpm_fh    = 0. # initial thruster rpm
        self.thruster_rpm_ah    = 0. # initial thruster rpm
        self.deadband_th        = 145 # deadband on thruster setpoint
        l_th_frt_nose           = 0.205 # location of the front horizontal thruster w.r.t. AUV nose [m]
        l_th_aft_nose           = 1.45 # location of the aft horizontal thruster w.r.t. AUV nose [m]
        l_th_fh                 = O_b[0]-l_th_frt_nose # location of the front horizontal thruster wrt. origin of b-frame [m]
        l_th_ah                 = O_b[0]-l_th_aft_nose # location of the aft horizontal thruster wrt. origin of b-frame [m]
        self.a_th_drag          = 0.565993917435650 # coefficient for extradrad due to thruster operation
        self.b_th_drag          = -7.608906885747783 # coefficient for extradrad due to thruster operation
        self.c_th_drag          = 0.056535966126787 # coefficient for extradrad due to thruster operation
        self.d_th_drag          = -0.896791505996002 # coefficient for extradrad due to thruster operation

        # rudders
        self.X_uu_delta_delta    = -0.003632126810901
        self.Y_uu_delta          = -0.324087707662035
        self.N_uu_delta          =  0.325363360000000
        
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
#        self.X_uu = -2.788      # [kg/m]
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
        
        # actuator configuration matrix
        t_prop      = np.array([1, 0, 0])
        t_cs_drag   = np.array([1, 0, 0])
        t_cs_lift   = np.array([0, 1, 0])
        t_cs_moment = np.array([0, 0, 1])
        t_th_fh     = np.array([0, 1, l_th_fh])
        t_th_ah     = np.array([0, 1, l_th_ah])
        self.t      = np.vstack((t_prop,t_cs_drag,t_cs_lift,t_cs_moment,t_th_fh,t_th_ah)).T


    def actuator_models(self,u,_nu):
        # _nu denotes a state vector at this time instance
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
        # _nu denotes a state vector at this time instance        
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
        # _nu denotes a state vector at this time instance
        # _tau denotes a force vector at this time instance

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
    
    def spin(self,controlRate):
        testScenarioID = 3 # 1: pure surge, 2: pure yaw, 3: coupled surge-yaw, 4: surge-yaw selection and 5: pullout test TODO: remove me
        self.depthNow = 1.0 # [m] current depth of the AUV
        self.dt = 1./controlRate
                
        # map the testScenarioID to the function blocks TODO: remove me
        def pureSurge():
            _timeSpan = 60 # [sec]
            _timeActuatorStart = 0 # [sec]
            _propDemand = np.array([10, 12, 14, 16, 18, 20, 22])
            _thDemand = np.array([0, 0, 0, 0, 0, 0, 0])
            _csDemand = np.array([0, 0, 0, 0, 0, 0, 0])
            if self.depthNow < self.depthSat: # at surface
                _refData = np.array([0.4191, 0.5698, 0.7140, 0.8226, 0.9002, 0.9645, 1.0310])
            else: # deeply submerge
                _refData = np.array([0.261294118, 0.337058824, 0.478823529, 0.591058824, 0.691294118, 0.838941176, 1.012705882])
            return _timeSpan, _timeActuatorStart, _propDemand, _thDemand, _csDemand, _refData
        def pureYaw():
            _timeSpan = 60 # [sec]
            _timeActuatorStart = 10 # [sec]
            _propDemand = np.array([0, 0, 0, 0, 0])
            _thDemand = np.array([500, 1000, 1500, 2000, 2500])
            _csDemand = np.array([0, 0, 0, 0, 0])
            _refData = np.array([5,11,18,26,32])*np.pi/180
            return _timeSpan, _timeActuatorStart, _propDemand, _thDemand, _csDemand, _refData
        def coupledSurgeYaw():
            _timeSpan = 80 # [sec]
            _timeActuatorStart = 0 # [sec]
            _propDemand = np.array([10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
                                    16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
                                    22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22])
            _thDemand   = np.array([0, 0, 0, 0, 800, 800, 800, 800, 1600, 1600, 1600, 1600, 2400, 2400, 2400, 2400,
                                    0, 0, 0, 0, 800, 800, 800, 800, 1600, 1600, 1600, 1600, 2400, 2400, 2400, 2400,
                                    0, 0, 0, 0, 800, 800, 800, 800, 1600, 1600, 1600, 1600, 2400, 2400, 2400, 2400])
            _csDemand   = np.array([0, 10, 20, 30, 0, 10, 20, 30, 0, 10, 20, 30, 0, 10, 20, 30,
                                    0, 10, 20, 30, 0, 10, 20, 30, 0, 10, 20, 30, 0, 10, 20, 30,
                                    0, 10, 20, 30, 0, 10, 20, 30, 0, 10, 20, 30, 0, 10, 20, 30])
            _refData    = np.array([0, 3.02, 4.51, 5.36, 5.2, 6.3, 7.27, 7.78, 15, 15.2, 15.8, 16.55, 24.17, 24.3, 25.2, 26.7,
                                    0, 5.93, 7.26, 8.53, 5.93, 7.7, 8.83, 10.1, 12.44, 13.61, 15.02, 16.16, 22.24, 22.79, 23.22, 24.34,
                                    0, 7.31, 8.79, 10.32, 8.25, 8.49, 10.28, 12.14, 12.15, 13.3, 14.65, 15.48, 21, 22, 22.8, 23.72])*np.pi/180
            return _timeSpan, _timeActuatorStart, _propDemand, _thDemand, _csDemand, _refData
            
        # TODO: remove me
        testScenario = {1 : pureSurge,
                        2 : pureYaw,
                        3 : coupledSurgeYaw,
        }
        timeSpan, timeActuatorStart, propDemand, thDemand, csDemand, refData = testScenario[testScenarioID]()
        
        # TODO: remove me
        nStep = int(timeSpan/self.dt)
        indTimeStart = timeActuatorStart/self.dt
        timeVector = np.linspace(0,nStep,nStep+1)*self.dt
        nCase = len(propDemand)

        fig = pylab.figure(1)
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(thDemand, csDemand, refData, c='k', marker='^')
        
        for indCase in range(nCase):
            print 'case:', indCase
            
            # reset the state before start the new case TODO this will be removed
            self.nu = np.array([0,0,0]) # [u: surge vel (m/s), v: sway vel (m/s), r: yaw rate (rad/s) ]
            self.thruster_rpm_fh    = 0. # initial thruster rpm
            self.thruster_rpm_ah    = 0. # initial thruster rpm

#            timeRef = time.time()
            for i in range(nStep): # for [0,nStep-1]
                nu0 = deepcopy(self.nu) # create a deep copy of the state vector
                
                if timeVector[i] < timeActuatorStart:        
                    u = [0., 0., 0., 0.] # demands: [u_prop, u_cs, u_th_frt, u_th_aft]
                else:
                    u = [propDemand[indCase], csDemand[indCase], thDemand[indCase], -thDemand[indCase]] # demands: [u_prop, u_cs, u_th_frt, u_th_aft]
                f = self.actuator_models(u,nu0) # forces expressed in b-frame
                tau = np.dot(self.t,f)
                
                # form the state vector and implement Runge-Kutta 4th order
                k1 = self.rigidbodyDynamics(nu0,tau)
                k2 = self.rigidbodyDynamics(nu0+self.dt/2.*k1,tau)
                k3 = self.rigidbodyDynamics(nu0+self.dt/2.*k2,tau)
                k4 = self.rigidbodyDynamics(nu0+self.dt*k3,tau)
                                
#                delta_nu0 = self.dt/6.*(k1+2*k2+2*k3+k4)
#                self.nu = nu0 + delta_nu0
#                pylab.figure(1)
#                pylab.subplot(3,1,1)
#                pylab.grid(True)
#                pylab.ylabel('surge velocity [m/s]')
#                pylab.plot(i*self.dt,self.nu[0],'*k')
#                pylab.subplot(3,1,2)
#                pylab.grid(True)
#                pylab.ylabel('sway velocity [m/s]')
#                pylab.plot(i*self.dt,self.nu[1],'*k')
#                pylab.subplot(3,1,3)
#                pylab.grid(True)
#                pylab.ylabel('yaw velocity [rad/s]')
#                pylab.plot(i*self.dt,self.nu[2],'*k')
#                pylab.xlabel('time [sec]')

#            print time.time()-timeRef
#            ax.scatter(thDemand[indCase], csDemand[indCase], self.nu[2]*180/np.pi, c='r', marker='*')
            pylab.figure(3)
            pylab.plot(i*self.dt,self.nu[2]*180/np.pi)
            pylab.figure(2)
            if testScenarioID == 1:
                pylab.plot(propDemand[indCase],self.nu[0],'*b')
            elif testScenarioID == 2:
                pylab.plot(thDemand[indCase],self.nu[2],'*b')
        
        # show figure after running all testing scenarios

        pylab.figure(2)
        if testScenarioID == 1:
            pylab.plot(propDemand,refData,'k')
        elif testScenarioID == 2:
            pylab.plot(thDemand,refData,'k')
        pylab.show()

        
if __name__ == '__main__':
    delphin2 = delphin2_AUV()
    controlRate = 20. # [Hz]
    delphin2.spin(controlRate)
