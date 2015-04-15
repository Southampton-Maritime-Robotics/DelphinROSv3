#!/usr/bin/python
import roslib; roslib.load_manifest('lowlevel_controllers')
import rospy
import time
import numpy as np
import scipy 
from scipy import linalg
from math  import *
import cvxopt
from cvxopt import matrix, solvers

from lowlevel_controllers.msg import tsl_setpoints
from lowlevel_controllers.msg import tail_setpoints
from lowlevel_controllers.msg import dead_reckoner
from lowlevel_controllers.msg import depthandspeed_MPC
from hardware_interfaces.msg import compass
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from std_msgs.msg import Int8


################################################################################
########## MAIN CONTROL LOOP ###################################################
################################################################################

def main_control_loop():
    
    global onOFF
    global speed_demand
    global depth_demand
    
    #### SOLVER PARAMETERS ####
    #solvers.options['show_progress'] = False   # Display solver information
    solvers.options['maxiters'] = 50            # Solver interations (default = 100, tune to stay at 10Hz)
    
    #### CONTROLLER PARAMETERS ####
    Np          = 150 #150 #120
    Nc          = 3
    gain_P      = 7.0
    gain_F      = 0.5
    gain_T      = 10.0
    Tgain       = 1.0
    delta_t     = 0.1
    Thrust_Smin = 0
    Thrust_Smax = 2500
    
    gain_matrix =    np.matrix([[gain_P, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, gain_F, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, gain_T, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, gain_T, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, gain_P, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, gain_F, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, gain_T, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, gain_T, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, gain_P, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, gain_F, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, gain_T, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, gain_T]], dtype = np.float)
    
    
    #### INITIALISE ####
    [velX, velZ, velP]    = [0, 0, 0]
    [pitch, depth]        = [0, 0]
    [prop, delta, T0, T1] = [0, 0, 0, 0]
                                                       
    [Ad, Bd, Cd, Dd] = get_model(velX,velZ,velP,pitch,delta,T0,T1,delta_t)
    [Phi_Phi, Phi_F, Phi_R, A_e, B_e, C_e] = mpc_gain(Np, Nc, Ad, Bd, Cd, Dd)   # Create augmented model
    
    time_zero = time.time()          
    [q, n]    = np.shape(Cd)
    [n, m]    = np.shape(Bd)   
    xm        = np.zeros([n,1],float)                                           # Initialise states
    Xf        = np.zeros([n+q,1],float)                                            
    u         = np.matrix([[0.0],[0.0],[0.0],[0.0]])                            # Initialise input values
    gammaN    = np.zeros([32,1],float)
    pn        = 0
    demands   = np.matrix([[speed_demand],[depth_demand]])   
    
    [gamma0] = set_constraints(Nc, m, velX, speed_demand)
    M = np.zeros([np.size(gamma0),Nc*m],float)
    M[0:16,0:4] = np.matrix([[1, 0, 0, 0],[-1, 0, 0, 0],[1, 0, 0, 0],[-1, 0, 0, 0],[0, 1, 0, 0],[0, -1, 0, 0],[0, 1, 0, 0],[0, -1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1],[0, 0, -1, 0],[0, 0, 0, -1],[0, 0, 1, 0],[0, 0, 0, 1],[0, 0, -1, 0],[0, 0, 0, -1]])
    M[16:24,4:8] = np.matrix([[1, 0, 0, 0],[-1, 0, 0, 0],[0, 1, 0, 0],[0, -1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1],[0, 0, -1, 0],[0, 0, 0, -1]])
    M[24:32,8:12] = np.matrix([[1, 0, 0, 0],[-1, 0, 0, 0],[0, 1, 0, 0],[0, -1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1],[0, 0, -1, 0],[0, 0, 0, -1]])

################################################################################
################################################################################ 
    print 'Depth controller ready'
    
    while not rospy.is_shutdown():
        
        if not backseat_driver_flag == 0:
            speed_demand = 0.0
            depth_demand = 0.0
            

        dt = time.time() - time_zero                                            # Calculate time since last calculation
        ########################################################################
        if dt >= delta_t and onOFF == True:                                     # If dt >= controller period and controller is on then proceed
            time_zero = time.time()              
                        
            #### Get latest feedback ###########################################
            velX  = DR.velX
            velZ  = DR.velZ
            velP  = DR.velP
            depth = DR.Z
            pitch = DR.pitch
            
            #### Get setpoints #################################################
            demands = [[speed_demand],[depth_demand]]              # Set demands 
            
            #### Build model ###################################################
            t = time.time()
            [Ad, Bd, Cd, Dd] = get_model(velX,velZ,velP,pitch,delta,T0,T1,delta_t)
            print 'model time = ',time.time() - t
            
            t = time.time()
            [Phi_Phi, Phi_F, Phi_R, A_e, B_e, C_e] = mpc_gain(Np, Nc, Ad, Bd, Cd, Dd)
            print 'mpc_gain time = ',time.time() - t
            
            #### Set constraints ###############################################
            [gamma0] = set_constraints(Nc, m, velX, speed_demand)
            
            gammaN[0:16,0:1] = np.matrix([[0], [0], [-u[0]], [u[0]], [0], [0], [-u[1]], [u[1]], [0], [0], [0], [0], [-u[2]], [-u[3]], [u[2]], [u[3]]])
            gamma  = gamma0 + gammaN
            
            #### Find optimal ##################################################
            E = Phi_Phi + gain_matrix                        # Calculate E matrix
            F = -(np.dot(Phi_R,demands) - np.dot(Phi_F,Xf) - np.dot(np.matrix([[0],[0],[u[2]],[u[3]],[0],[0],[0],[0],[0],[0],[0],[0]]),(Tgain*velX*abs(velX))))                     # Calculate F matrix
            
#            t = time.time()
#            [Delta_u, km] = QPhild(E, F, M, gamma, delta_t, time_zero)                       # Find optimum control horizon within constaints
#            print 'Qphild time = ',time.time() - t 
            km = 0
            
            t = time.time()
            E = matrix(E)
            F = matrix(F)
            M = matrix(M)
            b = matrix(gamma)
            
            try:
                sol = solvers.qp(E, F, M, b)            
                Delta_u = sol['x']
            except:
                Delta_u = Delta_u

            #### Set control values ############################################
            dprop  = float(Delta_u[0,0])                                        
            ddelta = float(Delta_u[1,0])      
            dT0    = float(Delta_u[2,0])                       
            dT1    = float(Delta_u[3,0])                     
            prop   = u[0] = float(u[0] + dprop)                
            delta  = u[1] = float(u[1] + ddelta) 
            
            delta = delta - pitch
            
            if depth_demand > 0.2 or depth > 0.5:                  
                T0     = u[2] = float(u[2] + dT0)                      
                T1     = u[3] = float(u[3] + dT1)    
            else:
                T0     = u[2] = 0                    
                T1     = u[3] = 0
                delta  = u[1] = -5

                    
            #### Calculate states ##############################################
            
            
            xm_old = xm                                                         # Save previous system state as xm_old
            xm     = np.dot(Ad,xm) + np.dot(Bd,u)                               # Calculate new system state using SS model
            
            xm[0]  = velX
            xm[1]  = depth                                                      # Set depth state to measured depth
            xm[3]  = pitch*pi/180.0                                             # Set pitch state to measured pitch 
            y      = np.matrix([[velX],[depth]])                                # Set y to measured depth and pitch
                                                
            Xf[0:n]   = xm - xm_old                                             # Create Xf array for use in calculating F
            Xf[n:n+q] = y   
            
            #### Convert thrust to setpoint demand #############################
            if velX > 0.4 and T0 < 1.25:
                thruster0 = 0
            else:
                thruster0 =  int(limits((1.13*np.sign(T0)*(60.0*(np.abs(T0)/(1000*0.46*0.07**4))**0.5)), Thrust_Smin, Thrust_Smax))
                
            if velX > 0.4 and T1 < 1.25:
                thruster1 = 0
            else:
                thruster1 =  int(limits((1.13*np.sign(T1)*(60.0*(np.abs(T1)/(1000*0.46*0.07**4))**0.5)), Thrust_Smin, Thrust_Smax))
            
            #### Convert prop thrust to setpoint demand ########################
            if prop > 2:
                propSP = int(round((prop + 13.9434)/1.5116))
            else:
                propSP = 0
                
            if propSP > 22:
                propSP = 22
                
                
            print 'km = ',km
            
            #### Calculation time ##############################################
            calc_time = time.time() - time_zero                               # Calculate time taken to do calculations
            ####################################################################
            print 'calc_time = ',calc_time
            print 'pn = ',pn
            print 'prop = ',prop
            print 'propSP = ',propSP
            print 'T0 = ',T0
            print 'T1 = ',T1

            ### Publish values #################################################
            pub_tsl.publish(thruster0 = thruster0, thruster1 = thruster1)   # Publish thruster setpoints
            pub_tail.publish(cs0 = -u[1], cs1 = -u[1])
            pub_prop.publish(propSP)
            pub_C.publish(onOFF = onOFF, depth = depth, depth_demand = depth_demand, pitch = pitch, speed = velX, speed_demand = speed_demand, prop = prop, dprop = dprop, ddelta = ddelta, delta = delta, dT0 = dT0, T0 = T0, T1 = T1, thruster0 = thruster0, thruster1 = thruster1, prop_gain = gain_P, delta_gain = gain_F, thruster_gain = gain_T, Np = Np, Nc = Nc, delta_t = delta_t, calc_time = calc_time, km = km, propSP = propSP)
            ####################################################################
        else:
            time.sleep(0.001)
                        
######## END OF LOW LEVEL CONTROLLER ###########################################
################################################################################
################################################################################

def set_constraints(Nc, n_in, velX, speed_demand):

    dPropmax = 0.5
    dPropmin =-0.5
    
    Tmax0    = 10.0
    Tmax1    = 10.0
    Tmin     = 0.7
    dTmax    = 2.0
    dTmin    =-2.0
    
    dCSmax   = 10.0
    dCSmin   =-10.0
    
    if speed_demand > 0.2:
        Propmin  = 2.67
        Propmax  = 17.8
    else:
        Propmax  = 0
        Propmin  = 0
   
    if velX >= 0.3:
        Tmin = 0
        
#    if velX < 0.6:
#        CSmax    = 0.0
#        CSmin    = 0.0
#    else:
#        CSmax    = 30
#        CSmin    =-30

    CSmax    = 30
    CSmin    =-30
        
        
    #gamma0 = np.transpose(np.matrix([dPropmax, -dPropmin, Propmax, -Propmin, dCSmax, -dCSmin, CSmax, -CSmin, dTmax, dTmax, -dTmin, -dTmin, Tmax0, Tmax1, -Tmin, -Tmin, dPropmax, -dPropmin, dCSmax, -dCSmin, dTmax, dTmax, -dTmin, -dTmin, dPropmax, -dPropmin, dCSmax, -dCSmin, dTmax, dTmax, -dTmin, -dTmin])) # Itialise gamma for controlling constaints
    
    gamma0 = np.matrix([[dPropmax], [-dPropmin], [Propmax], [-Propmin], [dCSmax], [-dCSmin], [CSmax], [-CSmin], [dTmax], [dTmax], [-dTmin], [-dTmin], [Tmax0], [Tmax1], [-Tmin], [-Tmin], [dPropmax], [-dPropmin], [dCSmax], [-dCSmin], [dTmax], [dTmax], [-dTmin], [-dTmin], [dPropmax], [-dPropmin], [dCSmax], [-dCSmin], [dTmax], [dTmax], [-dTmin], [-dTmin]],  dtype = np.float) # Itialise gamma for controlling constaints
    
    return [gamma0]

################################################################################
################################################################################

def get_model(velX,velZ,velP,pitch,delta,T0,T1,delta_t):                                                         

    #### FIXED MODEL VALUES ####
    massZ   = 167.5
    massX   = 85.0
    massBG  = 55.0
    I       = 70.0
    volume  = 0.08    
    Ltf     =-0.55                                                      # Moment arm of front thruster
    Ltr     = 0.49
    Lf      = 1.0
    L       = 2.0
    BG      =-0.011 #0.005
    rho     = 1000.0
    G       = massBG*9.81*BG
    #####################
    
    #### VERIABLE MODEL VALUES ####
    ## FOILS ##
    fCl     = 0.0001574
    fCd     = 3.351e-5*delta**2 + 0.0002868*delta
    fCm     = 7.563e-5
    
    ## HULL ##
    hCl     = 0.001512
    hCd     = 0.00143*pitch**2 + 0.002375*pitch + 0.06643
    hCm     = 0.0001375
    hCdp    = 4.84 #3
    hCdz    = 3.28
    
    ## THRUSTERS ##
    Told    = T0 + T1
    T_coeff = 1.0/exp(velX)
    
    
    #### UPDATED COEFFICIENTS ####
    ## FOILS ##
    FL      = 0.0 #-0.5*rho*(L**2)*fCl*velX*abs(velX)
    FDx     =-0.5*rho*(volume**(2.0/3.0))*fCd*abs(velX)
    FM      =-0.5*rho*(L**3.0)*fCm*velX*abs(velX) 
    
    ## HULL ##
    HL   = -0.5*rho*(L**2.0)*(hCl)*velX*abs(velX)*180.0/np.pi                                 
    HDx  = -0.5*rho*(volume**(2.0/3.0))*(hCd)*abs(velX)                               
    HM   =  0.5*rho*(L**3.0)*(hCm)*velX*abs(velX)*180.0/np.pi  # Sign ???
    HDp  = -0.5*rho*(volume**(2.0/3.0))*(hCdp)*abs(velP) #- 10.0 #10.0   
    HDz  = -0.5*rho*(volume**(2.0/3.0))*(hCdz)*abs(velZ) #- 20.0    
    
    #### STATE SPACE MODEL ####
    Ac = np.matrix([[(HDx + FDx)/massX, 0, 0, Told/massX, 0], 
                    [0, 0, 1, 0, 0],
                    [0, 0, HDz/massZ, HL/massZ, 0], 
                    [0, 0, 0, 0, 1],
                    [0, 0, 0, (HM + G)/I, HDp/I]], dtype = np.float)
    
    Bc = np.matrix([[1/massX, 0, 0, 0], 
                    [0, 0, 0, 0],
                    [0, FL/massZ, T_coeff/massZ, T_coeff/massZ], 
                    [0, 0, 0, 0],
                    [0, FM/I, Ltf/I, Ltr/I]],  dtype = np.float)
                    
    Cc = np.matrix([[1, 0, 0, 0, 0],[0, 1, 0, 0, 0]],  dtype = np.float)
    
    Dc = np.zeros([2,4], dtype = np.float)
    
    SYS = tuple([Ac, Bc, Cc, Dc])
    [Ad, Bd, Cd, Dd] = cont2discrete(SYS,delta_t)    
    return [Ad, Bd, Cd, Dd]
    

################################################################################
################################################################################

def QPhild(E,F,M,gamma,delta_t,time_zero):                                                        # Find optimum control horizon within constraints
    iter = 75
    
    km      = 0                                                                 # Hildreth iterations equals zero
    [n1,m1] = np.shape(M)
    eta     = -np.dot(np.linalg.inv(E),F)                                       # Find optimum control horizon

    x = True in (np.dot(M,eta) > gamma)                                         # Check if constraints have been violated
    
    if not x:                                                                 # If constaints have not been violated with optimum solution then return
        return [eta, km]
            
    H = np.dot(M,(np.dot(np.linalg.inv(E),np.transpose(M))))                    # Calculate H matrix
    K = np.dot(M,(np.dot(np.linalg.inv(E),F))) + gamma                          # Calculate K matrix
    
    [n,m] = np.shape(K)
    
    Lambda =  np.zeros([n,m],float)                                             # Initialise correction term lambda
    Lambda_p = np.zeros([n,m],float)                                            # Initialise 'old' correction term lambda_p

    al = 10                                                                     # Initialise convergance criterium to greater than 10e-8
    
################################################################################
    t = time.time()
    while (km < iter) and (al > 10e-8) and ((time.time() - time_zero) < (delta_t - 0.005)):                                                            # While number of iterations is less than permitted

        Lambda_p[:,0] =  Lambda[:,0]                                            # Set Lambda_p to old lambda before calculating the new one
        i = 0

        while i < n:
            w  = np.add(np.dot(H[i,:],Lambda), -np.dot(H[i,i],Lambda[i,0]))
            w  = np.add(w,K[i,0])
            la = np.divide(-w,H[i,i])
            
                        
            Lambda[i,0] = np.max([0.0, la])   
            i = i + 1

        al = np.dot(np.transpose(Lambda - Lambda_p),(Lambda - Lambda_p))        # Calculate square of difference between new and old lambda
        
        km = km + 1
#        if (al < 10e-8) or ((time.time() - time_zero) > (delta_t - 0.005)):                                                    # Check convergance criterium and if within limit break
#            break
################################################################################

    eta = -np.dot(np.linalg.inv(E),F) - np.dot(np.linalg.inv(E),(np.dot(np.transpose(M),Lambda))) # Calculate optimum within constaints

    return [eta, km]                                                            # Return solution and number of Hildreth algorithm iterations
################################################################################
################################################################################

################################################################################
################################################################################
def mpc_gain(Np,Nc,Ad,Bd,Cd,Dd):                                                # Create augmented model
   
    [m1,n1]   = np.shape(Cd)
    [n1,n_in] = np.shape(Bd)
    
    A_e = np.eye(n1+m1,n1+m1)
    A_e[0:n1,0:n1] = Ad
    A_e[n1:n1+m1,0:n1] = np.dot(Cd,Ad)
            
    B_e = np.zeros([n1+m1,n_in],float)
    B_e[0:n1,:] = Bd;
    B_e[n1:n1+m1,:] = np.dot(Cd,Bd)
    
    C_e = np.zeros([m1,n1+m1],float)
    C_e[:,n1:n1+m1] = np.eye(m1,m1)

    n = n1+m1
    P = np.zeros([Np*m1,n_in],float)
    F = np.zeros([Np*m1,n1 + m1],float)
    
    P[0:m1,0:n_in] = np.dot(C_e,B_e)
    F[0:m1,0:n]  = np.dot(C_e,A_e)
    
    kk = 1
    while kk < Np:
        F[kk*m1:(kk+1)*m1,:] = np.dot(F[(kk-1)*m1:kk*m1,:],A_e)        
        P[kk*m1:(kk+1)*m1,:] = np.dot(F[(kk-1)*m1:kk*m1,:],B_e)
        kk = kk+1
    
    Phi = np.zeros([Np*m1,Nc*n_in],float)
    Phi[:,0:n_in] = P
    
    i = 1
    while i < Nc:
        Phi[0:i*m1,i*n_in:(i+1)*n_in] = np.zeros([i*m1,n_in],float)
        Phi[i*m1:Np*m1,i*n_in:(i+1)*n_in] = P[0:Np*m1-i*m1,0:n_in]
        i = i + 1    
    
    Phi_Phi = np.dot(np.transpose(Phi),Phi)
    Phi_F = np.dot(np.transpose(Phi),F)
    
    [mF,nF] = np.shape(Phi_F)
    Phi_R   = Phi_F[:,(nF-m1):nF]
    
    return [Phi_Phi, Phi_F, Phi_R, A_e, B_e, C_e]
################################################################################
################################################################################

################################################################################
################################################################################
def cont2discrete(sys, dt, method="zoh", alpha=None):                           # Convert continous state space model to discrete state space model
    
    if len(sys) == 4:
        a, b, c, d = sys
    else:
        raise ValueError("First argument must either be a tuple of 4 (ss) arrays.")
    
    if method == 'zoh':
        # Build an exponential matrix
        em_upper = np.hstack((a, b))

        # Need to stack zeros under the a and b matrices
        em_lower = np.hstack((np.zeros((b.shape[1], a.shape[0])),
                              np.zeros((b.shape[1], b.shape[1])) ))
        em = np.vstack((em_upper, em_lower))
        ms = linalg.expm(dt * em)

        # Dispose of the lower rows
        ms = ms[:a.shape[0], :]
        ad = ms[:, 0:a.shape[1]]
        bd = ms[:, a.shape[1]:]
        cd = c
        dd = d

    else:
        raise ValueError("Unknown transformation method '%s'" % method)

    return ad, bd, cd, dd

################################################################################
################################################################################

def limits(value, min, max):                                                    # Contrain value within set limits
    if value < min:				   
       value = min
    elif value > max:
       value = max
    return value
    
################################################################################
################################################################################

def depth_demand_cb(depthd):
    global depth_demand
    depth_demand = depthd.data
    
def speed_demand_cb(speed_d):
    global speed_demand
    speed_demand = speed_d.data

def dead_reckoner_cb(data):
    global DR
    DR  = data 

def depth_onOff_cb(onOff):
    global onOFF
    onOFF = onOff.data
    
def compass_cb(data):
    global cur_compass
    cur_compass  = data 

def backseat_driver_cb(data):
    global backseat_driver_flag
    backseat_driver_flag  = data.data
################################################################################
#### INITIALISATION ############################################################
################################################################################

if __name__ == '__main__':

    rospy.init_node('MPC_Depth_controller')
    
    global onOFF
    global dead
    global speed_demand
    global depth_demand
    global compass
    global backseat_driver_flag
    
    backseat_driver_flag = 0
    
    cur_compass = compass()
    onOFF       = False
    [depth_demand, speed_demand] = [0.0,0.0]
    DR = dead_reckoner()
    
    rospy.Subscriber('depth_demand', Float32, depth_demand_cb)
    rospy.Subscriber('speed_demand', Float32, speed_demand_cb) 
    rospy.Subscriber('dead_reckoner', dead_reckoner, dead_reckoner_cb)
    rospy.Subscriber('Depth_onOFF', Bool, depth_onOff_cb)
    rospy.Subscriber('compass_out', compass, compass_cb)
    rospy.Subscriber('back_seat_flag', Int8, backseat_driver_cb)
    
    pub_tsl  = rospy.Publisher('TSL_setpoints_vertical', tsl_setpoints)
    pub_tail = rospy.Publisher('tail_setpoints_horizontal', tail_setpoints)
    pub_C    = rospy.Publisher('DepthandSpeed_MPC_values', depthandspeed_MPC)
    pub_prop = rospy.Publisher('prop_demand',Int8)                
    
    rospy.loginfo("Depth controller online")

    main_control_loop()
