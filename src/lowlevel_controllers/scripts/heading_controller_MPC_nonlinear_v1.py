#!/usr/bin/python
import roslib; roslib.load_manifest('DelphinROSv2')
import rospy
import serial
import time
import numpy as np
import scipy 
from scipy import linalg
from math  import *

from DelphinROSv2.msg import tsl_setpoints
from DelphinROSv2.msg import tail_setpoints
from DelphinROSv2.msg import dead_reckoner
from DelphinROSv2.msg import heading_MPC
from DelphinROSv2.msg import compass
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from std_msgs.msg import Int8


################################################################################
########## MAIN CONTROL LOOP ###################################################
################################################################################

def main_control_loop():
    
    global onOFF

    #### CONTROLLER PARAMETERS ####
    Np          = 150
    Nc          = 3
    
    gain_F      = 3
    gain_T      = 4
    Tgain       = 0.5
    delta_t     = 0.2
    Thrust_Smin =-2500
    Thrust_Smax = 2500
    
    gain_matrix =    np.matrix([[gain_F, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, gain_T, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, gain_T, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, gain_F, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, gain_T, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, gain_T, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, gain_F, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, gain_T, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, gain_T]])
    
    #### INITIALISE ####
    [velX, velH]          = [0, 0]
    [heading]             = [0]
    [delta, T2, T3]       = [0, 0, 0]
                                                       
    [Ad, Bd, Cd, Dd] = get_model(velX,velH,delta,delta_t)
    [Phi_Phi, Phi_F, Phi_R, A_e, B_e, C_e] = mpc_gain(Np, Nc, Ad, Bd, Cd, Dd)   # Create augmented model
    
    time_zero = time.time()          
    [q, n]    = np.shape(Cd)
    [n, m]    = np.shape(Bd)   
    xm        = np.zeros([n,1],float)                                           # Initialise states
    Xf        = np.zeros([n+q,1],float)                                            
    u         = np.matrix([[0.0],[0.0],[0.0]])                                  # Initialise input values
    gammaN    = np.zeros([24,1],float)
    
    

################################################################################
################################################################################ 
    print 'Heading controller ready'
    
    while not rospy.is_shutdown():

        dt = time.time() - time_zero                                            # Calculate time since last calculation
        ########################################################################
        if dt >= delta_t and onOFF == True:                                     # If dt >= controller period and controller is on then proceed
            time_zero = time.time()              
                
            #### Calculate error ###############################################
            error  = (heading_demand)%360 - cur_compass.heading
            
            if error <-180:
                error =   error%360
            if error > 180:
                error= -(-error%360)

            #### Get latest feedback ###########################################
            velX    = DR.velX
            velH    = DR.velH
            heading = cur_compass.heading
                        
            #### Build model ###################################################
            [Ad, Bd, Cd, Dd] = get_model(velX,velH,delta,delta_t)
            [Phi_Phi, Phi_F, Phi_R, A_e, B_e, C_e] = mpc_gain(Np, Nc, Ad, Bd, Cd, Dd)
            
            #### Set constraints ###############################################
            [gamma0, M] = set_constraints(Nc, m, error)
            
            gammaN[0:12,0:1] = np.matrix([[0], [0], [-u[0]], [u[0]], [0], [0], [0], [0], [-u[1]], [-u[2]], [u[1]], [u[2]]])
            
            gamma  = gamma0 + gammaN
            
            #### Find optimal ##################################################
            E = Phi_Phi + gain_matrix                        # Calculate E matrix
            F = -(np.dot(Phi_R,0) - np.dot(Phi_F,Xf) - np.dot(np.matrix([[0],[u[1]],[u[2]],[0],[u[1]],[u[2]],[0],[u[1]],[u[2]]]),Tgain))                     # Calculate F matrix
            [Delta_u, km] = QPhild(E, F, M, gamma, delta_t, time_zero)                       # Find optimum control horizon within constaints

            #### Set control values ############################################
            ddelta = float(Delta_u[0,0])      
            dT2    = float(Delta_u[1,0])                       
            dT3    = float(Delta_u[2,0])                     
            delta  = u[0] = float(u[0] + ddelta)                   
            T2     = u[1] = float(u[1] + dT2)                      
            T3     = u[2] = float(u[2] + dT3)  
            
            #### Calculate states ##############################################
            xm_old = xm                                                         # Save previous system state as xm_old
            xm     = np.dot(Ad,xm) + np.dot(Bd,u)                               # Calculate new system state using SS model
            
            xm[0]  = -np.radians(error)
            y      = -np.radians(error)
                                                
            Xf[0:n]   = xm - xm_old                                             # Create Xf array for use in calculating F
            Xf[n:n+q] = y   
            
            #### Inforce hard constraints ######################################
            if T2 > 8:
                T2 = u[1] = 8
            if T3 > 8:
                T3 = u[2] = 8
            if T2 < -8:
                T2 = u[1] = -8
            if T3 < -8:
                T3 = u[2] = -8
            if delta > 30:
                delta  = u[0] = 30
            if delta < -30:
                delta  = u[0] = -30
            
            #### Convert thrust to speed demand ################################ 
            if abs(T2) >= 0.5:
                thruster2 =  int(limits((1.13*np.sign(T2)*(60*(np.abs(T2)/(1000*0.46*0.07**4))**0.5)), Thrust_Smin, Thrust_Smax))
            else:
                thruster2 = 0
            
            if abs(T3) >= 0.5:
                thruster3 =  int(limits((1.13*np.sign(T3)*(60*(np.abs(T3)/(1000*0.46*0.07**4))**0.5)), Thrust_Smin, Thrust_Smax))
            else:
                thruster3 = 0
            print 'delta = ',delta
            #### Calculation time ##################################################
            calc_time = time.time() - time_zero                               # Calculate time taken to do calculations
            ####################################################################

            ### Publish values #################################################
            pub_tsl.publish(thruster0 = thruster2, thruster1 = thruster3)   # Publish thruster setpoints
            pub_tail.publish(cs0 = u[0], cs1 = u[0])
            pub_C.publish(onOFF = onOFF, heading = heading, heading_demand = heading_demand, error = error, speed = velX, ddelta = ddelta, delta = delta, dT2 = dT2, dT3 = dT3, T2 = T2, T3 = T3, thruster2 = thruster2, thruster3 = thruster3, delta_gain = gain_F, thruster_gain = gain_T, thruster_penalty = Tgain, Np = Np, Nc = Nc, delta_t = delta_t, calc_time = calc_time, km = km)
            ####################################################################
        else:
            time.sleep(0.001)
                        
######## END OF LOW LEVEL CONTROLLER ###########################################
################################################################################
################################################################################

def set_constraints(Nc, n_in, error):
    Propmax  = 0
    Propmin  = 0
    dPropmax = 2
    dPropmin =-2
    Tmax     = 8
    Tmin     =-8
    dTmax    = 3
    dTmin    =-3
    CSmax    = 30
    CSmin    =-30
    dCSmax   = 3
    dCSmin   =-3
    
    if abs(error) <= 5:
        Tmax     = 0    
        Tmin     = 0
            
    gamma0 = np.transpose(np.matrix([[dCSmax, -dCSmin, CSmax, -CSmin, dTmax, dTmax, -dTmin, -dTmin, Tmax, Tmax, -Tmin, -Tmin, dCSmax, -dCSmin, dTmax, dTmax, -dTmin, -dTmin, dCSmax, -dCSmin, dTmax, dTmax, -dTmin, -dTmin]])) # Itialise gamma for controlling constaints    
    M = np.zeros([np.size(gamma0),Nc*n_in],float)
    M[0:12,0:3]  = np.matrix([[1, 0, 0], [-1, 0, 0], [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, 0, 1], [0, -1, 0], [0, 0, -1], [0, 1, 0], [0, 0, 1], [0, -1, 0], [0, 0, -1]])
    M[12:18,3:6] = np.matrix([[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, 0, 1], [0, -1, 0], [0, 0, -1]])
    M[18:24,6:9] = np.matrix([[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, 0, 1], [0, -1, 0], [0, 0, -1]])
    
    return [gamma0, M]

################################################################################
################################################################################

def get_model(velX,velH,delta,delta_t):                                                         

    #### FIXED MODEL VALUES ####
    mass    = 167.5
    I       = 70
    volume  = 0.08    
    Ltf     =-0.65                                                      # Moment arm of front thruster
    Ltr     = 0.59
    L       = 2
    rho     = 1000
    #####################
    
    #### VERIABLE MODEL VALUES ####
    ## FOILS ##
    fCl     = 0.0001574
    fCm     = 7.563e-5
    
    ## HULL ##
    hCdh    = 2
    hCm     = 0.0001375
    
    #### UPDATED COEFFICIENTS ####
    ## FOILS ##
    FM      =-0.5*rho*(L**3)*fCm*velX*abs(velX)
    
    ## HULL ##                          
    HM   =  0.5*rho*(L**3)*(hCm*180/np.pi)*velX*abs(velX)
    HDh  = -0.5*rho*(volume**(2/3))*(hCdh)*abs(velH) - 10   
    
    #### STATE SPACE MODEL ####
    Ac = np.matrix([[0, 1], 
                    [0, (HM + HDh)/I]])
                    
    Bc = np.matrix([[0, 0, 0], 
                    [FM/I, Ltf/I, Ltr/I]])
                    
    Cc = np.matrix([[1, 0]])
    
    Dc = np.zeros([1,3],float)
    
        
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

    while km < iter:                                                            # While number of iterations is less than permitted

        Lambda_p[:,0] =  Lambda[:,0]                                            # Set Lambda_p to old lambda before calculating the new one
        i = 0
        while i < n:
            w = np.dot(H[i,:],Lambda) - np.dot(H[i,i],Lambda[i,0])
            w = w + K[i,0]
            la = -w/H[i,i]
            Lambda[i,0] = np.max([0, la])        
            i = i + 1
        
        al = np.dot(np.transpose(Lambda - Lambda_p),(Lambda - Lambda_p))        # Calculate square of difference between new and old lambda
        
        km = km + 1
        if (al < 10e-8) or ((time.time() - time_zero) > (delta_t - 0.01)):                                                    # Check convergance criterium and if within limit break
            if ((time.time() - time_zero) > (delta_t - 0.01)):
                print 'BROKE!'
            break
    
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

def heading_demand_cb(data):
    global heading_demand
    heading_demand = data.data
    
def sway_demand_cb(data):
    return    

def dead_reckoner_cb(data):
    global DR
    DR  = data 

def onOff_cb(onOff):
    global onOFF
    onOFF = onOff.data
    
def compass_cb(data):
    global cur_compass
    cur_compass  = data 

################################################################################
#### INITIALISATION ############################################################
################################################################################

if __name__ == '__main__':

    rospy.init_node('MPC_Depth_controller')
    
    global onOFF
    global dead
    global heading_demand
    global compass
    cur_compass = compass()
    onOFF       = False
    heading_demand = 0
    DR = dead_reckoner()
    
    rospy.Subscriber('heading_demand', Float32, heading_demand_cb)
    rospy.Subscriber('sway_demand', Float32, sway_demand_cb)
    rospy.Subscriber('dead_reckoner', dead_reckoner, dead_reckoner_cb)
    rospy.Subscriber('Heading_onOFF', Bool, onOff_cb)
    rospy.Subscriber('compass_out', compass, compass_cb)
    
    pub_tsl  = rospy.Publisher('TSL_setpoints_horizontal', tsl_setpoints)
    pub_tail = rospy.Publisher('tail_setpoints_vertical', tail_setpoints)
    pub_C    = rospy.Publisher('Heading_MPC_values', heading_MPC)
    
    rospy.loginfo("Depth controller online")

    main_control_loop()
