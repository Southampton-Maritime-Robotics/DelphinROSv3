#!/usr/bin/python
import roslib; roslib.load_manifest('DelphinROSv2')
import rospy
import serial
import time
import numpy as np
import scipy 

#from matplotlib.pylab import *
#from control.matlab import *
#from scipy import linalg
#from math  import *
#from numpy import hstack,vstack,pi
#from scipy import zeros,ones,eye,mat,shape,size,size, \
#    arange,real,poly,array,diag
#from scipy.linalg import det,inv,expm,eig,eigvals,logm
#import numpy as np
#import scipy as sp
from yottalab import *


from DelphinROSv2.msg import tsl_setpoints
from DelphinROSv2.msg import tail_setpoints
from DelphinROSv2.msg import position
from DelphinROSv2.msg import compass
from DelphinROSv2.msg import depthandpitch_MPC
from std_msgs.msg import Float32
from std_msgs.msg import Bool


################################################################################
#### CONTROLLER PARAMETERS #####################################################
################################################################################

def initialisation():                                                               # Set controller parameters and build model
    global M
    global gamma0
    global C
    global Model
    global iter
    global gain

    #### MODEL VALUES ####
    massZ         = 167.5                                                     # Mass (inc added) 
    massBG        = 55.0
    I             = 70.0
    Ltf           =-0.55
    Ltr           = 0.49
    G             = massBG*-0.011*9.81
    
    HDz  = -0.0862 * 0.5*1000.0*0.08**(2.0/3.0)
    HDp  = -0.01303 * 0.5*1000.0*0.08**(2.0/3.0)
    
    
    #####################

    #### CONTROLLER VALUES ####
    iter          = 30                                                        # Maximum iterations of Hildreth algorithm
    Np            = 100                                                       # Length of prediction horizon
    Nc            = 8                                                         # Length of control horizon
    delta_t       = 0.1                                                       # Period of controller
    gain          = 12                                                       # Controller gain
    #####################
    
    #### CONSTRAINTS ####
    dtMax         =  2                                                        # Max change in thrust per control loop (N)
    dtMin         = -2                                                        # Min change in thrust per control loop (N)
    Tmax          =  10                                                        # Max allowable thrust (N)
    Tmin          =  0.7                                                      # Min allowable thrust (N)
    Thrust_Smax   =  2300                                                     # Max thruster speed (rpm)
    Thrust_Smin   =  0                                                        # Min thruster speed (rpm)
    #####################
    
    #### DEPTH MODEL ####                                                       # State space model (2 DoF)
    Ac = np.matrix([[0, 1, 0, 0],
                    [0, HDz/massZ, 0, 0],
                    [0, 0, 0, 1],
                    [0, 0, G/I, HDp/I]])
                    
    Bc = np.matrix([[0, 0], 
                    [1.0/massZ, 1.0/massZ],
                    [0, 0],
                    [Ltf/I, Ltr/I]])                    
                    
    Cc = np.matrix([[1.0, 0, 0, 0], [0, 0, 1.0, 0]])
    Dc = np.matrix([[0, 0], [0, 0]])
    
    
    SYS = tuple([Ac, Bc, Cc, Dc])
    [Ad, Bd, Cd, Dd] = cont2discrete(SYS,delta_t)                             # Convert continuous SS model to discrete
    [rd,cd] = np.shape(Bd)

    [Phi_Phi, Phi_F, Phi_R, A_e, B_e, C_e] = mpc_gain(Np, Nc, Ad, Bd, Cd, Dd)
    
    
    #### INITIAL CONDITIONS ####
    time_zero = time.time()                                                     # Create time zero
    [n,n_in]  = np.shape(B_e)
    xm        = np.zeros([4,1],float)                                           # Initialise states
    Xf        = np.zeros([6,1],float)                                            
    u         = np.matrix([[0.0],[0.0]])                                        # Initialise input values
    
    E = Phi_Phi + np.dot(gain,np.eye(Nc*n_in,Nc*n_in))                             # Calculate E matrix
        
    #### CONSTRAINTS ####
    [rd,cd] = np.shape(Bc) 
    gamma0 = np.transpose(np.matrix([dtMax, dtMax, -dtMin, -dtMin, Tmax, Tmax, -Tmin, -Tmin])) # Itialise gamma for controlling constaints
    M = np.zeros([np.size(gamma0),cd*Nc],float)
    M[:,0:2] = np.matrix([[1, 0],[0, 1],[-1, 0],[0, -1],[1, 0],[0, 1],[-1, 0],[0, -1]]) # M matrix used for defining constaints
    
    #### OBSERVER ####
    Q = np.eye(6,6,dtype = 'float')
    R = 50.0*np.eye(2,2,dtype = 'float')     
    [K, So, Eo] = dlqr(np.transpose(A_e), np.transpose(C_e), Q, R)
    K_ob           = np.transpose(K)
  
    return [Phi_Phi, Phi_F, Phi_R, A_e, B_e, C_e, xm, Xf, u, E, M, gamma0, time_zero, delta_t, Np, Nc, gain, K_ob]
    #####################

################################################################################
########## MAIN CONTROL LOOP ###################################################
################################################################################

def main_control_loop():
    zzz = time.time()
    #### SETUP ####
    global compass
    global onOFF
    global depth_demand 
    global pitch_demand
    global depth 
    global pitch
    
    pitch_demand   = 0.0                                                      # Initialise pitch demand to zero
    depth_demand   = 0.0                                                      # Initialise depth demand to zero
    
    onOFF          = False                                                    # Initialise controller onOff to OFF
    
    [Phi_Phi, Phi_F, Phi_R, A_e, B_e, C_e, xm,\
         Xf, u, E, M, gamma0, time_zero, delta_t, Np, Nc, gain, K_ob] = initialisation()                                                                # Set parameters and build model (see above)
    
    print 'Time = ',time.time() - zzz
    pitch_old = 0
    
################################################################################
################################################################################        
    while not rospy.is_shutdown():
        dt = time.time() - time_zero                                            # Calculate time since last calculation

        if dt >= delta_t and onOFF == True:                                    # If dt >= controller period and controller is on then proceed
            
            depth = compass.depth_filt
            pitch = compass.pitch_filt
            
            time_zero = time.time()                                             # Set time zero
            demands   = np.matrix([[depth_demand],[pitch_demand*np.pi/180.0]])              # Set demands 
            
            ### Depth controller ###############################################
            F = -(np.dot(Phi_R,demands) - np.dot(Phi_F,Xf))                     # Calculate F matrix
            gamma = gamma0 + np.matrix([[0], [0], [0], [0], [-u[0]], [-u[1]], [u[0]], [u[1]]]) # Calculate gamma for controlling constaints

            [Delta_u, km] = QPhild(E,F,M,gamma,time_zero, delta_t)            # Find optimum control horizon within constaints
            
            dT0 = float(Delta_u[0,0])                                           # Change in thrust for front thruster
            dT1 = float(Delta_u[1,0])                                           # Change in thrust for rear thruster
            u[0]  = T0  = float(u[0] + dT0)                                     # Add delta_thrust to previous thrust value
            u[1]  = T1  = float(u[1] + dT1)                                     # Add delta_thrust to previous thrust value
            
            ### Observer #######################################################
            y  = np.matrix([[depth],[pitch*np.pi/180.0]])              # Set demands 
            
            print 'y ',y
            print 'demands',demands
            
            Xf = np.dot(A_e,Xf) + np.dot(K_ob, (y - np.dot(C_e, Xf))) + np.dot(B_e, Delta_u[0:2])
            print 'Xf = ',Xf
            print 'u = ',u
            print '############################'        
                     
            ##################################################################### Convert thrust values to thruster setpoints within hard limits
            thruster0 =  int(limits((1.13*np.sign(T0)*(60*(np.abs(T0)/(1000*0.46*0.07**4))**0.5)), 0.0, 2300.0))
            thruster1 =  int(limits((1.13*np.sign(T1)*(60*(np.abs(T1)/(1000*0.46*0.07**4))**0.5)), 0.0, 2300.0))
            ####################################################################
            
            ####################################################################
            calc_time = time.time() - time_zero                               # Calculate time taken to do calculations
            print 'Calc time = ',calc_time
            ####################################################################

            ### Publish values #################################################
            pub_tsl.publish(thruster0, thruster1)   # Publish thruster setpoints
            
            pub_C.publish(onOFF = onOFF, depth = depth, depth_demand = depth_demand, pitch = pitch, pitch_demand = pitch_demand, dT0 = dT0, dT1 = dT1, T0 = T0, T1 = T1, thruster0 = thruster0, thruster1 = thruster1, gain = gain, Np = Np, Nc = Nc, delta_t = delta_t, calc_time = calc_time, km = km, xf1 = Xf[0], xf2 = Xf[1], xf3 = Xf[2], xf4 = Xf[3], xf5 = Xf[4], xf6 = Xf[5])  
            ####################################################################
            
                        
######## END OF LOW LEVEL CONTROLLER ###########################################
################################################################################
################################################################################

################################################################################
################################################################################
def QPhild(E,F,M,gamma,time_zero, delta_t):                                                        # Find optimum control horizon within constraints
    
    km      = 0                                                                 # Hildreth iterations equals zero
    [n1,m1] = np.shape(M)
    eta     = -np.dot(np.linalg.inv(E),F)                                       # Find optimum control horizon
    
    kk = 0                                                                      # Number of constaint violations set to zero
    i  = 0
   
    while i < n1:                                                               # Loop through each constaint line
        if np.dot(M[i,:],eta) > gamma[i]:                                       # If optimum solution is exceeds constraints...
            kk = kk + 1                                                         # Increase count of constain violations
            i = i + 1
        else:                                                                   # Do nothing then proceed
            kk = kk + 0
            i = i + 1
            
    if kk == 0:                                                                 # If constaints have not been violated with optimum solution then return
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

        if (al < 10e-8) or ((time.time() - time_zero) > (delta_t - 0.01)):                                                          # Check convergance criterium and if within limit break
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
    P = np.zeros([Np*m1,m1],float)
    F = np.zeros([Np*m1,n1 + m1],float)
    
    P[0:m1,0:m1] = np.dot(C_e,B_e)
    F[0:m1,0:n]  = np.dot(C_e,A_e)
    
    kk = 1
    while kk < Np:
        F[kk*m1:(kk+1)*m1,:] = np.dot(F[(kk-1)*m1:kk*m1,:],A_e)        
        P[kk*m1:(kk+1)*m1,:] = np.dot(F[(kk-1)*m1:kk*m1,:],B_e)
        kk = kk+1
    
    Phi = np.zeros([Np*m1,Nc*m1],float)
    Phi[:,0:m1] = P
    
    i = 1
    while i < Nc:
        Phi[0:i*m1,i*m1:(i+1)*m1] = np.zeros([i*m1,m1],float)
        Phi[i*m1:Np*m1,i*m1:(i+1)*m1] = P[0:Np*m1-i*m1,0:m1]
        i = i + 1    
    
    Phi_Phi = np.dot(np.transpose(Phi),Phi)
    Phi_F = np.dot(np.transpose(Phi),F)
    
    [mF,nF] = np.shape(Phi_F)
    Phi_R   = Phi_F[:,4:6]
    
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
        ms = expm(dt * em)

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

################################################################################
################################################################################

def depth_demand_cb(depthd):
    global depth_demand
    depth_demand = depthd.data
    
def pitch_demand_cb(pitchd):
    global pitch_demand
    pitch_demand = pitchd.data

def compass_cb(data):
    global compass
    compass  = data 

def depth_onOff_cb(onOff):
    global onOFF
    print 'recieved'
    onOFF = onOff.data


################################################################################
#### INITIALISATION ############################################################
################################################################################

if __name__ == '__main__':

    rospy.init_node('MPC_Depth_controller')
    
    global depth_onOff
    global compass

    
    rospy.Subscriber('depth_demand', Float32, depth_demand_cb)
    rospy.Subscriber('pitch_demand', Float32, pitch_demand_cb)
    rospy.Subscriber('compass_out', compass, compass_cb)
    rospy.Subscriber('Depth_onOFF', Bool, depth_onOff_cb)
    
    
    pub_tsl  = rospy.Publisher('TSL_setpoints_vertical', tsl_setpoints)
    pub_C    = rospy.Publisher('DepthandPitch_MPC_values', depthandpitch_MPC)
    
    rospy.loginfo("Depth controller online")

    main_control_loop()
