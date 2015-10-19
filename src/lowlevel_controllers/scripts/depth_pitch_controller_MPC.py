#!/usr/bin/python

###
import rospy
import serial
import time
import numpy as np
import scipy
from scipy import linalg
import math
from lowlevel_controllers.yottalab import *

from math import *
from cvxopt import matrix,solvers
import cacu_cons
import auv_model_low
import auv_model_high
import auv_model_mul

##########
from hardware_interfaces.msg    import tsl_setpoints
from hardware_interfaces.msg    import tail_setpoints
from hardware_interfaces.msg    import compass
from hardware_interfaces.msg    import depth
from lowlevel_controllers.msg   import depthandspeed_MPC
from std_msgs.msg               import Float32
from std_msgs.msg               import Int8
from std_msgs.msg               import Bool
from std_msgs.msg               import String
from hardware_interfaces.msg import status

def main_control_loop():

    global speed_demand
    global depth_demand

    global controller_onOff
    global DPC
    
    global timeLastDemandMax
    global timeLastCallback


    speed_demand = 0.
    depth_demand = 0.

    controller_onOff = Bool()
    controlRate = 5. # [Hz]
    r = rospy.Rate(controlRate)
    DPC.delta_t = delta_t = controlPeriod = 1/controlRate # [sec]

    solvers.options['show_progress'] = False
    solvers.options['maxiters'] =100


    DPC.Np = Np = 60#60
    DPC.Nc = Nc = 10#8
    Thrust_Smin = 0
    Thrust_Smax = 2500
    start_flag = 1 ######new################
    switch_flag = 0.6
    speed_flag = 0

    old_set_velocity = set_velocity = speed_demand

    ###########build gain matrix##################
    inputs_high = 2
    inputs_low = 3
    inputs_combine  = 4
    
    gain_matrix_combine = np.zeros([inputs_combine*Nc,inputs_combine*Nc],dtype = np.float)
    gain_matrix_combine[0,0] = 0.002
    gain_matrix_combine[1,1] = 0.002
    gain_matrix_combine[2,2] =0.002
    gain_matrix_combine[3,3] = 0.002
        
    gain_matrix_high = np.zeros([inputs_high*Nc,inputs_high*Nc],dtype = np.float)
    gain_matrix_high[0,0] = 0.002
    gain_matrix_high[1,1] =0.002

    gain_matrix_low = np.zeros([inputs_low*Nc,inputs_low*Nc],dtype = np.float)
    gain_matrix_low[0,0] = 0.002
    gain_matrix_low[1,1] = 0.002
    gain_matrix_low[2,2] = 0.002

    for i in xrange(Nc):
        gain_matrix_low[i*inputs_low:i*inputs_low+inputs_low,
                        i*inputs_low:i*inputs_low+inputs_low] = gain_matrix_low[0:inputs_low,0:inputs_low]
        gain_matrix_high[i*inputs_high:i*inputs_high+inputs_high,
                         i*inputs_high:i*inputs_high+inputs_high] = gain_matrix_high[0:inputs_high,0:inputs_high]
        gain_matrix_combine[i*inputs_combine:i*inputs_combine+inputs_combine,
                            i*inputs_combine:i*inputs_combine+inputs_combine] = gain_matrix_combine[0:inputs_combine,0:inputs_combine]
        
   ########### initial conditions############

    [velX,velZ,velP]=[0,0,0]
    [pitch,depth] = [0,0]
    [prop,delta,T0,T1] = [0,0,0,0]
    P = 0.0
   ########build augmented model############

    [Phi_Phi_low,Phi_F_low,Phi_R_low,n_low,E_low,A_o_low,
     B_o_low, C_o_low, K_ob_low]=LowSpeed(delta,velX,P,Nc,Np,
                                          gain_matrix_low,prop,T0,T1)
    [Phi_Phi_high,Phi_F_high,Phi_R_high,n_high,E_high,A_o_high,
     B_o_high, C_o_high, K_ob_high]=HighSpeed(delta,velX,P,Nc,
                                              Np,gain_matrix_high,prop)
    [Phi_Phi,Phi_F,Phi_R,n_combine,E,A_o, B_o,
     C_o, K_ob]=CombineJ(delta,velX,P,Nc,Np,gain_matrix_combine,prop,T0,T1)
    
   ######initialse############
    Xf = np.zeros([n_combine,1],float)     
    Xfo = np.zeros([n_combine+1,1],float)    
    Xf_low = np.zeros([n_low,1],float)
    Xfo_low = np.zeros([n_low+1,1],float)
    Xf_high = np.zeros([n_high,1],float)
    Xfo_high = np.zeros([n_high+1,1],float)
    
    u = np.matrix([[0.0],[0.0],[0.0],[0.0]])
    u_low = np.matrix([[0.0],[0.0],[0.0]])
    u_high = np.matrix([[0.0],[0.0]])
    
    Delta_u = np.matrix([[0.0],[0.0],[0.0],[0.0]])
    Delta_u_high = np.matrix([[0.0],[0.0]])
    Delta_u_low = np.matrix([[0.0],[0.0],[0.0]])
    
    if speed_demand < switch_flag:
        demands = np.matrix([[speed_demand],[depth_demand],[0.0]])
    else:
#        demands = np.matrix([[speed_demand],[depth_demand],[0.02*depth_demand/(speed_demand)**0.5]])
        demands = np.matrix([[speed_demand],[depth_demand],[0.0]])

    ###set contraints######
    [M_low,gamma_low]=conMatricesnew1(Nc,u_low,velX,0);
    [M_high,gamma_high]=conMatricesnew1(Nc,u_high,velX,1);
    [M,gamma]=conMatricesnew1(Nc,u,velX,-1);
   ########################
   
    timeLastDemandMax = 1 # [sec] if there is no onOff flag updated within this many seconds, controller will be turnned off#problem
    timeLastCallback = time.time()
    
    propSP = 0
    speed = 0
    while not rospy.is_shutdown():

        pubStatus.publish(nodeID = 8, status = True)    ####
        
        timeRef = time.time()

        propDemand = propSP
        speed_current = speedObserver(propDemand, speed, controlPeriod)
        speed = speed_current
        DPC.speed = speed
        
        if controller_onOff == True:    
            
            depth = DPC.depth # depth filtered by PT_filter in compass_oceanserver.py
            depth_demand = round(DPC.depth_demand,2)
                      
            velX = DPC.speed
            oldvelX = velX
            
            #test

            if start_flag == 1:  ####new#################              
                old_set_velocity = set_velocity = speed_demand = round(DPC.speed_demand,2)
                start_flag = 0
            
            if speed_demand != round(DPC.speed_demand,2):
                old_set_velocity = set_velocity
                set_velocity = speed_demand = round(DPC.speed_demand,2)
                
            
            pitch = DPC.pitch # pitch angle measured by xsens
            velP = DPC.pitch_speed  ####
            

            if speed_demand < switch_flag:
                demands = np.matrix([[speed_demand],[depth_demand],[0.0]])
                if speed_demand > 0:
                    gain_matrix_low[1,1] = 8.0#8.0#20.0
                    gain_matrix_low[2,2] = 8.0#8.0#20.0
            else:
                demands = np.matrix([[speed_demand],[depth_demand],[0.01*depth_demand/(speed_demand)**0.5]])
                
#if  speed_demand > 0.3:
#     speed_flag = 1
      
                #demands = np.matrix([[speed_demand],[depth_demand],[0.0]])
            #if round(speed_demand,2) == 0.0:
            #    velX = 0.5

            if old_set_velocity != set_velocity:
                if set_velocity >= switch_flag:
                    if old_set_velocity >= switch_flag:
                        u_high[0] = u[0]
                        u_high[1] = u[1]
                        Delta_u_high[0,0] = Delta_u[0,0]
                        Delta_u_high[1,0] = Delta_u[1,0]
                    
                        [Phi_Phi_high,Phi_F_high,Phi_R_high,n_high,E_high,A_o_high, B_o_high, C_o_high, K_ob_high]=HighSpeed(delta,velX,P,Nc,Np,gain_matrix_high,prop)                   
                        [M_high,gamma_high]=conMatricesnew1(Nc,u_high,velX,1)
                        [Delta_u_high]= GetU( Phi_R_high,Phi_F_high,Xf_high,E_high,M_high,gamma_high,demands,Delta_u_high)
                    
                        DPC.dprop = Delta_u[0,0] = dprop = float(Delta_u_high[0,0])
                        DPC.ddelta = Delta_u[1,0] = ddelta = float(Delta_u_high[1,0])
                        DPC.dT0 = Delta_u[2,0] = dT0 = 0.0
                        DPC.dT1 = Delta_u[3,0] = dT1 = 0.0

                        u[0] = float(u[0] + dprop)
                        u[1] = float(u[1] + ddelta)
                        u[2] = float(u[2] + dT0)
                        u[3] = float(u[3] + dT1)

                        DPC.prop = prop = round(u[0],1) 
                        DPC.delta = delta = round(u[1],1) 
                        DPC.T0 = T0 = round(u[2],1)
                        DPC.T1 = T1 = round(u[3],1)
                        flag_cp = 1
                    else:
                    
                        [Phi_Phi,Phi_F,Phi_R,n_combine,E,A_o, B_o, C_o, K_ob]=CombineJ(delta,velX,P,Nc,Np,gain_matrix_combine,prop,T0,T1)#
                        [M,gamma]=conMatricesnew1(Nc,u,velX,-1)
                        [Delta_u]= GetU( Phi_R,Phi_F,Xf,E,M,gamma,demands,Delta_u)

                        DPC.dprop = dprop = float(Delta_u[0,0])
                        DPC.ddelta = ddelta = float(Delta_u[1,0])
                        DPC.dT0 = dT0 = float(Delta_u[2,0])
                        DPC.dT1 = dT1 = float(Delta_u[3,0])

                        u[0] = float(u[0] + dprop)
                        u[1] = float(u[1] + ddelta)
                        u[2] = float(u[2] + dT0)
                        u[3] = float(u[3] + dT1)

                        DPC.prop = prop = round(u[0],1) 
                        DPC.delta = delta = round(u[1],1) 
                        DPC.T0 = T0 = round(u[2],1)
                        DPC.T1 = T1 = round(u[3],1)
                        
                        flag_cp = -1
                else:
                    if old_set_velocity < switch_flag:
                        u_low[0] = u[0]
                        u_low[1] = u[2]
                        u_low[2] = u[3]
                        Delta_u_low[0,0] = Delta_u[0,0]
                        Delta_u_low[1,0] = Delta_u[2,0]
                        Delta_u_low[2,0] = Delta_u[3,0]
                        [Phi_Phi_low,Phi_F_low,Phi_R_low,n_low,E_low,A_o_low, B_o_low, C_o_low, K_ob_low]=LowSpeed(delta,velX,P,Nc,Np,gain_matrix_low,prop,T0,T1)
                    
                        [M_low,gamma_low]=conMatricesnew1(Nc,u_low,velX,0)
                        [Delta_u_low]= GetU( Phi_R_low,Phi_F_low,Xf_low,E_low,M_low,gamma_low,demands,Delta_u_low)                    
              
                        DPC.dprop = Delta_u[0,0] = dprop = float(Delta_u_low[0,0])
                        DPC.ddelta = Delta_u[1,0] = ddelta = 0.0
                        DPC.dT0 = Delta_u[2,0] = dT0 = float(Delta_u_low[1,0])
                        DPC.dT1 = Delta_u[3,0] = dT1 = float(Delta_u_low[2,0])

                        u[0] = float(u[0] + dprop)
                        u[1] = float(u[1] + ddelta)
                        u[2] = float(u[2] + dT0)
                        u[3] = float(u[3] + dT1)

                        DPC.prop = prop = round(u[0],1) 
                        DPC.delta = delta = round(u[1],1) 
                        DPC.T0 = T0 = round(u[2],1)
                        DPC.T1 = T1 = round(u[3],1)
                        flag_cp = 0
                    else:
                        [Phi_Phi,Phi_F,Phi_R,n_combine,E,A_o, B_o, C_o, K_ob]=CombineJ(delta,velX,P,Nc,Np,gain_matrix_combine,prop,T0,T1)#
                        [M,gamma]=conMatricesnew1(Nc,u,velX,-2)
                        [Delta_u]= GetU( Phi_R,Phi_F,Xf,E,M,gamma,demands,Delta_u)

                        DPC.dprop = Delta_u[0,0] = dprop = float(Delta_u[0,0])
                        DPC.ddelta = Delta_u[1,0] = ddelta = float(Delta_u[1,0])
                        DPC.dT0 = Delta_u[2,0] = dT0 = float(Delta_u[2,0])
                        DPC.dT1 = Delta_u[3,0] = dT1 = float(Delta_u[3,0])

                        u[0] = float(u[0] + dprop)
                        u[1] = float(u[1] + ddelta)
                        u[2] = float(u[2] + dT0)
                        u[3] = float(u[3] + dT1)

                        DPC.prop = prop = round(u[0],1) 
                        DPC.delta = delta = round(u[1],1) 
                        DPC.T0 = T0 = round(u[2],1)
                        DPC.T1 = T1 = round(u[3],1)
                        
                        flag_cp = -2
            else:
                if set_velocity >= switch_flag:                    
                    u_high[0] = u[0]
                    u_high[1] = u[1]
                    Delta_u_high[0,0] = Delta_u[0,0]
                    Delta_u_high[1,0] = Delta_u[1,0]

                    [Phi_Phi_high,Phi_F_high,Phi_R_high,n_high,E_high,A_o_high, B_o_high, C_o_high, K_ob_high]=HighSpeed(delta,velX,P,Nc,Np,gain_matrix_high,prop)
                    [M_high,gamma_high]=conMatricesnew1(Nc,u_high,velX,1)
                
                    [Delta_u_high]= GetU( Phi_R_high,Phi_F_high,Xf_high,E_high,M_high,gamma_high,demands,Delta_u_high)
   
                    DPC.dprop = Delta_u[0,0] = dprop =float (Delta_u_high[0,0])
                    DPC.ddelta = Delta_u[1,0] = ddelta = float(Delta_u_high[1,0])
                    DPC.dT0 = Delta_u[2,0] = dT0 = 0.0
                    DPC.dT1 = Delta_u[3,0] = dT1 = 0.0

                    u[0] = float(u[0] + dprop)
                    u[1] = float(u[1] + ddelta)
                    u[2] = float(u[2] + dT0)
                    u[3] = float(u[3] + dT1)

                    DPC.prop = prop = round(u[0],1) 
                    DPC.delta = delta = round(u[1],1) 
                    DPC.T0 = T0 = round(u[2],1)
                    DPC.T1 = T1 = round(u[3],1)
                    
                    flag_cp = 1
                
                else:   #
                    u_low[0] = u[0]
                    u_low[1] = u[2]
                    u_low[2] = u[3]
                    [Phi_Phi_low,Phi_F_low,Phi_R_low,n_low,E_low,A_o_low, B_o_low,
                     C_o_low, K_ob_low]=LowSpeed(delta,velX,P,Nc,Np,gain_matrix_low,prop,T0,T1)
                    
                    [M_low,gamma_low]=conMatricesnew1(Nc,u_low,velX,0)
                    
               
                    [Delta_u_low]= GetU( Phi_R_low,Phi_F_low,Xf_low,E_low,M_low,gamma_low,demands,Delta_u_low)                    
            
                    DPC.dprop = Delta_u[0,0] = dprop = float(Delta_u_low[0,0])
                    DPC.ddelta = Delta_u[1,0] = ddelta = 0.0
                    DPC.dT0 = Delta_u[2,0] = dT0 = float(Delta_u_low[1,0])
                    DPC.dT1 = Delta_u[3,0] = dT1 = float(Delta_u_low[2,0])

                    u[0] = float(u[0] + dprop)
                    u[1] = float(u[1] + ddelta)
                    u[2] = float(u[2] + dT0)
                    u[3] = float(u[3] + dT1)

                    DPC.prop = prop = round(u[0],3) 
                    DPC.delta = delta = round(u[1],3) 
                    DPC.T0 = T0 = round(u[2],3)
                    DPC.T1 = T1 = round(u[3],3)
                    
                    flag_cp = 0
        
            if flag_cp == 1:
                y_high   = [[velX],[depth],[pitch*np.pi/180.0],[velP]]

                Xfo_high = np.dot(A_o_high,Xfo_high) + np.dot(K_ob_high,(y_high - np.dot(C_o_high,Xfo_high)))+ np.dot(B_o_high,Delta_u_high[0:2])
                Xf_high  = Xfo_high[0:n_high]
            
                Xfo = Xfo_high
                Xf  = Xf_high
            else:
                if flag_cp==0:
                    y_low   = [[velX],[depth],[pitch*np.pi/180.0],[velP]]
                    Xfo_low = np.dot(A_o_low,Xfo_low) + np.dot(K_ob_low,(y_low - np.dot(C_o_low,Xfo_low)))+ np.dot(B_o_low,Delta_u_low[0:3])
                    Xf_low  = Xfo_low[0:n_low]

                    Xfo = Xfo_low
                    Xf  = Xf_low
                else:
                    if flag_cp== -1:
                        y   = [[velX],[depth],[pitch*np.pi/180.0],[velP]]
                        Xfo = np.dot(A_o,Xfo) + np.dot(K_ob,(y - np.dot(C_o,Xfo)))+ np.dot(B_o,Delta_u[0:4])
                        Xf  = Xfo[0:n_combine]
                        Xfo_high =  Xfo
                        Xf_high = Xf
                        Xfo_low =  Xfo
                        Xf_low = Xf
                    else:
                        if flag_cp == -2:
                            y   = [[velX],[depth],[pitch*np.pi/180.0],[velP]]
                            Xfo = np.dot(A_o,Xfo) + np.dot(K_ob,(y - np.dot(C_o,Xfo)))+ np.dot(B_o,Delta_u[0:4])
                            Xf  = Xfo[0:n_combine]
                            Xfo_high =  Xfo
                            Xf_high = Xf
                            Xfo_low =  Xfo
                            Xf_low = Xf

#######problem?????
            #if velX > 0.4 and T0<1.25:
            #    DPC.thruster0 = thruster0 = 0
            #else:
            #    DPC.thruster0 = thruster0 = int(limits((1.13*np.sign(T0)*(60.0*(np.abs(T0)/(1000*0.46*0.07**4))**0.5)),Thrust_Smin,Thrust_Smax))
            #if velX > 0.4 and T1<1.25:
            #    DPC.thruster1 = thruster1= 0
            #else:
            #    DPC.thruster1 = thruster1 = int(limits((1.13*np.sign(T0)*(60.0*(np.abs(T0)/(1000*0.46*0.07**4))**0.5)),Thrust_Smin,Thrust_Smax))
            #print T0, round(T0,2)
            #print T1
            if round(T0,2) >= 0.7:#0.7
                DPC.thruster0 = thruster0 = int(limits((1.13*np.sign(T0)*(60.0*(np.abs(T0)/(1000*0.46*0.07**4))**0.5)),Thrust_Smin,Thrust_Smax))
            else:
                DPC.thruster0 = thruster0 = 0
            if round(T1,2) >= 0.7:#0.7
                DPC.thruster1 = thruster1 = int(limits((1.13*np.sign(T1)*(60.0*(np.abs(T1)/(1000*0.46*0.07**4))**0.5)),Thrust_Smin,Thrust_Smax))
            else:
                DPC.thruster1 = thruster1 = 0               

            if (round(prop,2) >2.0): ##and speed_demand > 0.1:###new
                DPC.propSP = propSP = int(round((abs(prop)+13.9434)/1.5116,2))#######
            else:
                DPC.propSP = propSP = 0
                
            if propSP > 22:
                DPC.propSP = propSP = 22
            if propSP < -22:
                DPC.propSP = propSP = -22
            

            #publish values
            pub_tsl.publish(thruster0 = thruster0,thruster1 = thruster1)
            if round(speed_demand,2) == 0.0:
               u[1] =  0.0
            
            pub_tail.publish(cs0 = -u[1],cs1 = -u[1])
            
            
            pub_prop.publish(propSP)

            DPC.onOFF = controller_onOff            
            
            DPC.Xf1 = Xfo[0,0]
            DPC.Xf2 = Xfo[1,0]
            DPC.Xf3 = Xfo[2,0]
            DPC.Xf4 = Xfo[3,0]
            DPC.Xf5 = Xfo[4,0]
            DPC.Xf6 = Xfo[5,0]
            DPC.Xf7 = Xfo[6,0]
            DPC.Xf8 = Xfo[7,0]
            
            pub_DPC.publish(DPC)
            
        if time.time()-timeLastCallback > timeLastDemandMax:
            controller_onOff = False
            
        timeElapse = time.time()-timeRef

        if timeElapse < controlPeriod:
            r.sleep()
        else:
            str = "Pitch-Depth control rate does not meet the desired value of %.2fHz: actual control rate is %.2fHz" %(controlRate,1/timeElapse)
            rospy.logwarn(str)
            pubMissionLog.publish(str)
                

##########################################################################
def CombineJ(delta,velX,P,Nc,Np,gain,prop,T0,T1):

    ##fixed model values
    massZ = 167.5
    massX = 85.0
    massBG = 55.0
    I = 70.00

    volume = 0.08
    Ltf = -0.55
    Ltr = 0.49
    Lf = 1.0
    L = 2.0
    BG = -0.011
    rho = 1000.0
    G = massBG*9.81*BG
    ##################

    #####veriable model values
    ##foils
    fCl = 0.0001574#0.0 #0.0001574
    fCd = 3.351e-5*abs(delta)**2
    fCm = 7.563e-5

    ##Hull
    hCl = 0.001512
    hCd = 0.07265
    hCm = 0.0001596
    hCdp = -0.01303
    hCdz = -0.0862

    #Thruster
    Told = T0 + T1
    T_coeff = 1.0/exp(abs(velX))

    ##updated coefficients
    ##foilsDelta_u
    FL = -0.5*rho*(L**2)*fCl*velX*abs(velX)
    FDx = -0.5*rho*(volume**(2.0/3.0))*fCd*abs(velX)
    FM = -0.5*rho*(L**3.0)*fCm*velX*abs(velX)

    #hull
    HL = -0.5*rho*(L**2.0)*(hCl)*velX*abs(velX)*180.0/np.pi
    HDx = -0.5*rho*(volume**(2.0/3.0))*(hCd)*abs(velX)
    HM = 0.5*rho*(L**3.0)*(hCm)*velX*abs(velX)*180.0/np.pi
#    HDp = (hCdp * 0.5*rho*L**2) + (hCdp * 0.5*rho*rho*L**2)*abs(velX)*0.0####
#    HDz = (hCdz * 0.5*rho*volume**(2.0/3.0)) + (hCdp * 0.5*rho*volume**(2.0/3.0))*abs(velX)*0.0##
    HDp = (hCdp * 0.5*rho*volume**(2.0/3.0)) + (hCdp * 0.5*rho*volume**(2.0/3.0))*abs(velX)*20
    HDz = (hCdz * 0.5*rho*volume**(2.0/3.0)) + (hCdz * 0.5*rho*volume**(2.0/3.0))*abs(velX)*20##

    #state space model
  #state space model
    [Phi_Phi,Phi_F,Phi_R,n,E,Ad, Bd, Co] = auv_model_mul.mpc_model_mul(HDx,FDx,massX,Told,HDz,massZ,
                                                                       prop,HL,HM,G,I,HDp ,FL,FM,T_coeff,Ltf,Ltr,
                                                                       Np,Nc,gain)
    [A_o,B_o,C_o,K_ob] = obsv_matrices(Ad,Bd,Co)
    
    return [Phi_Phi,Phi_F,Phi_R,n,E,A_o, B_o, C_o, K_ob]




##########################################################################
def HighSpeed(delta,velX,P,Nc,Np,gain,prop):

    ##fixed model values
    massZ = 167.5
    massX = 85.0
    massBG = 55.0
    I = 70.00

    volume = 0.08
    Ltf = -0.55
    Ltr = 0.49
    Lf = 1.0
    L = 2.0
    BG = -0.011
    rho = 1000.0
    G = massBG*9.81*BG
    ##################

    #####veriable model values
    ##foils
    fCl = 0.0#0.0001574
    fCd = 3.351e-5*abs(delta)**2
    fCm = 7.563e-5

    ##Hull
    hCl = 0.001512
    hCd = 0.07265
    hCm = 0.0001596
    hCdp = -0.01303
    hCdz = -0.0862


    ##updated coefficients
    ##foils
    temp_x = abs(velX)
    FL = -0.5*rho*(L**2)*fCl*velX* abs(velX)
    FDx = -0.5*rho*(volume**(2.0/3.0))*fCd* abs(velX)
    FM = -0.5*rho*(L**3.0)*fCm*velX*abs(velX)
    #hull
    HL = -0.5*rho*(L**2.0)*(hCl)*velX* abs(velX)*180.0/np.pi
    HDx = -0.5*rho*(volume**(2.0/3.0))*(hCd)* abs(velX)
    HM = 0.5*rho*(L**3.0)*(hCm)*velX* abs(velX)*180.0/np.pi
   # HDp = (hCdp * 0.5*rho*L**2) + (hCdp * 0.5*rho*rho*L**2)*abs(velX)*0.0####
    HDp = (hCdp * 0.5*rho*volume**(2.0/3.0)) + (hCdp * 0.5*rho*volume**(2.0/3.0))* abs(velX)*0
    HDz = (hCdz * 0.5*rho*volume**(2.0/3.0)) + (hCdz * 0.5*rho*volume**(2.0/3.0))* abs(velX)*0##

    [Phi_Phi,Phi_F,Phi_R,n,E,Ad, Bd, Co] = auv_model_high.mpc_model_high(HDx,FDx,massX,HDz,massZ,
                                                                         prop,HL,HM,G,I,HDp,FL,FM,Np,Nc,gain)

    [A_o,B_o,C_o,K_ob] = obsv_matrices(Ad,Bd,Co)

    
    return [Phi_Phi,Phi_F,Phi_R,n,E,A_o, B_o, C_o, K_ob]




##########################################################################
def LowSpeed(delta,velX,P,Nc,Np,gain,prop,T0,T1):

    ##fixed model values
    massZ = 167.5
    massX = 85.0
    massBG = 55.0
    I = 70.00

    volume = 0.08
    Ltf = -0.55
    Ltr = 0.49
    Lf = 1.0
    L = 2.0
    BG = -0.011
    rho = 1000.0
    G = massBG*9.81*BG
    ##################

    #####veriable model values
    ##Hull
    hCl = 0.001512
    hCd = 0.07265
    hCm = 0.0001596
    hCdp = -0.01303
    hCdz = -0.0862

    #Thruster
    Told = T0 + T1
    T_coeff = 1.0/exp(abs(velX))

    ##updated coefficients

    #hull
    temp_x = abs(velX)
    HL = -0.5*rho*(L**2.0)*(hCl)*velX*temp_x*180.0/np.pi
    HDx = -0.5*rho*(volume**(2.0/3.0))*(hCd)*temp_x
    HM = 0.5*rho*(L**3.0)*(hCm)*velX*temp_x*180.0/np.pi
    #HDp = (hCdp * 0.5*rho*L**2) + (hCdp * 0.5*rho*rho*L**2)*temp_x*0.0####
    HDp = (hCdp * 0.5*rho*volume**(2.0/3.0))+ (hCdp * 0.5*rho*volume**(2.0/3.0))*temp_x*0##
    HDz = (hCdz * 0.5*rho*volume**(2.0/3.0)) + (hCdz * 0.5*rho*volume**(2.0/3.0))*temp_x*0##

 
    [Phi_Phi,Phi_F,Phi_R,n,E,Ad, Bd, Co] = auv_model_low.mpc_model_low(HDx,massX,Told,HDz,
                                                                                       massZ,prop,HL,HM,G,I,HDp ,T_coeff,Ltf,Ltr,Np,Nc,gain)
    
    [A_o,B_o,C_o,K_ob] = obsv_matrices(Ad,Bd,Co)
   
 
    return [Phi_Phi,Phi_F,Phi_R,n,E,A_o, B_o, C_o, K_ob] 

##############################################################################
def GetU( Phi_R,Phi_F,Xf,E,M,gamma,demands,Delta_u):

    #F = -(np.dot(Phi_R,demands) - np.dot(Phi_F,Xf))
    #E = matrix(E)#
    #F = matrix(-(np.dot(Phi_R,demands) - np.dot(Phi_F,Xf)))
   # M = matrix(M)
    #b = matrix(gamma)
    ##F = -(np.dot(Phi_R,demands) - np.dot(Phi_F,Xf))

    #E = matrix(E)#
    #F = matrix(F)
    #M = matrix(M)
    #b = matrix(gamma)

    #[Delta_u]= QPhild(E,F,M,b,0,0)
   
    try:
        sol = solvers.qp(matrix(E),matrix(-(np.dot(Phi_R,demands) - np.dot(Phi_F,Xf))),matrix(M),matrix(gamma))
        Delta_u = sol['x']
    except:
        Delta_u = Delta_u
        print 'solution failed!'    
    return[Delta_u]
#####################################################################################

##############################################################################
def conMatricesnew1(Nc,u,velX,flag):
    global speed_demand

#    if speed_demand > 0.2:
#        Propmin = 2.0
#        Propmax = 17.8
#    else:
#        Propmax = 0
#        Propmin = 0

    Propmin = 2.0#2.0
    Propmax = 17.8
    dPropmax = 1.0
    dPropmin = -1.0

    Tmax = 10.0
    Tmin = 0.7

    dTmax = 2.0
    dTmin = -2.0

    CSmax = 30.0
    CSmin = -29.0

    dCSmax = 4.0
    dCSmin = -4.0




    if round(speed_demand,2) == 0.0:  #new
        Propmax = 0.0
        Propmin =-17.8
        CSmax = CSmin = 0.0

        #if round(velX,1) <= 0.1: 
        #    Propmax = 0.0
        #    Propmin =0.0
    if round(depth_demand,2) == 0.0:  #new
        CSmax = CSmin = 0.0
        Tmax = Tmin = 0.0
    #constraints = np.matrix([[0.0],[0.0],[0.0],[0.0]])
    #print flag
    if flag == 1:
        constraints= cacu_cons.constraint(Nc,u[0],u[1],velX,flag,0.4,0.6,Propmin,Propmax,dPropmax,dPropmin,Tmax,Tmin,dTmax,dTmin,CSmax,CSmin,dCSmax,dCSmin)
    if flag == 0:
        constraints= cacu_cons.constraint(Nc,u[0],u[1],velX,flag,0.4,0.6,Propmin,Propmax,dPropmax,dPropmin,Tmax,Tmin,dTmax,dTmin,CSmax,CSmin,dCSmax,dCSmin)
    if flag == -1:
        constraints= cacu_cons.constraint(Nc,u[1],u[2],velX,flag,0.4,0.6,Propmin,Propmax,dPropmax,dPropmin,Tmax,Tmin,dTmax,dTmin,CSmax,CSmin,dCSmax,dCSmin)
    if flag == -2:
        constraints= cacu_cons.constraint(Nc,u[1],u[2],velX,flag,0.4,0.6,Propmin,Propmax,dPropmax,dPropmin,Tmax,Tmin,dTmax,dTmin,CSmax,CSmin,dCSmax,dCSmin)

    [i,c] = np.shape(constraints)

    #print constraints
    Mdef = np.matrix([1.0,-1.0,1.0,-1.0])
    M = np.zeros([Nc*i*c,Nc*i])
    gamma = np.zeros([Nc*i*c,1])


    for Un in xrange(i):
        for NC in xrange(Nc):
            rowstart = ((Un+1-1)*(Nc)*c)+((NC+1-1)*c + 1) - 1
            col = (NC+1 - 1)*i + Un+1 - 1
            M[rowstart:rowstart+c,col] = Mdef
            gamma[rowstart:rowstart+c,0:1] = np.diag([1.0,-1.0,1.0,-1.0])*np.transpose(constraints[Un+1-1,:]) +np.matrix([[0],[0],[-u[Un+1-1]],[u[Un+1-1]]])

    return [M,gamma]






#####################################################################################################
def mpc_gain(Np,Nc,Ad,Bd,Cd,Dd):

    [m1,n1] = np.shape(Cd)
    [n1,n_in] = np.shape(Bd)
    A_e = np.eye(n1+m1,n1+m1)
    A_e[0:n1+m1,0:n1] = np.concatenate((Ad, np.dot(Cd,Ad)), axis=0)



    B_e = np.zeros([n1+m1,n_in],float)
    B_e[0:n1+m1,:] = np.concatenate((Bd, np.dot(Cd,Bd)), axis=0)

    C_e = np.zeros([m1,n1+m1],float)
    C_e[:,n1:n1+m1] = np.eye(m1,m1)

    n = n1+m1
    P = np.zeros([Np*m1,n_in],float)
    F = np.zeros([Np*m1,n1 + m1],float)
    P[0:m1,0:n_in] = np.dot(C_e,B_e)
    F[0:m1,0:n] = np.dot(C_e,A_e)


    for kk in xrange(Np-1):
        F[(kk+1)*m1:(kk+2)*m1,:] = np.dot(F[(kk)*m1:(kk+1)*m1,:],A_e)
        P[(kk+1)*m1:(kk+2)*m1,:] = np.dot(F[(kk)*m1:(kk+1)*m1,:],B_e)

    Phi = np.zeros([Np*m1,Nc*n_in],float)
    Phi[:,0:n_in] = P

    for i in xrange(Nc-1):
        Phi[0:Np*m1,(i+1)*n_in:(i+2)*n_in]  = np.concatenate((np.zeros([(i+1)*m1,n_in],float), P[0:Np*m1-(i+1)*m1,0:n_in]), axis=0)

    Phi_Phi = np.dot(np.transpose(Phi),Phi)
    Phi_F = np.dot(np.transpose(Phi),F)

    [mF,nF] = np.shape(Phi_F)
    Phi_R = Phi_F[:,(nF-m1):nF]

    return [Phi_Phi,Phi_F,Phi_R,A_e,B_e,C_e]

#####################################################################
def cont2discrete(sys,dt,method="zoh",alpha=None):

    if len(sys) == 4:
        a,b,c,d = sys
        
    else:
        raise ValueError("First argument must either be a tuple of 4(ss) arrays.")

    if method == 'zoh':
        em_upper = np.hstack((a,b))
        em_lower = np.hstack((np.zeros((b.shape[1],a.shape[0])),

        np.zeros((b.shape[1],b.shape[1]))))#####
        em = np.vstack((em_upper,em_lower))
        ms = linalg.expm(dt * em)

        ms = ms[: a.shape[0],:]
        ad = ms[:,0:a.shape[1]]
        bd = ms[:,a.shape[1]:]
        cd = c
        dd = d
       
    else:
        raise ValueError("Unknown transformation method '%s'" %method)

    return [ad, bd, cd ,dd]
#######################################################################
def obsv_matrices(Ad,Bd,Cd):
    [m1,n1] = np.shape(Cd)
    [n1,n_in] = np.shape(Bd)

    A_e = np.eye(n1+m1,n1+m1)
    
    #A_e = np.concatenate((Ad, np.dot(Cd,Ad)), axis=0)
    A_e[0:n1,0:n1] = Ad
    A_e[n1:n1+m1,0:n1] = np.dot(Cd,Ad)

    B_e = np.zeros([n1+m1,n_in],float)
    B_e[0:n1,0:n1] = Bd
    B_e[n1:n1+m1,0:n1] = np.dot(Cd,Bd)

    C_e = np.zeros([m1,n1+m1],float)
    C_e[:,n1:n1+m1] = np.eye(m1,m1)

    A_o = A_e
    B_o = B_e
    C_o = C_e

    [r,c] = np.shape(C_e)
    Q = np.eye(c,c,dtype = 'float')

    if r == 3:
        R = np.diag([1.0,1.0,1.0])
    if r == 4:
        #R = np.diag([500.0,200.0,200.0,200.0]) 
        R = np.diag([1.0,1.0,1.0,21.0])         
    #R = np.diag([1.0,500.0,20.0])

    [K] = dlqr(np.transpose(A_o),np.transpose(C_o),Q,R)

    K_ob = np.transpose(K)

    return [A_o,B_o,C_o,K_ob]

#######################################################################
#######################################################################
def QPhild(E,F,M,gamma,delta_t,time_zero):
    iter = 100
    km = 0
    [n1,m1] = np.shape(M)
    eta = -np.dot(np.linalg.inv(E),F)

    x = True in (np.dot(M,eta) > gamma)

    if not x:
        return [eta]

    H = np.dot(M,(np.dot(np.linalg.inv(E),np.transpose(M))))
    K = np.dot(M,(np.dot(np.linalg.inv(E),F))) + gamma

    [n,m] = np.shape(K)

    Lambda = np.zeros([n,m],float)
    Lambda_p = np.zeros([n,m],float)

    al = 10

    ################################
    time_zero = time.time()
    while (km<iter) and (al > 10e-8) and ((time.time() - time_zero) < (0.2 - 0.005)):
    #while (km<iter) and (al > 10e-8) :
        Lambda_p[:,0] = Lambda[:,0]
        i = 0

        while i< n:
            w = np.add(np.dot(H[i,:],Lambda),-np.dot(H[i,i],Lambda[i,0]))
            w = np.add(w,K[i,0])
            la = np.divide(-w,H[i,i])

            Lambda[i,0] = np.max([0.0,la])
            i = i+1

        al = np.dot(np.transpose(Lambda - Lambda_p),(Lambda - Lambda_p))

        km = km + 1
        
            #############################
    eta = -np.dot(np.linalg.inv(E),F) -np.dot(np.linalg.inv(E),(np.dot(np.transpose(M),Lambda)))
    return [eta]
#######################################################################

def limits(value,min,max):
    if value < min:
        value = min
    elif value > max:
        value = max
    return value
    
def depth_demand_callback(depthd):
    global DPC
    DPC.depth_demand = depthd.data
    
def speed_demand_callback(speedd):
    global DPC
    DPC.speed_demand = speedd.data
    
def compass_callback(compass):
    global DPC
    DPC.pitch = compass.pitch # pitch angle measured by xsens
    DPC.pitch_speed = compass.angular_velocity_y*np.pi/180.
    
def depth_callback(data):
    global DPC
    DPC.depth = data.depth_filt # depth filtered by PT_filter in compass_oceanserver.py
    
def depth_onOff_callback(onOff):
    global controller_onOff
    global timeLastCallback
    controller_onOff=onOff.data
    timeLastCallback = time.time()

################################################################################
######## SPEED OBSERVER ########################################################
################################################################################

def propeller_model(u_prop,_speed):
    if np.abs(u_prop)<10:   # deadband
        F_prop = 0
    else:
        # propeller model based on Turnock2010 e.q. 16.19
        Kt0 = 0.1001
        a = 0.6947
        b = 1.6243
        w_t = 0.36 # wake fraction
        t = 0.50 # thrust deduction
        D = 0.305 # peopeller diameter [m]
        rho = 1000 # water density [kg/m^3]
        
        rps = 0.5451*u_prop - 2.384 # infer propeller rotation speed from the propeller demand [rps]
                    
        J = _speed *(1-w_t)/rps/D;
        Kt = Kt0*(1 - (J/a)**b );
        F_prop = rho*rps**2*D**4*Kt*(1-t);

    return F_prop

def rigidbodyDynamics(_speed,F_prop):
    m = 79.2 # mass of the AUV [kg]
    X_u_dot = -3.46873716858361 # added mass of the AUV [kg]
    X_uu = -29.9811131464837 # quadratic damping coefficient [kg/m]
    
    acc = (X_uu*abs(_speed)*_speed+F_prop)/(m-X_u_dot)
    
    return acc
    
def speedObserver(u_prop,_speed,dt):
    # compute force from a propeller demand
    F_prop = propeller_model(u_prop,_speed)
    # implement Runge-Kutta 4th order to update the AUV speed
    k1 = rigidbodyDynamics(_speed,F_prop)
    k2 = rigidbodyDynamics(_speed+dt/2.*k1,F_prop)
    k3 = rigidbodyDynamics(_speed+dt/2.*k2,F_prop)
    k4 = rigidbodyDynamics(_speed+dt*k3,F_prop)

    speed_change = dt/6.*(k1+2*k2+2*k3+k4)
    _speed = _speed + speed_change
    return _speed

##################################
##initialisation
#################################

if __name__== '__main__':
    rospy.init_node('MPC_Depth_controller')#####

    global DPC
    DPC = depthandspeed_MPC()

    rospy.Subscriber('depth_demand', Float32, depth_demand_callback)
    rospy.Subscriber('speed_demand', Float32, speed_demand_callback)    
    DPC.pitch_demand = 0 # rospy.Subscriber('pitch_demand', Float32, pitch_demand_callback)
    rospy.Subscriber('compass_out', compass, compass_callback)    
    rospy.Subscriber('depth_out', depth, depth_callback)
    rospy.Subscriber('Depth_onOFF', Bool, depth_onOff_callback)
    
    pub_tail = rospy.Publisher('tail_setpoints_horizontal', tail_setpoints)
    pub_tail_vert = rospy.Publisher('tail_setpoints_vertical', tail_setpoints)
    pub_tsl  = rospy.Publisher('TSL_setpoints_vertical', tsl_setpoints)
    pub_DPC  = rospy.Publisher('Depth_pitch_controller_values_MPC', depthandspeed_MPC)
    pub_prop = rospy.Publisher('prop_demand', Int8)###
    pubMissionLog = rospy.Publisher('MissionStrings', String)
    pubStatus = rospy.Publisher('status', status)
    
    rospy.loginfo("Depth-Pitch controller online")

    main_control_loop()

    
