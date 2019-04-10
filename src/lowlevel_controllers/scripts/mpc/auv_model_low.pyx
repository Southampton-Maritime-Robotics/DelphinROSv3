import numpy as np
cimport numpy as np
from ctypes import *
import scipy

def mpc_model_low(float HDx,float massX,float Told,float HDz, float massZ,float prop,float HL, float HM,float G,float I, float HDp ,
                  float T_coeff,float Ltf,float Ltr,int Np,int Nc,gain = np.zeros([24,24],dtype = np.float)):
    Ac = np.matrix([[HDx*1.0/massX,0,0,Told*1.0/massX,0],
                    [0,0,1.0,0,0],
                    #[0,0,HDz/massZ,HL/massZ,0],
                    [0,0,HDz*1.0/massZ,(-prop+HL)*1.0/massZ,0],
                    [0,0,0,0,1.0],
                    [0,0,0,(HM + G)*1.0/I,HDp*1.0/I]],dtype = np.float)

    Bc = np.matrix([[1.0/massX,0,0],
                    [0,0,0],
                    [0,T_coeff*1.0/massZ,T_coeff*1.0/massZ],
                    [0,0,0],
                    [0,(Ltf*T_coeff)*1.0/I,(Ltr*T_coeff)*1.0/I]],dtype = np.float)
    
    Cc = np.matrix([[1,0,0,0,0], [0,1,0,0,0], [0,0,0,1,0]],dtype = np.float)
    Dc = np.zeros([3,3],dtype = np.float)

    
    SYS = tuple([Ac,Bc,Cc,Dc])
    
    [Ad,Bd,Cd,Dd] = cont2discrete(SYS,0.2)
    
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

    [n,n_in] = np.shape(B_e)
    E = Phi_Phi + gain

    Co = np.matrix([[1.0,0.0,0.0,0.0,0.0],[0.0,1.0,0.0,0.0,0.0],[0.0,0.0,0.0,1.0,0.0],[0.0,0.0,0.0,0,1.0]],dtype = np.float)
    return [Phi_Phi,Phi_F,Phi_R,n,E,Ad, Bd, Co]
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
        ms = scipy.linalg.expm(dt * em)

        ms = ms[: a.shape[0],:]
        ad = ms[:,0:a.shape[1]]
        bd = ms[:,a.shape[1]:]
        cd = c
        dd = d
       
    else:
        raise ValueError("Unknown transformation method '%s'" %method)

    return [ad, bd, cd ,dd]
