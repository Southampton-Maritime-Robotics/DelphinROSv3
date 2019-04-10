import numpy as np
cimport numpy as np
from ctypes import *
def  constraint(int Nc,float u1,float u2,float velX,float flag,float cs_flag,float vel_flag,
                float Propmin,float Propmax,float dPropmax,float dPropmin,
               float Tmax,float Tmin,float dTmax,float dTmin,float CSmax, float CSmin,float dCSmax,float dCSmin):
#cs =0.4
#vel = 0.6

    cdef  constraintshigh = np.zeros([2,4],float)
    cdef  constraintslow = np.zeros([3,4],float)
    cdef  constraintsmul = np.zeros([4,4],float)

    if flag == 1:

        if velX < cs_flag:   #
            CSmax = 0.000001
            CSmin = 0.0
            
        Tmax = 0.0
        Tmin = 0.0
        
        constraintshigh = np.matrix([[dPropmax,dPropmin,Propmax,Propmin],
                             [dCSmax,dCSmin,CSmax,CSmin]])        

        return constraintshigh
    else:
        if flag == 0:
            CSmax = 0.0
            CSmin = 0.0
            constraintslow = np.matrix([[dPropmax,dPropmin,Propmax,Propmin],
                             [dTmax,dTmin,Tmax,Tmin],
                             [dTmax,dTmin,Tmax,Tmin]])
            return constraintslow
        else:
            if flag == -1:    
                if velX >= vel_flag: 
                    Tmax=vel_flag +0.1-velX
                    if Tmax <= 0:
                        Tmax = 0.0
                        Tmin = 0.0
                    else:
                        Tmax = (abs(vel_flag +0.1-velX)/0.1)**(0.5)*abs(u2)
                        Tmin = (abs(vel_flag +0.1-velX)/0.1)**(0.5)*abs(u2)
                else:
                    if velX < cs_flag:
                        CSmax = 0.0
                        CSmin = 0.0
                constraintsmul = np.matrix([[dPropmax,dPropmin,Propmax,Propmin],
                             [dCSmax,dCSmin,CSmax,CSmin],
                             [dTmax,dTmin,Tmax,Tmin],
                             [dTmax,dTmin,Tmax,Tmin]])
                return constraintsmul
            else:
                if flag == -2:    
                    if velX < vel_flag: 
                        CSmax = velX-vel_flag+0.1
                        if CSmax <= 0.0:
                            CSmax = 0.0
                            CSmin = 0.0
                        else:
                            CSmax=abs(u1)*(abs(velX-vel_flag+0.1)/0.1)  #
                            CSmin = -abs(u1)*(abs(velX-vel_flag+0.1)/0.1) #

                    else:
                        Tmax = 0.0
                        Tmin = 0.0
                        dTmax = 0.0
                        dTmin = 0.0

        
                constraintsmul = np.matrix([[dPropmax,dPropmin,Propmax,Propmin],
                             [dCSmax,dCSmin,CSmax,CSmin],
                             [dTmax,dTmin,Tmax,Tmin],
                             [dTmax,dTmin,Tmax,Tmin]])

                return constraintsmul




