import numpy as np
def constraint(cdef int Nc,cdef float u,cdef float velX,cdef int flag, cdef float float cs_flag,cdef float vel_flag
               cdef float Propmin,cdef float Propmax,cdef float dPropmax,cdef float dPropmin,cdef float Tmax,cdef float Tmin,cdef float dTmax,cdef float dTmin,cdef float CSmax,
               cdef float CSmin, cdef float dCSmax,cdef float dCSmin):
#cs =0.4
#vel = 0.6
    if flag == 1:
        
        if velX < cs_flag:   #
            CSmax = 0.000001
            CSmin = 0.0
            
        Tmax = 0.0
        Tmin = 0.0
        constraints = np.matrix([[dPropmax,dPropmin,Propmax,Propmin],
                             [dCSmax,dCSmin,CSmax,CSmin]])    

    if flag == 0:
        CSmax = 0.0
        CSmin = 0.0

        constraints = np.matrix([[dPropmax,dPropmin,Propmax,Propmin],
                             [dTmax,dTmin,Tmax,Tmin],
                             [dTmax,dTmin,Tmax,Tmin]])

    if flag == -1:    

        if velX >= vel_flag: 
            Tmax=vel_flag +0.1-velX
            if Tmax <= 0:
                Tmax = 0.0
                Tmin = 0.0
            else:
                Tmax = (abs(vel_flag +0.1-velX)/0.1)**(0.5)*abs(u[2])
                Tmin = (abs(vel_flag +0.1-velX)/0.1)**(0.5)*abs(u[2])
        else:
            if velX < cs_flag:
                CSmax = 0.0
                CSmin = 0.0
 
    
        constraints = np.matrix([[dPropmax,dPropmin,Propmax,Propmin],
                             [dCSmax,dCSmin,CSmax,CSmin],
                             [dTmax,dTmin,Tmax,Tmin],
                             [dTmax,dTmin,Tmax,Tmin]])

    if flag == -2:    

        if velX < vel_flag: 
            CSmax = velX-vel_flag+0.1
            if CSmax <= 0.0:
                CSmax = 0.0
                CSmin = 0.0
            else:
                CSmax=abs(u[1])*(abs(velX-vel_flag+0.1)/0.1)  #
                CSmin = -abs(u[1])*(abs(velX-vel_flag+0.1)/0.1) #

        else:
            Tmax = 0.0
            Tmin = 0.0
            dTmax = 0.0
            dTmin = 0.0
  
        constraints = np.matrix([[dPropmax,dPropmin,Propmax,Propmin],
                             [dCSmax,dCSmin,CSmax,CSmin],
                             [dTmax,dTmin,Tmax,Tmin],
                             [dTmax,dTmin,Tmax,Tmin]])

    [i,c] = np.shape(constraints)

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
