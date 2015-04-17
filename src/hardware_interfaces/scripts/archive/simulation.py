#!/usr/bin/python
import roslib; roslib.load_manifest('DelphinROSv2')
import rospy
import time
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from std_msgs.msg import Bool

from DelphinROSv2.msg import compass
from DelphinROSv2.msg import position
from DelphinROSv2.msg import gps
from DelphinROSv2.msg import altitude
from DelphinROSv2.msg import depthandspeed_MPC
from DelphinROSv2.msg import heading_MPC
#from DelphinROSv2.msg import heading_control
from DelphinROSv2.msg import dead_reckoner
from DelphinROSv2.msg import camera_info
import math


######################################
#Modifications
#6/1/12 Modified GPS callback to only operate if a valid fix is returned
#9/2/12 Modified definition of X and Y. X is east and Y is north to be consistant with everything else.
#30/4/12 Need to modify code so using heading_control.msh not heading_mpc


#### DEFINE GLOBAL VARIABLES ####
global Flag
global cur_compass
global cur_sway
global cur_prop


########## LOW LEVEL CONTROL ############################################################
def reckoner():
    
        global Flag
        Flag = True
        
    #### CONSTANTS ####
        mass     = 167.5
        massX    = 85.0
        massBG   = 55.0
        buoyancy =-5.0
        I        = 70.0
        Volume   = 0.08
        LTf      =-0.55
        LTr      = 0.49        
        LTfh     =-0.65
        LTrh     = 0.59
        Lr       = 1
        L        = 2
        BG       =-0.011
        m        = massBG*9.81
        rho      = 1000
        R        = 6367500 #Radius of the earth in m

    #### SETUP ####
        dt        = 1.0/20.0
        
        try:
            lat_orig  = rospy.get_param('lat_orig')
            long_orig = rospy.get_param('long_orig')
        except:
            lat_orig  =  50.95702499999999
            long_orig = -1.36675104
            
        time_zero = time.time()	
        
        X  = 0.0
        Y  = 0.0
        Z  = 0.0
        H  = 0.0
        P  = 0.0
        distance = 0.0
        velX = 0.0
        velY = 0.0
        velZ = 0.0
        velP = 0.0
        velH = 0.0
        latitude  = lat_orig
        longitude = long_orig
        
        output  = position()
        output2 = dead_reckoner()
        
        t0      = time.time()

        i = 0
        it = 0.0
        print 'Entering main loop!!!'
        
############ MAIN RECKONER LOOP ################################################
        while not rospy.is_shutdown():
            
            delta_t = time.time() - time_zero
            
            if Flag:
                time_zero = time.time()
        
          
            #### ACTUATOR DATA #################################################
                prop    = float(cur_prop)
                sternP  = DaS.delta
                T0      = DaS.T0
                T1      = DaS.T1
                
                rudder  = head.delta
                T2      = head.T2
                T3      = head.T3
                
            #### ACTUATOR CONSTANTS ############################################
                fCl = -0.0001574                                                   # Lift coefficient per degree
                fCd =  3.351e-5*((abs(sternP) + abs(rudder))/2)**2 + 0.0002868*((abs(sternP) + abs(rudder))/2)                         # Drag coefficient per degree squared
                fCm =  7.563e-5 
                
                hClZ =  0.001512*P                                               # Lift coefficient equation per degree
                hClY =  0.001512*velH
                hCd  =  0.06643*1.3    # 0.00143*np.abs(P)**2 +0.002375*np.abs(P) +       # Drag coefficient equation per degree squared
                hCmP =  0.0001375*P                                   # Moment coefficient equation
                hCmH =  0.0001375*velH                                   # Moment coefficient equation
                hCdp =  4.84
                hCdh =  2.0
                hCdz =  3.28  
            
            #### ACTUATOR FORCES ###############################################
                foil_liftZ   = -0.5*rho*(L**2)*(fCl*sternP)*velX*abs(velX)      
                foil_liftY   = -0.5*rho*(L**2)*(fCl*rudder)*velX*abs(velX)           
                foil_drag    = -0.5*rho*(Volume**(2.0/3.0))*(3.351e-5*np.abs(sternP)**2 + 0.0002868*np.abs(sternP))*velX*abs(velX)
                foil_momentP = -0.5*rho*(L**3)*(fCm*sternP)*velX*abs(velX)
                foil_momentH = -0.5*rho*(L**3)*(fCm*rudder)*velX*abs(velX)
                
                hull_liftZ   = -0.5*rho*(L**2)*(hClZ)*velX*abs(velX)  
                hull_liftY   = -0.5*rho*(L**2)*(hClY)*velX*abs(velX)  
                hull_drag    = -0.5*rho*(Volume**(2.0/3.0))*(hCd)*velX*abs(velX)
                hull_momentP =  0.0#0.5*rho*(L**3)*(hCmP)*velX*abs(velX)
                hull_momentH = -0.5*rho*(L**3)*(hCmH)*velX*abs(velX)
                
                if Z < 0.2 and velX > 0.5:
                    hull_wave_drag = -(15.233*velX**3 - 16.516*velX**2 + 5.7093*velX - 0.6264)
                else:
                    hull_wave_drag = 0

                hull_pitch_drag_moment   = -0.5*rho*(Volume**(2.0/3.0))*(hCdp)*velP*abs(velP)
                hull_heading_drag_moment = -0.5*rho*(Volume**(2.0/3.0))*(hCdh)*velH*abs(velH)
                hull_drag_Z              = -0.5*rho*(Volume**(2.0/3.0))*(hCdz)*velZ*abs(velZ)
                hull_drag_Y              = -0.5*rho*(Volume**(2.0/3.0))*(hCdz)*velH*abs(velH)
                
                
                T0 = T0/np.exp(abs(velX))
                T1 = T1/np.exp(abs(velX))
            
            #### PROPELLER #####################################################
                
                if prop > 9:
                    propT = (1.5116*prop - 13.9434)*0.5
                elif prop > 4 and prop < 9:
                    propT = (1.5116*prop - 6.9434)*0.5
                else:
                    propT = 0
                    
                                
            #### AUV MODEL #####################################################
                
                ## pitch ##
                accP = (hull_momentP + foil_momentP + T0*LTf + T1*LTr + BG*np.sin(np.radians(P))*m + hull_pitch_drag_moment -10*velP)/(I)
                velP = (accP*dt) + velP
                P    = velP*dt*180/np.pi + P
                
                ## heading ##
                accH = (hull_momentH + foil_momentH + T2*LTfh + T3*LTrh + hull_heading_drag_moment -10*velH)/(I)
                velH = accH*dt + velH
                H    = (velH*dt*180/np.pi + H)%360
                errorH  = cur_compass.heading - H
                
                ## sway speed ##
                accY = (hull_liftY + foil_liftY + T2 + T3 + hull_drag_Y - 20*velY)/mass 
                velY = accY*dt + velY
                
                ## forward speed ##
                accX = (propT*np.cos(np.radians(P)) + hull_drag + hull_wave_drag + foil_drag + T0*np.sin(np.radians(P)) + T1*np.sin(np.radians(P)))/massX
                velX = accX*dt + velX
                distance = velX*dt + distance
                
                ## depth ##
                accZ = (foil_liftZ + hull_liftZ + T0*np.cos(np.radians(P)) + T1*np.cos(np.radians(P)) + buoyancy + hull_drag_Z - 20*velZ)/mass
                velZ = accZ*dt + velZ
                Z    = velZ*dt + Z
                
                if Z < 0.0:
                    accZ = 0.0
                    velZ = 0.0
                    Z    = 0.0
            #### POSITION ######################################################
                h = H/180*np.pi

                #### XY VELOCITIES ####
                u = velX*np.sin(h)-velY*np.sin(h)
                v = velX*np.cos(h)+velY*np.cos(h)
            
                #### XY LOCATION ####
                X = X+u*dt
                Y = Y+v*dt

                
                #### LAT LONG ESTIMATION #######################################
                try:
                    brng=math.atan2(X,Y)
                    d=math.sqrt(X**2+Y**2)
                    #brng=math.radians(brng)
                    lat1 = math.radians(lat_orig)
                    lon1 = math.radians(long_orig)
                    lat2 = math.asin(math.sin(lat1)*math.cos(d/R) + math.cos(lat1)*math.sin(d/R)*math.cos(brng))
                    lon2 = lon1 + math.atan2(math.sin(brng)*math.sin(d/R)*math.cos(lat1), math.cos(d/R)-math.sin(lat1)*math.sin(lat2))
                    latitude=np.float64(math.degrees(lat2))
                    longitude=np.float64(math.degrees(lon2))
#                    print 'latitude = ',latitude
#                    print 'longitude = ',longitude
                except:
                    pass
            #### PUBLISH #######################################################
#                output.X = X
#                output.Y = Y
#                output.Z = depth
#                output.forward_vel = velX
#                output.sway_vel    = velY
#                output.lat         = latitude
#                output.long        = longitude
#                output.ValidGPSfix = GPS.fix
#                output.altitude    = alt.altitude
#                pub.publish(output)
                
                
                output2.X = X
                output2.Y = Y
                output2.Z = Z
                output2.distance= distance
                output2.heading = H
                output2.pitch   = P
                output2.roll    = 0.0
                output2.velX    = velX
                output2.velY    = velY
                output2.velZ    = velZ
                output2.velP    = velP
                output2.velH    = velH
                output2.latitude    = float(latitude)
                output2.longitude   = float(longitude)
                output2.ValidGPSfix = GPS.fix
                output2.altitude    = alt.altitude
                output2.X_dead = 0.0
                output2.Y_dead = 0.0
                output2.velX_dead = 0.0
                output2.velY_dead = 0.0
                output2.temperature = 0.0
                output2.frame0 = 0
                output2.frame1 = 0
                pub2.publish(output2)
                
                print 'Depth = ',Z
                print 'Pitch = ',P
                print 'velX  = ',velX
                print 'velZ  = ',velZ
                print 'time  = ',it*dt
                
                it = it + 1
                
                if i == 1:
                    sim_flag.publish(True)
                    Flag = False
                    i = 0
                else:
                    i = i + 1
                
                #print output2
                
                #print 'time = ',time.time() - time_zero
                #print output2
            else:
                pass#time.sleep(0.01)

		#### END OF MAIN DEAD RECKONING LOOP ####

######## END DEAD RECKONER #####################################################


#### WHEN NEW MISSION OR COMPASS DATA ARRIVES UPDATE VALUES AND RESET FLAG ####

def prop_cb(prop):
    global cur_prop
    cur_prop = prop.data       
    
def depthandspeed_cb(data):
    global DaS
    global Flag
    DaS = data
    Flag = True

def heading_cb(data):
    global head
    head = data
    
#### INITIALISATION ####
if __name__ == '__main__':
    rospy.init_node('dead_reckoner')
    
    global cur_compass
    global cur_sway
    global cur_prop
    global GPS
    global DaS
    global Flag
    
    cur_compass = compass()
    GPS         = gps()
    alt         = altitude()
    DaS         = depthandspeed_MPC()
    head        = heading_MPC()
    cur_prop    = 0
    cur_sway    = 0
    Flag        = True
    
    rospy.Subscriber('DepthandSpeed_MPC_values', depthandspeed_MPC, depthandspeed_cb)
    rospy.Subscriber('prop_demand',Int8, prop_cb)
      
    pub  = rospy.Publisher('position_dead', position)
    pub2 = rospy.Publisher('dead_reckoner', dead_reckoner)
    sim_flag = rospy.Publisher('simulation_flag', Bool)

    reckoner()
