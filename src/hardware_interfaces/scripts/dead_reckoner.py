#!/usr/bin/python

"""
A node that gives an estimated AUV's state based on a dead-reckoning algorithm.

This node relies more on GPS when it is available, otherwise an equation of motion will be used to keep tracking the AUV state.

#TODO
-include sway_demand in the model

######################################
#Modifications
6/1/12 Modified GPS callback to only operate if a valid fix is returned
9/2/12 Modified definition of X and Y. X is east and Y is north to be consistant with everything else.
22/4/15 Have seperate callback blocks that subscribe to different actuator demands instead of using the MPC-based topic.

"""

import rospy
import time
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from hardware_interfaces.msg import compass
from hardware_interfaces.msg import depth
from hardware_interfaces.msg import position
from hardware_interfaces.msg import tail_setpoints
from hardware_interfaces.msg import tsl_setpoints
from hardware_interfaces.msg import gps
from hardware_interfaces.msg import altitude
from hardware_interfaces.msg import dead_reckoner
from hardware_interfaces.msg import camera_info
import math

########## LOW LEVEL CONTROL ############################################################
def reckoner():
                
    #### CONSTANTS ####
        mass     = 167.5
        massX    = 90.0
        buoyancy =-6.0
        I        = 70
        Volume   = 0.08
        LTf      =-0.55
        LTr      = 0.49        
        LTfh     =-0.65
        LTrh     = 0.59
        Lr       = 1
        L        = 2
        BG       =-0.011
        m        = mass*9.81
        rho      = 1000
        R        = 6367500 #Radius of the earth in m

    #### SETUP ####
        dt        = 0.05
        
        try:
            lat_orig  = rospy.get_param('lat_orig')
            long_orig = rospy.get_param('long_orig')
        except:
            lat_orig  =  50.95702499999999
            long_orig = -1.36675104
            
        time_zero = time.time()	
        
        X  = 0.0
        Y  = 0.0
        H  = 0.0
        distance = 0.0
        velX = 0.0
        velY = 0.0
        velZ = 0.0
        velP = 0.0
        velH = 0.0
        velP_dead = 0.0
        latitude  = lat_orig
        longitude = long_orig
        
        output  = position()
        output2 = dead_reckoner()
        
        t0      = time.time()

        print 'Entering main loop!!!'

############ MAIN RECKONER LOOP ################################################
        while not rospy.is_shutdown():
            
            delta_t = time.time() - time_zero
            
            if delta_t >= dt:
                time_zero = time.time()
            #### GPS DATA ######################################################    
                if gpsInfo.fix == 1 and gpsInfo.number_of_satelites >= 5 and DaS.depth_demand < 0.5: # FIXME: use depth_demand instead
                    X = gpsInfo.x
                    Y = gpsInfo.y
                    latitude  = np.float64(gpsInfo.latitude)
                    longitude = np.float64(gpsInfo.longitude)
                    velX = gpsInfo.speed
            
            #### COMPASS DATA ##################################################
                heading = cur_compassInfo.heading
                depth   = cur_depth.depth_filt
                pitch   = cur_compassInfo.pitch
                roll    = cur_compassInfo.roll
                velZ    = cur_depth.depth_der
                velP    = cur_compassInfo.angular_velocity_y # TODO: double check if this is of the convention
                
            #### ACTUATOR DATA #################################################

                prop    = float(cur_prop)
                sternP  = cur_sternP
                T0      = cur_thVert[0]
                T1      = cur_thVert[1]
                
                rudder  = cur_rudder
                T2      = cur_thHori[0]
                T3      = cur_thHori[1]
                
            #### ACTUATOR CONSTANTS ############################################
                fCl = -0.0001574                                                   # Lift coefficient per degree
                fCd =  3.351e-5*((abs(sternP) + abs(rudder))/2)**2 + 0.0002868*((abs(sternP) + abs(rudder))/2)                         # Drag coefficient per degree squared
                fCm =  7.563e-5 
                
                hClZ =  0.001512*pitch                                               # Lift coefficient equation per degree
                hClY =  0.001512*velH
#                hCd  =  0.00143*np.abs(pitch)**2 +0.002375*np.abs(pitch) + 0.06643       # Drag coefficient equation per degree squared
                hCd  =  0.06643*1.3       # Drag coefficient equation per degree squared
                hCmP =  0.0001375*pitch                                   # Moment coefficient equation
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
                hull_momentP = -0.5*rho*(L**3)*(hCmP)*velX*abs(velX)
                hull_momentH = -0.5*rho*(L**3)*(hCmH)*velX*abs(velX)
                
                if depth < 0.2 and velX > 0.5:
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
                    
####                print 'sternP = ',sternP
####                print 'propT = ',propT*np.cos(np.radians(pitch))
####                print 'hull drag = ',hull_drag
####                print 'foil drag = ',foil_drag
####                print 'T0 drag = ',T0*np.sin(np.radians(pitch))
####                print 'T1 drag = ',T1*np.sin(np.radians(pitch))
                                
            #### AUV MODEL #####################################################
                
                ## pitch ##
                accP = (hull_momentP + foil_momentP + T0*LTf + T1*LTr + BG*np.sin(np.radians(pitch))*m + hull_pitch_drag_moment)/(I)
                velP_dead = ((accP*dt)*180/np.pi + velP_dead)*0.75 + velP*0.25
                #P    = velP*dt*180/np.pi + pitch
                
                ## heading ##
                accH = (hull_momentH + foil_momentH + T2*LTfh + T3*LTrh + hull_heading_drag_moment -10*velH)/(I)
                velH = accH*dt + velH
                H    = (velH*dt*180/np.pi + H)%360
                errorH  = cur_compassInfo.heading - H
                
                ## sway speed ##
                accY = (hull_liftY + foil_liftY + T2 + T3 + hull_drag_Y - 20*velY)/mass 
                velY = accY*dt + velY
                
                ## forward speed ##
                accX = (propT*np.cos(np.radians(pitch)) + hull_drag + hull_wave_drag + foil_drag + T0*np.sin(np.radians(pitch)) + T1*np.sin(np.radians(pitch)))/massX
                velX = accX*dt + velX
                distance = velX*dt + distance
                
                ## depth ##
                accZ = (foil_liftZ + hull_liftZ + T0*np.cos(np.radians(pitch)) + T1*np.cos(np.radians(pitch)) + buoyancy + hull_drag_Z - 20*velZ)/mass
                velZ = accZ*dt + velZ
                Z    = velZ*dt + depth
                    
            #### POSITION ######################################################
                h = heading/180*np.pi

                #### XY VELOCITIES ####
                u = velX*np.sin(h)-velY*np.sin(h)
                v = velX*np.cos(h)+velY*np.cos(h)
            
                #### XY LOCATION ####
                X = X+u*dt
                Y = Y+v*dt

                if (time.time()-t0)<20.0:
                    X_dead=X
                    Y_dead=Y
                    velX_dead=velX
                    velY_dead=velY
                else:
    
                    foil_liftY   = -0.5*rho*(L**2)*(fCl*rudder)*velX_dead*abs(velX_dead)           
                    foil_drag    = -0.5*rho*(Volume**(2.0/3.0))*(3.351e-5*np.abs(sternP)**2 + 0.0002868*np.abs(sternP))*velX_dead*abs(velX_dead)
                    hull_liftY   = -0.5*rho*(L**2)*(hClY)*velX_dead*abs(velX_dead)  
                    hull_drag    = -0.5*rho*(Volume**(2.0/3.0))*(hCd)*velX_dead*abs(velX_dead)
                    hull_drag_Y  = -0.5*rho*(Volume**(2.0/3.0))*(hCdz)*velH*abs(velH)
                    
                    if depth < 0.2 and velX_dead > 0.5:
                        hull_wave_drag = -(15.233*velX_dead**3 - 16.516*velX_dead**2 + 5.7093*velX_dead - 0.6264)
                    else:
                        hull_wave_drag = 0

                    
                    accY_dead = (hull_liftY + foil_liftY + T2 + T3 + hull_drag_Y - 20*velY_dead)/mass 
                    velY_dead = accY_dead*dt + velY_dead              
                    accX_dead = (propT*np.cos(np.radians(pitch)) + hull_drag + hull_wave_drag + foil_drag + T0*np.sin(np.radians(pitch)) + T1*np.sin(np.radians(pitch)))/massX
                    velX_dead = accX_dead*dt + velX_dead
                
                    u_dead = velX_dead*np.sin(h)-velY_dead*np.sin(h)
                    v_dead = velX_dead*np.cos(h)+velY_dead*np.cos(h)
                    
                    X_dead=X_dead+u_dead*dt
                    Y_dead=Y_dead+v_dead*dt
                
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
                    print 'latitude = ',latitude
                    print 'longitude = ',longitude
                except:
                    pass
            #### PUBLISH #######################################################
                output.X = X
                output.Y = Y
                output.Z = depth
                output.forward_vel = velX
                output.sway_vel    = velY
                output.lat         = latitude
                output.long        = longitude
                output.ValidGPSfix = gpsInfo.fix
                output.altitude    = altInfo.altitude
                pub.publish(output)
                
                output2.X = X
                output2.Y = Y
                output2.Z = depth
                output2.distance= distance
                output2.heading = heading
                output2.pitch   = pitch
                output2.roll    = roll
                output2.velX    = velX
                output2.velY    = velY
                output2.velZ    = velZ
                output2.velP    = velP
                output2.velH    = velH
                output2.latitude    = float(latitude)
                output2.longitude   = float(longitude)
                output2.ValidGPSfix = gpsInfo.fix
                output2.altitude    = altInfo.altitude
                output2.X_dead = X_dead
                output2.Y_dead = Y_dead
                output2.velX_dead = velX_dead
                output2.velY_dead = velY_dead
                output2.temperature = tempInfo
                output2.frame0 = frame0
                output2.frame1 = frame1
                output2.velP_dead = velP_dead
                pub2.publish(output2)
                
            else:
                time.sleep(0.01)

		#### END OF MAIN DEAD RECKONING LOOP ####

######## END DEAD RECKONER #####################################################


#### WHEN NEW MISSION OR COMPASS DATA ARRIVES UPDATE VALUES AND RESET FLAG ####

def compass_cb(newcompass):
    global cur_compassInfo
    cur_compassInfo = newcompass
    
def depthOut_cb(newdepth):
    global cur_depthInfo
    cur_depthInfo = newdepth
    
def depthDemand_cb(depthd):
    global cur_depth_demand
    cur_depth_demand = depthd.data

def prop_cb(prop):
    global cur_prop_demand
    cur_prop = prop.data
    
def cs_hori_cb(data):
    global cur_sternP_demand
    cur_sternP_demand = data.cs0 # assume the demand on both surfaces are identical
    
def cs_vert_cb(data):
    global cur_rudder_demand
    cur_rudder_demand = data.cs0 # assume the demand on both surfaces are identical
    
def th_hori_cb(data):
    global cur_thHori_demand
    cur_thHori_demand = data
    
def th_vert_cb(data):
    global cur_thVert_demand
    cur_thVert_demand = data

def sway_cb(sway):
    global cur_sway_demand
    cur_sway_demand = sway.data

def gps_callback(gps):
    global gpsInfo
    gpsInfo = gps
    
def altimeter_callback(altimeter):
    global altInfo
    altInfo = altimeter    
    
def temperature_cb(data):
    global tempInfo
    tempInfo = data.data
    
def camera_cb(data):
    global frame0
    global frame1
    frame0=data.cam0frame  
    frame1=data.cam1frame  
    
#### INITIALISATION ####
if __name__ == '__main__':
    rospy.init_node('dead_reckoner')
    
    global cur_compassInfo
    global cur_depthInfo
    global cur_depth_demand
    global cur_prop_demand
    global cur_sternP_demand
    global cur_rudder_demand
    global cur_thHori_demand
    global cur_thVert_demand
    global cur_sway_demand
    global gpsInfo
    global altInfo
    global tempInfo
    global frame0
    global frame1

    cur_compassInfo = compass()
    cur_depthInfo = depth()
    cur_depth_demand = 0.
    cur_prop_demand = 0.
    cur_sternP_demand = tail_setpoints()
    cur_rudder_demand = tail_setpoints()
    cur_thHori_demand = tsl_setpoints()
    cur_thVert_demand = tsl_setpoints()
    cur_sway_demand   = 0.
    gpsInfo         = gps()
    altInfo         = altitude()
    tempInfo        = 0.
    frame0=0
    frame1=0
    
    rospy.Subscriber('altimeter_out',altitude, altimeter_callback) # Will not be used in here, just subscribe and pass it through the publisher.
    rospy.Subscriber('compass_out', compass, compass_cb)
    rospy.Subscriber('depth_out', depth, depthOut_cb)
    rospy.Subscriber('prop_demand',Int8, prop_cb)
    rospy.Subscriber('depth_demand',Float32, depthDemand_cb)
    rospy.Subscriber('sway_demand',Float32, sway_cb)
    rospy.Subscriber('tail_setpoints_horizontal', tail_setpoints, cs_hori_cb)
    rospy.Subscriber('tail_setpoints_vertical', tail_setpoints, cs_vert_cb)
    rospy.Subscriber('TSL_setpoints_horizontal', tsl_setpoints, th_hori_cb)
    rospy.Subscriber('TSL_setpoints_vertical', tsl_setpoints, th_vert_cb)        
    rospy.Subscriber('gps_out', gps, gps_callback)
    rospy.Subscriber('water_temp', Float32, temperature_cb)
    rospy.Subscriber('camera_info', camera_info, camera_cb)
      
    pub  = rospy.Publisher('position_dead', position)
    pub2 = rospy.Publisher('dead_reckoner', dead_reckoner)

    reckoner()
