#!/usr/bin/python
import roslib; roslib.load_manifest('hardware_interfaces')
import rospy
import time
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from hardware_interfaces.msg import compass
from hardware_interfaces.msg import position
from hardware_interfaces.msg import gps
from hardware_interfaces.msg import altitude
from hardware_interfaces.msg import depthandspeed_MPC
from hardware_interfaces.msg import heading_MPC
#from DelphinROSv2.msg import heading_control
from hardware_interfaces.msg import dead_reckoner
from hardware_interfaces.msg import camera_info
import math

######################################
#Modifications
#6/1/12 Modified GPS callback to only operate if a valid fix is returned
#9/2/12 Modified definition of X and Y. X is east and Y is north to be consistant with everything else.
#30/4/12 Need to modify code so using heading_control.msh not heading_mpc

#### DEFINE GLOBAL VARIABLES ####
#global flag
#global cur_compass
#global cur_sway
#global cur_prop

########## LOW LEVEL CONTROL ############################################################
def reckoner():

        R        = 6367500 #Radius of the earth in metre
#        dt        = 0.05
        controlRate = 100. # Hz FIXME: make this value as high as gps and altimeter node can be
        controlPeriod = 1/controlRate # sec
        r = rospy.rate(controlRate) # as high as the gps rate could be
        
        try:
            lat_orig  = rospy.get_param('lat_orig')
            long_orig = rospy.get_param('long_orig')
        except:
            lat_orig  =  50.95702499999999
            long_orig = -1.36675104
        
        distance = 0.0
        velX = 0.0 # surge vel
        velY = 0.0 # sway vel
        velZ = 0.0 # heave vel
        velP = 0.0 # roll vel
        velQ = 0.0 # pitch vel
        velR = 0.0 # yaw vel
        latitude  = lat_orig
        longitude = long_orig
        
        output  = position()
        output2 = dead_reckoner()
        
        t0      = time.time()

        print 'Entering main loop!!!'
        
############ MAIN RECKONER LOOP ################################################
        while not rospy.is_shutdown():
           
        #### GPS DATA ######################################################
            # only use GPS when it is possible   
            if GPS.fix == 1 and GPS.number_of_satelites >= 5 and DaS.depth_demand < 0.5:
                X = GPS.x # location in Earth-fixed frame relative to the origin: +ve East
                Y = GPS.y # location in Earth-fixed frame relative to the origin: +ve North
                latitude  = np.float64(GPS.latitude)
                longitude = np.float64(GPS.longitude)
                velX = GPS.speed # speed given from GPS module
        
        #### COMPASS DATA ##################################################
            heading = cur_compass.heading
            roll    = cur_compass.roll
            pitch   = cur_compass.pitch #_filt
            depth   = cur_compass.depth_filt #_filt
            velZ    = cur_compass.depth_der
            
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
            output.ValidGPSfix = GPS.fix
            output.altitude    = alt.altitude
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
            output2.ValidGPSfix = GPS.fix
            output2.altitude    = alt.altitude
            output2.X_dead = X_dead
            output2.Y_dead = Y_dead
            output2.velX_dead = velX_dead
            output2.velY_dead = velY_dead
            output2.temperature = temperature
            output2.frame0 = frame0
            output2.frame1 = frame1
            output2.velP_dead = velP_dead
            pub2.publish(output2)
            
            print output2
            
            #print 'time = ',time.time() - time_zero
            #print output2
            
            r.sleep()

		#### END OF MAIN DEAD RECKONING LOOP ####

######## END DEAD RECKONER #####################################################


#### WHEN NEW MISSION OR COMPASS DATA ARRIVES UPDATE VALUES AND RESET FLAG ####

def compass_cb(newcompass):
    global cur_compass
    cur_compass = newcompass   

def gps_callback(gps):
    global GPS
    GPS = gps
    
def altimeter_callback(altimeter):
    global alt
    alt = altimeter    
    
def depthandspeed_cb(data):
    global DaS
    DaS = data

def heading_cb(data):
    global head
    head = data
    
def temperature_cb(data):
    global temperature
    temperature = data.data
    
def camera_cb(data):
    global frame0
    global frame1
    frame0=data.cam0frame  
    frame1=data.cam1frame  
    
#### INITIALISATION ####
if __name__ == '__main__':
    rospy.init_node('dead_reckoner')
    
    global cur_compass
    global cur_sway
    global cur_prop
    global GPS
    global DaS
    global head
    global alt
    global temperature
    global frame0
    global frame1
    
    cur_compass = compass()
    GPS         = gps()
    alt         = altitude()
    DaS         = depthandspeed_MPC()
    head        = heading_MPC()
    cur_prop    = 0
    cur_sway    = 0
    temperature = 0.0
    frame0=0
    frame1=0
    
    rospy.Subscriber('altimeter_out',altitude, altimeter_callback)
    rospy.Subscriber('compass_out', compass, compass_cb)
    rospy.Subscriber('gps_out', gps, gps_callback)
      
    pub  = rospy.Publisher('position_dead', position, queue_size=3)
    pub2 = rospy.Publisher('dead_reckoner', dead_reckoner, queue_size=3)

    reckoner()
