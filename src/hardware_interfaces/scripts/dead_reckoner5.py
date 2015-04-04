#!/usr/bin/python
import rospy
import time
from numpy import *
from hardware_interfaces.msg import compass
from hardware_interfaces.msg import position
from hardware_interfaces.msg import gps
from hardware_interfaces.msg import altitude
#import math

######################################
#Modifications


########## LOW LEVEL CONTROL ############################################################
def reckoner():

        controlRate = 100. # Hz FIXME: make this value as high as gps and altimeter node can be
        controlPeriod = 1/controlRate # sec
        r = rospy.rate(controlRate) # as high as the gps rate could be
        
        try:
            lat_orig  = rospy.get_param('lat_orig')
            long_orig = rospy.get_param('long_orig')
        except:
            lat_orig  =  50.95702499999999
            long_orig = -1.36675104
        
        latitude  = lat_orig
        longitude = long_orig
        
        output  = position()
        
        print 'Entering main loop!!!'
        
############ MAIN RECKONER LOOP ################################################
        while not rospy.is_shutdown():
            
            timeRef = time.time()
        #### COMPASS DATA ##################################################
            # orientation in earth fixed frame: filtered by xsens
            heading = cur_compass.heading   # [deg]
            roll    = cur_compass.roll      # [deg]
            pitch   = cur_compass.pitch     # [deg]
            
            # translational accelerations in body-fixed frame FIXME: check the reference frame convention and g force
            accX    = cur_compass.ax
            accY    = cur_compass.ay
            accZ    = cur_compass.az
            
        #### DETERMINE VEHICLE STATE #######################################
            # translational velocities in body-fixed frame
            velX    = accX*controlPeriod
            velY    = accY*controlPeriod
            velZ    = accZ*controlPeriod
        
            # position
            Z   = cur_compass.depth_filt
            # Use XY from GPS when it is possible, otherwise, estimate from motion sensor
            if GPS.fix == 1 and GPS.number_of_satelites >= 5:
                X = GPS.x # location in Earth-fixed frame relative to the origin: +ve East
                Y = GPS.y # location in Earth-fixed frame relative to the origin: +ve North
                latitude  = float(GPS.latitude)
                longitude = float(GPS.longitude)
            else:
                # displacement in body-fixed frame
                disX    = velX*controlPeriod
                disY    = velY*controlPeriod
                disZ    = velZ*controlPeriod
                
                headingR = heading/180*pi # [rad]
                rollR    = roll/180*pi    # [rad]
                pitchR   = pitch/180*pi   # [rad]
                
                # XY LOCATION
                X = X + cos(headingR)*cos(pitchR)*disX + (-sin(headingR)*cos(rollR)+cos(headingR)*sin(pitchR)*sin(rollR))*disY + (sin(headingR)*sin(rollR)+cos(headingR)*cos(rollR)*sin(pitchR))*disZ # TODO need double-check
                Y = Y + sin(headingR)*cos(pitchR)*disX + (cos(headingR)*cos(rollR)+sin(rollR)*sin(pitchR)*sin(headingR))*disY + (-cos(headingR)*sin(rollR)+sin(pitchR)*sin(headingR)*cos(rollR))*disZ # TODO need double-check
                
        #### PUBLISH #######################################################
            output.X = X # position w.r.t. Earth-fixed frame relative to the origin
            output.Y = Y # position w.r.t. Earth-fixed frame relative to the origin
            output.Z = Z # position w.r.t. Earth-fixed frame relative to the origin
            output.forward_vel = velX # surge velocity w.r.t. body-fixed frame
            output.sway_vel    = velY # sway velocity w.r.t. body-fixed frame
            output.lat         = latitude
            output.long        = longitude
            output.ValidGPSfix = GPS.fix
            output.altitude    = alt.altitude
            pub.publish(output)
            
            timeElapse = time.time()-timeRef
            if timeElapse < controlPeriod:
                r.sleep()
            else:
                str = "gps_nmea2 rate does not meet the desired value of %.2fHz: actual control rate is %.2fHz" %(controlRate,1/timeElapse) 
                rospy.logwarn(str)

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
        
#### INITIALISATION ####
if __name__ == '__main__':
    rospy.init_node('dead_reckoner')
    
    global cur_compass
    global GPS
    global alt
    
    cur_compass = compass()
    GPS         = gps()
    alt         = altitude()
    
    rospy.Subscriber('altimeter_out',altitude, altimeter_callback)
    rospy.Subscriber('compass_out', compass, compass_cb)
    rospy.Subscriber('gps_out', gps, gps_callback)
    
    pub  = rospy.Publisher('position_dead', position, queue_size=3)
#    pub2 = rospy.Publisher('dead_reckoner', dead_reckoner, queue_size=3)

    reckoner()
