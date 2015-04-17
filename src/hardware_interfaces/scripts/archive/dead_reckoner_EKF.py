#!/usr/bin/python

"""
######################################

This node is not functioning yet!!.
It is found that acceleration along produce a significant error that is not acceptable.
May need to implement SLAM alongside to provide a correction step when the GPS is not available.
Otherwise, the estimation is useless.

This node gives an estimation of AUV position in earth-fixed frame relative to the origin specified in launch file.
Z is measured directly from depth sensor whereas an altitude is measured directly from the altimeter.
Accommodate XY measure from GPS when it available. Otherwise calculate from FreeAccelerations in body-fixed frame measured by xsens.

Convention for position is [X:+veEast,Y:+veNorth,Z:+veDown].
state consists of [X,Y,Z,surge,sway,heave]: note that position is denoted in earth-fixed frame while velocity is denoted in body-fixed frame]
measurement consists of [X,Y,Z]: note that XY is measured from GPS while Z is measured from depth transducer

# TODO
check the rate of this node
verify this node
    -turn-off GPS, drag the vehicle around then check the trace this node made
    -check the covariance matrix for measurement and process noise

######################################
#Modifications
9/4/2015:   Exclude the altitude from the state as it is measure directly from altimeter
12/4/2015:  Fuse measurements from GPS and depth transducer with a dead_reckoning technique using Extended Kalman Filter

"""

import rospy
import time
from numpy import *
from math import *
from hardware_interfaces.msg import compass
from hardware_interfaces.msg import position
from hardware_interfaces.msg import gps
from hardware_interfaces.msg import altitude

########## LOW LEVEL CONTROL ############################################################
def reckoner():

    global cur_compass
    global GPS
    global hasGPS
    global alt

    output  = position()
    
    controlRate = 100. # Hz FIXME: make this value as high as gps and xsens node can be
    controlPeriod = 1/controlRate # sec
    r = rospy.rate(controlRate) # as high as the gps rate could be
    
    try:
        lat_orig  = rospy.get_param('lat_orig')
        long_orig = rospy.get_param('long_orig')
    except:
        lat_orig  =  50.95702499999999
        long_orig = -1.36675104
    
    # constants
    e_x = array([[0.01/2.*controlPeriod**2],    # process noise in X (std.) [m] # TODO: adjust me based on the stdndard deviation of accelerations
                 [0.01/2.*controlPeriod**2],    # process noise in Y (std.) [m]
                 [0.01/2.*controlPeriod**2],    # process noise in Z (std.) [m]
                 [0.02*controlPeriod],          # process noise in u (std.) [m/s]
                 [0.02*controlPeriod],          # process noise in v (std.) [m/s]
                 [0.02*controlPeriod]])         # process noise in w (std.) [m/s]
    e_z = array([[1.0],  # measurement noise in X [m] # TODO: adjust me based on the stanrard deviation of gps and depth transducer
                 [1.0],  # measurement noise in Y [m]
                 [0.2],  # measurement noise in Z [m]
    Ex = e_x*e_x.T # process noise covariance    
    Ez = e_z*e_z.T # measurement noise covariance
    H = array([[1., 0., 0., 0., 0., 0.],     # take X from state vector as the expected measurement
               [0., 1., 0., 0., 0., 0.],     # take Y from state vector as the expected measurement
               [0., 0., 1., 0., 0., 0.]])    # take Z from state vector as the expected measurement

    # initial conditions    
    stateAUV = array([[0.],  # X: absolute location in earth-fixed frame
                      [0.],  # Y: absolute location in earth-fixed frame
                      [0.],  # Z: absolute location in earth-fixed frame
                      [0.],  # u: translational velocities in body-fixed frame
                      [0.],  # v: translational velocities in body-fixed frame
                      [0.]]) # w: translational velocities in body-fixed frame
    P = Ex  # initial covariance matrix
    latitude  = lat_orig
    longitude = long_orig
    rateOK = True   # delta t is determined based on the rate of this node
    hasGPS = False
    
    print 'Entering main loop!!!'
    
############ MAIN RECKONER LOOP ################################################
    while not rospy.is_shutdown():
        
        timeRef = time.time()
        
    #### GET SAMPLING FROM SENSORS AND UPDATE PARAMETERS ###########################
        ## IMU DATA (xsens MTi-30)
        # orientation in earth fixed frame
        heading     = cur_compass.heading       # [deg]
        roll        = cur_compass.roll          # [deg]
        pitch       = cur_compass.pitch         # [deg]
        headingR    = heading/180*pi            # [rad]
        pitchR      = pitch/180*pi              # [rad]
        depth       = cur_compass.depth_filt    # [m]
        # linear FreeAccelerations in body-fixed frame FIXME: check the reference frame convention and g force
        acc = array([[cur_compass.ax],
                     [cur_compass.ay],
                     [cur_compass.az]])

        ## UPDATE PARAMETERS
        if rateOK:
            T = controlPeriod
        else:
            T = timeElapse
        # TODO needed double-check
        a11 = cos(headingR)*cos(pitchR)
        a12 = -sin(headingR)
        a13 = cos(headingR)*sin(pitchR)
        a21 = sin(headingR)*cos(pitchR)
        a22 = cos(headingR)
        a23 = sin(pitchR)*sin(headingR)
        a31 = -sin(pitchR)
        a32 = 0.
        a33 = cos(pitchR)
        A = array([[1., 0., 0., a11*T,  a12*T,  a13*T],     # X
                   [0., 1., 0., a21*T,  a22*T,  a23*T],     # Y
                   [0., 0., 1., a31*T,  a32*T,  a33*T],     # Z
                   [0., 0., 0., 1.,     0.,     0.],        # u
                   [0., 0., 0., 0.,     1.,     0.],        # v
                   [0., 0., 0., 0.,     0.,     1.]])       # w
        B = array([[a11/2.*T**2,  a12/2.*T**2,  a13/2.*T**2],   # X
                   [a21/2.*T**2,  a22/2.*T**2,  a23/2.*T**2],   # Y
                   [a31/2.*T**2,  a32/2.*T**2,  a33/2.*T**2],   # Z
                   [T,            0.,           0.],            # u
                   [0.,           T,            0.],            # v
                   [0.,           0.,           T]])            # w

    #### Prediction of AUV State #####################################################
        stateEstimated = A*stateAUV + B*acc
        P = A*P*A.T + Ex

    #### Measurement of AUV State ####################################################
        # assume the AUV aligns horizontally, hence, a measurement of depth and its derivative are taken from the depth transducer.
        Z_mea = depth 
        # get the gps reading when it is available, otherwise rely on the estimation
        if hasGPS:
            X_mea = GPS.x # location in Earth-fixed frame relative to the origin: +ve East
            Y_mea = GPS.y # location in Earth-fixed frame relative to the origin: +ve North
            latitude  = float(GPS.latitude)
            longitude = float(GPS.longitude)
        else:
            # solely rely on the estimation when the GPS is not available
            X_mea = X_est
            Y_mea = Y_est
            
        measurement = array([[X_mea],     # X
                             [Y_mea],     # Y
                             [Z_mea]])    # Z
                               
    #### Correction #################################################################
        res = measurement-H*stateEstimated # reasidual or measurement innovation: H is identity matrix, so, it is ignored to ease a computation
        S   = H*P*H.T+Ez
        K   = P*H.T*linalg.inv(S)
        
        stateAUV = stateEstimated+K*res
        P = P-K*H*P
        
        X_ref = X
        Y_ref = Y
        hasGPS = False

    #### PUBLISH #######################################################
        output.X = stateAUV[0][0] # position w.r.t. Earth-fixed frame relative to the origin
        output.Y = stateAUV[1][0] # position w.r.t. Earth-fixed frame relative to the origin
        output.Z = stateAUV[2][0] # position w.r.t. Earth-fixed frame relative to the origin
        output.forward_vel = stateAUV[3][0] # surge velocity w.r.t. body-fixed frame
        output.sway_vel    = stateAUV[4][0] # sway velocity w.r.t. body-fixed frame
        output.lat         = latitude
        output.long        = longitude
        output.ValidGPSfix = GPS.fix
        output.altitude    = alt.altitude # distance from the bottom
        pub.publish(output)
        
        timeElapse = time.time()-timeRef
        if timeElapse < controlPeriod:
            rateOK = True
            r.sleep()
        else:
            rateOK = False
            str = "dead_reckoner rate does not meet the desired value of %.2fHz: actual control rate is %.2fHz" %(controlRate,1/timeElapse) 
            rospy.logwarn(str)

	#### END OF MAIN DEAD RECKONING LOOP ####

######## END DEAD RECKONER #####################################################

#### WHEN NEW SENSOR DATA ARRIVES UPDATE VALUES ####

def compass_cb(newcompass):
    global cur_compass
    cur_compass = newcompass   

def gps_callback(gps):
    global GPS
    global hasGPS
    GPS = gps
    if GPS.fix == 1 and GPS.number_of_satelites >= 5:
        hasGPS = true
    
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
    
    pub  = rospy.Publisher('position_dead', position)
#    pub2 = rospy.Publisher('dead_reckoner', dead_reckoner)

    reckoner()
