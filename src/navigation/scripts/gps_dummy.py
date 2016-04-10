#!/usr/bin/python

"""
A node to publish a fake gps information. It is mainly used when testing the dead_reckoner.

"""

import rospy
import serial
import time
import numpy
import math
import string
from pylab import *
from hardware_interfaces.msg import gps
from hardware_interfaces.msg import status
from std_msgs.msg import String

################### GLOBAL VARIABLES ################### 

def listenForData(n_sat, valid_gpsFix):
    # initialise message structure
    gpsOut = gps()

    #Initialise values to zero - important as different messages contain different values
    X = 0
    Y = 0

    controlRate = 1. # Hz
    r = rospy.Rate(controlRate)
    controlPeriod = 1/controlRate
    
    try: 
        lat_orig = rospy.get_param('lat_orig')
        long_orig = rospy.get_param('long_orig')
    except:
        lat_orig = 50.9567
        long_orig = -1.36735

    # to control a timing for status publishing
    timeZero_status = time.time()
    try:
        dt_status = rospy.get_param('status_timing')
    except:
        dt_status = 2.
        
    timeZero = time.time()
    while not rospy.is_shutdown():

        # to control a timing for status publishing
        if time.time()-timeZero_status > dt_status:
            timeZero_status = time.time()
            pubStatus.publish(nodeID = 4, status = True)
        
        timeRef = time.time()
        gpsOut.latitude = 555
        gpsOut.longitude = 666 
        gpsOut.time = 99
        gpsOut.number_of_satelites = n_sat
        gpsOut.fix = valid_gpsFix
        gpsOut.speed = 99
        gpsOut.x = X
        gpsOut.y = Y
        pub.publish(gpsOut)
                    
        timeElapse = time.time()-timeRef
        if timeElapse < controlPeriod:
            r.sleep()
        else:
            str = "GPS rate does not meet the desired value of %.2fHz: actual control rate is %.2fHz" %(controlRate,1/timeElapse) 
            rospy.logwarn(str)

################### INITIALISING FUNCTION ################### 
if __name__ == '__main__':
    time.sleep(4) #Allow System to come Online
    
    rospy.init_node('gps_sensor')
  
    #Define Publishers
    pub = rospy.Publisher('gps_out', gps)
    pubStatus = rospy.Publisher('status', status)
    n_sat = 11
    valid_gpsFix = False
    str = "\n >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> \n A dummy auv system is being used instead of the actual system. \n >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> \n number of (dummy) satellites being detected: %s \n valid GPS fix: %s \n >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" %(n_sat, valid_gpsFix) 
    rospy.logwarn(str)
    
    listenForData(n_sat, valid_gpsFix)                     #Main loop for receiving data
