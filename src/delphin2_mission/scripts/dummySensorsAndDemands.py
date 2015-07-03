#!/usr/bin/python

"""
# Description

A node for testing of a dead_reckoning.py

It publishs a dummy information onto a following topic:
-compass_out
-depth_out
-prop_demand
-depth_demand
-sway_demand
-gps_out

# Modification

"""

import rospy
import time
import math
import pylab
from hardware_interfaces.msg import compass
from hardware_interfaces.msg import depth
from hardware_interfaces.msg import position
from std_msgs.msg            import Int8
from hardware_interfaces.msg import gps
from hardware_interfaces.msg import tail_setpoints
from hardware_interfaces.msg import tsl_setpoints

from delphin2_mission.library_highlevel     import library_highlevel

################################################################
################################################################
def publishData():

    controlRate = 20. # [Hz]
    controlPeriod = 1./controlRate
    r = rospy.Rate(controlRate)
    
    comInfo = compass()
    propDemand = 9.
    gpsInfo = gps()
    depthInfo = depth()
    thVer = tsl_setpoints()
    thHor = tsl_setpoints()
    csVer = tail_setpoints()
    csHor = tail_setpoints()
    
    controller = library_highlevel()
    
    timeZero = time.time()
    
    while not rospy.is_shutdown():
        
        timeRef = time.time()

        # fake compass information
        heading = 0 + 50*math.sin(2*math.pi/10*(time.time()-timeZero))

        heading = pylab.mod(heading,360)
        comInfo.heading = heading # [deg] relative to North, +ve CW

        # fake propeller demand
        if time.time()-timeZero < 20:
            propDemand = 0
        elif time.time()-timeZero < 40:
            propDemand = 22
        elif time.time()-timeZero < 60:
            propDemand = 10
        elif time.time()-timeZero < 80:
            propDemand = 15
        elif time.time()-timeZero < 100:
            propDemand = 0
####        propDemand = 22

        thVer.thruster0 = 0 # [2500,2500]
        thVer.thruster1 = 0 # [2500,2500]
        thHor.thruster0 = 0 # [2500,2500]
        thHor.thruster1 = 0 # [2500,2500]

        csVer.cs0 = 0 # [-30,30]
        csVer.cs1 = 0 # [-30,30]
        csHor.cs0 = 0 # [-30,30]
        csHor.cs1 = 0 # [-30,30]
        
        # fake gps information
        gpsInfo.number_of_satelites = 5
        gpsInfo.fix = 0
####        if time.time()-timeZero < 70:
####            gpsInfo.fix = 1
####        elif time.time()-timeZero < 80:
####            gpsInfo.fix = 0
        gpsInfo.speed = 0.657 # [m/s]
        gpsInfo.x = 0.
        gpsInfo.y = 0.
        
        # fake depth information
        pubDepth.publish(depthInfo)
        
        # fake depth demand # see if the AUV ignore the gps when the depth demand goes beyoud the threshold
        controller.setDepth(0)
        
        # publish
        pubCompass.publish(comInfo)
        pubProp.publish(propDemand)
        pubGPS.publish(gpsInfo)
        pub_tail_ver.publish(csVer)
        pub_tail_hor.publish(csHor)
        pub_tsl_ver.publish(thVer)
        pub_tsl_hor.publish(thHor)
                
        timeElapse = time.time()-timeRef
        if timeElapse < controlPeriod:
            r.sleep()
        else:
            str = "dummySensorAndDemand rate does not meet the desired value of %.2fHz: actual control rate is %.2fHz" %(controlRate,1/timeElapse) 
            rospy.logwarn(str)

################################################################################
######## INITIALISATION ########################################################
################################################################################

if __name__ == '__main__':

    time.sleep(1) #Allow System to come Online
    rospy.init_node('dummySensorsAndDemands')
    
    pubCompass = rospy.Publisher('compass_out', compass)
    pubProp = rospy.Publisher('prop_demand', Int8)
    pub_tail_ver = rospy.Publisher('tail_setpoints_vertical', tail_setpoints)
    pub_tail_hor = rospy.Publisher('tail_setpoints_horizontal', tail_setpoints)
    pub_tsl_hor = rospy.Publisher('TSL_setpoints_horizontal', tsl_setpoints)
    pub_tsl_ver = rospy.Publisher('TSL_setpoints_vertical', tsl_setpoints)
    pubGPS = rospy.Publisher('gps_out', gps)
    pubDepth = rospy.Publisher('depth_out', depth)

    publishData()   #Main loop for update the dummy topic
