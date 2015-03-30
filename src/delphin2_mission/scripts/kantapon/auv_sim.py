#!/usr/bin/python
import rospy
import numpy
import time
import math
from hardware_interfaces.msg import compass
from hardware_interfaces.msg import position
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from pylab import *

#### from kantapon's folder
from utilities                    import uti

# this node subscribe to a motion demand, e.g. speed, heading, depth
# A state vector of the virtual AUV is updated accordingly
# The current state is then published as dummy messages of compass_out and position_dead

################################################################
################################################################
def listenForData():
  
    #### INITIALISE PARAMETERS ####
    
    # utilities
    myUti = uti()
    
    # demand
    global headingDemand_cb
    global depthDemand_cb
    global rearPropDemand_cb
    global swayDemand_cb
    headingDemand_cb = 0
    depthDemand_cb = 0
    rearPropDemand_cb = 0
    swayDemand_cb = 0
    
    global hasHeadingDemand
    global hasDepthDemand
    global hasPropDemand
    global hasSwayDemand
    hasHeadingDemand = False
    hasDepthDemand = False
    hasPropDemand = False
    hasSwayDemand = False
    
    # increment
    incHeading = 10 # [(deg)/s] how quickly the heading can change FIXME: tune the value
    incDepth = 0.01 # [(m)/s] how quickly the depth can change FIXME: tune the value
    # saturation
    speedMax = 1 # [m/s] maximum surge speed
    swayMax = 0.5 # [m/s] maximum sway speed
    depthMax = rospy.get_param('over-depth') # [m] maximum depth
    dt = 0.1 # [sec] time step size
    
    # nu: velocity vector
    u = 0 # [m/s] initial surge velocity
    
    # eta: state vector
    X = 0 # [m]
    Y = 0 # [m]
    Z = 0 # [m]
    heading = 0 # [deg] North CW
    
    com = compass()
    pos = position()
    
    while not rospy.is_shutdown():
        
        # get demand
        demandRearProp = rearPropDemand_cb # [m/s]
        demandDepth = depthDemand_cb # [m]
        demandHeading = headingDemand_cb # [deg] North CW
        demandSway = swayDemand_cb # [m/s] stb
        
        # determine error state
        errDepth = demandDepth-Z
        errHeading = myUti.computeHeadingError(demandHeading,heading)
        
        # update kinematic parameters and state vector
        if hasHeadingDemand:
            heading = mod( heading+sign(errHeading)*incHeading*dt, 360 )
            hasHeadingDemand = False

        if hasPropDemand:
            if demandRearProp < 10:
                u = 0 # propeller deadband
            else:
                u = demandRearProp*1./22. # estimated from Leo's thesis
            u = myUti.limits(u,0,speedMax)
            X = X+u*dt*sin(heading*pi/180)
            Y = Y+u*dt*cos(heading*pi/180)
            hasPropDemand = False
        
        if hasSwayDemand:
            v = myUti.limits(demandSway,-swayMax,swayMax)
            X = X+v*dt*cos(heading*pi/180)
            Y = Y+v*dt*sin(heading*pi/180)
            hasSwayDemand = False
            
        if hasDepthDemand:
            Z = Z + sign(errDepth)*incDepth*dt
            Z = myUti.limits(Z,0,depthMax)
            hasDepthDemand = False
                    
#        print demandRearProp, u
#        print X, Y, Z, heading, u
#        print demandSpeed, demandHeading, demandDepth
                
        com.heading = heading
        com.depth_filt = Z
        
        pos.X = X
        pos.Y = Y
        pos.Z = Z
        pos.forward_vel = u
        
        # publish
        pubCompass.publish(com)
        pubPosition.publish(pos)
#        time.sleep(0.0001)

################################################################################
######## SATURATION AND UPDATE PARAMETERS FROM TOPICS ##########################
################################################################################

def heading_demand_cb(headingd):
    global headingDemand_cb
    global hasHeadingDemand
    headingDemand_cb = headingd.data
    hasHeadingDemand = True
   
def rearProp_demand_cb(propd):
    global rearPropDemand_cb
    global hasPropDemand
    rearPropDemand_cb = propd.data
    hasPropDemand = True
    
def depth_demand_cb(depthd):
    global depthDemand_cb
    global hasDepthDemand
    depthDemand_cb = depthd.data
    hasDepthDemand = True
    
def sway_demand_cb(swayd):
    global swayDemand_cb
    global hasSwayDemand
    swayDemand_cb = swayd.data
    hasSwayDemand = True
    
################################################################################
######## INITIALISATION ########################################################
################################################################################

if __name__ == '__main__':
    time.sleep(1) #Allow System to come Online
    rospy.init_node('auvsim')
    
    global pubCompass
    
    pubCompass = rospy.Publisher('compass_out', compass)
    pubPosition = rospy.Publisher('position_dead', position)
    
    rospy.Subscriber('heading_demand', Float32, heading_demand_cb)
    rospy.Subscriber('depth_demand', Float32, depth_demand_cb)
    rospy.Subscriber('prop_demand', Int8, rearProp_demand_cb)
    rospy.Subscriber('sway_demand', Float32, sway_demand_cb)
        
    listenForData()   #Main loop for update the AUV parameters
