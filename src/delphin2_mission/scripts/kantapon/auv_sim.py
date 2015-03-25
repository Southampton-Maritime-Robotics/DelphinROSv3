#!/usr/bin/python
import rospy
import numpy
import time
import math
from hardware_interfaces.msg import compass
from hardware_interfaces.msg import position
from std_msgs.msg import Float32
from pylab import *

# this node subscribe to a motion demand, e.g. speed, heading, depth
# A state vector of the virtual AUV is updated accordingly
# The current state is then published as dummy messages of compass_out and position_dead

################################################################
################################################################
def listenForData():
  
    #### INITIALISE PARAMETERS ####
    
    # demand
    global headingDemand_cb
    global speedDemand_cb
    global depthDemand_cb
    headingDemand_cb = 0
    speedDemand_cb = 0
    depthDemand_cb = 0
    
    # increment
    incHeading = 2.5 # [(deg)/s] how quickly the heading can change FIXME
    incDepth = 0.01 # [(m)/s] how quickly the depth can change FIXME
    # saturation
    speedMax = 1 # [m/s] maximum speed
    depthMax = rospy.get_param('over-depth') # [m] maximum depth
    dt = 0.1 # [sec] time step size
    #####################
    
    # nu: velocity vector
    u = 0 # [m/s] initial surge velocity
    
    # eta: state vector
    X = 0 # [m]
    Y = 0 # [m]
    Z = 0 # [m]
    heading = 20 # [deg] North CW
    
    com = compass()
    pos = position()
    
    while not rospy.is_shutdown():
        
        # get demand
        demandSpeed = speedDemand_cb # [m/s]
        demandDepth = depthDemand_cb # [m]
        demandHeading = headingDemand_cb # [deg] North CW
        
        # determine error state
        errDepth = demandDepth-Z
        errHeading = demandHeading-heading
        if errHeading < -180:
            errHeading = errHeading%360
        elif errHeading > 180:
            errHeading = -(-errHeading%360)
        
        # update kinematic parameters
        u = demandSpeed
        if u > speedMax:
            u = speedMax
        elif u < 0:
            u = 0
            
        # update state vector
        heading = heading + sign(errHeading)*incHeading*dt
        if heading > 360:
            heading = heading-360
        elif heading < 0:
            heading = heading+360

        X = X+u*sin(heading*pi/180)
        Y = Y+u*cos(heading*pi/180)
        
        Z = Z + sign(errDepth)*incDepth*dt
        if Z > depthMax:
            Z = depthMax
        elif Z < 0:
            Z = 0
            
#        print X, Y, Z, heading, u
#        print demandSpeed, demandHeading, demandDepth
                
        com.heading = heading
        com.depth_filt = Z
        
        pos.X = X
        pos.Y = Y
        pos.Z = Z
        
        # publish
        pubCompass.publish(com)
        pubPosition.publish(pos)
#        time.sleep(0.02)

################################################################################
######## SATURATION AND UPDATE PARAMETERS FROM TOPICS ##########################
################################################################################

def limits(value, min, max):       #Function to contrain within defined limits
    if value < min:				   
       value = min
    elif value > max:
       value = max
    return value

################################################################################

def heading_demand_cb(headingd):
    global headingDemand_cb
    headingDemand_cb = headingd.data
    
def speed_demand_cb(speedd):
    global speedDemand_cb
    speedDemand_cb = speedd.data
    
def depth_demand_cb(depthd):
    global depthDemand_cb
    depthDemand_cb = depthd.data
    
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
    rospy.Subscriber('speed_demand', Float32, speed_demand_cb)
    rospy.Subscriber('depth_demand', Float32, depth_demand_cb)
        
    listenForData()   #Main loop for update the AUV parameters
