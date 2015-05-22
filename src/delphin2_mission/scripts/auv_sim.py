#!/usr/bin/python

"""
# Description
This node subscribe to a motion demand, e.g. speed, heading, depth.
A state vector of the virtual AUV is updated accordingly based on the kinematic equation, i.e. the inertia is neglected.
The current AUV state is then published as dummy messages of compass_out, depth_out and position_dead.

#Available command
-depth
-heading
-rudder
-thrusterHor
-prop

#Todo
-have the AUV depth reset to zero when there is no depth demand for longer than 1sec
-turn thruster demand to the yaw rate, make the AUV yaw

# Modification
21/May/2015: have the AUV depth (Z) reset to zero when there is no depth demand updated for longer than 1sec

"""

import rospy
import numpy
import time
import math
from hardware_interfaces.msg import compass
from hardware_interfaces.msg import depth
from hardware_interfaces.msg import position
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from hardware_interfaces.msg import tsl_setpoints
from hardware_interfaces.msg import tail_setpoints
from pylab import *

from delphin2_mission.utilities     import uti

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
    global thruster_cb
    headingDemand_cb = 0
    depthDemand_cb = 0
    rearPropDemand_cb = 0
    swayDemand_cb = 0
    thruster_cb = 0
    
    global hasHeadingDemand
    global hasDepthDemand
    global hasPropDemand
    global hasThDemand
    hasHeadingDemand = False
    hasDepthDemand = False
    hasPropDemand = False
    hasThDemand = False
    
    # increment
    incHeading = 10 # [(deg)/s] how quick the heading can change FIXME: tune the value
    incDepth = 0.01 # [(m)/s] how quick the depth can change FIXME: tune the value
    # saturation
    speedMax = 1 # [m/s] maximum surge speed
    swayMax = 0.5 # [m/s] maximum sway speed
    depthMax = rospy.get_param('over-depth') # [m] maximum depth
    dt = 0.01 # [sec] time step size
    
    # nu: velocity vector
    u = 0 # [m/s] initial surge velocity
    
    # eta: state vector
    X = 0 # [m]
    Y = 0 # [m]
    Z = 0 # [m]
    heading = 0 # [deg] North CW
    depthTimeLastDemand = time.time()
    
    com = compass()
    dep = depth()
    pos = position()
    
    while not rospy.is_shutdown():
        
        # get demand
        demandRearProp = rearPropDemand_cb # [m/s]
        demandDepth = depthDemand_cb # [m]
        demandHeading = headingDemand_cb # [deg] North CW
        demandSway = swayDemand_cb # [m/s] stb
        
        # update kinematic parameters and state vector
        if hasPropDemand:
            if demandRearProp < 10:
                u = 0 # propeller deadband
            else:
                u = demandRearProp*1./22. # estimated from Leo's thesis
            u = myUti.limits(u,0,speedMax)
            X = X+u*dt*sin(heading*pi/180)
            Y = Y+u*dt*cos(heading*pi/180)
            hasPropDemand = False
            
        # a priority of actuator demand heading>thrusterSway>thrusterRudderYaw (note: this is not real!)
        # yaw due to heading demand
        if hasHeadingDemand:
            errHeading = myUti.computeHeadingError(demandHeading,heading)
            heading = mod( heading+sign(errHeading)*incHeading*dt, 360 )
            hasHeadingDemand = False
        # sway and yaw due to thruster demand
        elif hasThDemand:
            #sway due to thruster
            if th_hori_frt*th_hori_aft>0: # if thrusters are spining in a same direction, do sway
                v = myUti.limits(float(th_hori_frt)/2200.*swayMax,-swayMax,swayMax)
                X = X+v*dt*cos(heading*pi/180)
                Y = Y+v*dt*sin(heading*pi/180)
            #yaw due to thruster and rudder
            elif th_hori_frt*th_hori_aft<=0: # if thrusters are spining in a different direction, do yaw
                incHeadingTH = float(th_hori_frt)/2200.*incHeading*dt
                incHeadingCS = float(csAngle)/30.*incHeading*dt
                heading = mod( heading+incHeadingTH+incHeadingCS, 360 )
            hasThDemand = True
            
        if hasDepthDemand:
            errDepth = demandDepth-Z
            Z = Z + sign(errDepth)*incDepth*dt
            Z = myUti.limits(Z,0,depthMax)
            hasDepthDemand = False
            depthTimeLastDemand = time.time()
        else:
            # reset depth if there is no depth demand for longer than 1 sec
            if time.time()-depthTimeLastDemand > 1:
                Z = 0

#        print demandRearProp, u
#        print X, Y, Z, heading, u
#        print demandSpeed, demandHeading, demandDepth
                
        com.heading = heading
        dep.depth_filt = Z
        
        pos.X = X
        pos.Y = Y
        pos.Z = Z
        pos.forward_vel = u
        
        # publish
        pubCompass.publish(com)
        pubDepth.publish(dep)
        pubPosition.publish(pos)
#        time.sleep(0.0001)

################################################################################
######## SATURATION AND UPDATE PARAMETERS FROM TOPICS ##########################
################################################################################
   
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

def heading_demand_cb(headingd):
    global headingDemand_cb
    global hasHeadingDemand
    headingDemand_cb = headingd.data
    hasHeadingDemand = True
    
def th_horiz_cb(new_sp):
    global th_hori_frt
    global th_hori_aft
    global hasThDemand
    th_hori_frt = new_sp.thruster0
    th_hori_aft = new_sp.thruster1
    hasThDemand = True
    
def vertical_cb(new_angles):
    global csAngle
    csAngle = new_angles.cs0 # assume the command on top and bottom surface are identical
    # There is no need to have "hasCsDemand" to check if the demand is available since the thruste and control surface demand are specified at the same place
    
################################################################################
######## INITIALISATION ########################################################
################################################################################

if __name__ == '__main__':
    time.sleep(1) #Allow System to come Online
    rospy.init_node('auvsim')
    
    global pubCompass
    global pubDepth
    global pubPosition
    
    pubCompass = rospy.Publisher('compass_out', compass)
    pubDepth = rospy.Publisher('depth_out', depth)
    pubPosition = rospy.Publisher('position_dead', position)
    
    rospy.Subscriber('heading_demand', Float32, heading_demand_cb)
    rospy.Subscriber('depth_demand', Float32, depth_demand_cb)
    rospy.Subscriber('prop_demand', Int8, rearProp_demand_cb)
    rospy.Subscriber('TSL_setpoints_horizontal', tsl_setpoints, th_horiz_cb)
    rospy.Subscriber('tail_setpoints_vertical', tail_setpoints, vertical_cb)
        
    listenForData()   #Main loop for update the AUV parameters
