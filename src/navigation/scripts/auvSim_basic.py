#!/usr/bin/python

"""
# Description
Simulator to the AUV kinematic.

This node subscribes to commands.
A state vector of the virtual AUV is updated accordingly based on the kinematic equation, i.e. the inertia is neglected.
The current AUV state is then published as dummy messages of compass_out, depth_out and position_dead.

#Available commands
-self.delay_action: how long the action will be held
-self.__controller.setHeading(0)
-self.__controller.setDepth()
-self.__controller.setRearProp(0)
-self.__controller.setControlSurfaceAngle(0,0,0,0) # (VerUp,HorRight,VerDown,HorLeft) # only the vertical ones that work
-self.__controller.setArduinoThrusterHorizontal(0,0) # (FrontHor,RearHor)

#note
-if the actuator demands are assigned directly, the heading demand has to be removed

#Todo
-make the AUV drive through the thruster demand instead of depth demand

# Modification
21/May/2015: have the AUV depth (Z) reset to zero when there is no depth demand updated for longer than 1sec
22/May/2015: turn the thruster and rudder demands into yaw and sway motion

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
    global csAngle_cb
    headingDemand_cb = 0
    depthDemand_cb = 0
    rearPropDemand_cb = 0
    swayDemand_cb = 0
    thruster_cb = tsl_setpoints()
    csAngle_cb = 0

    # watchdog to turn demand off
    global timeLastDemand_heading
    global timeLastDemand_depth
    global timeLastDemand_prop
    global timeLastDemand_th_hori
    global timeLastDemand_cs_vert
    global timeLastDemand_max
    timeLastDemand_heading = time.time()
    timeLastDemand_depth = time.time()
    timeLastDemand_prop = time.time()
    timeLastDemand_th_hori = time.time()
    timeLastDemand_cs_vert = time.time()
    timeLastDemand_max = 1 # [sec]

    # demand onOff flag    
    global haveDemand_heading
    global hasDemand_depth
    global haveDemand_prop
    global haveDemand_th
    global haveDemand_cs
    haveDemand_heading = False
    haveDemand_depth = False
    haveDemand_prop = False
    haveDemand_th = False
    haveDemand_cs = False
    
    # loop timing
    controlRate = 100. # [Hz]
    controlPeriod = 1./controlRate
    r = rospy.Rate(controlRate)
    
    # increment
    incHeading = 5 # [(deg)/s] how quick the heading can change FIXME: tune the value
    incDepth = 0.1 # [(m)/s] how quick the depth can change FIXME: tune the value
    # saturation
    speedMax = 1. # [m/s] maximum surge speed
    swayMax = 0.5 # [m/s] maximum sway speed
    depthMax = rospy.get_param('over-depth') # [m] maximum depth
    dt = 0.02 # [sec] fake time step size
    
    # nu: velocity vector
    u = 0 # [m/s] initial surge velocity
    
    # eta: state vector
    X = 1 # [m]
    Y = 1 # [m]
    Z = 0 # [m]
    heading = 0 # [deg] North CW
    
    com = compass()
    dep = depth()
    pos = position()

    demandRearProp = 0
    th_hori_frt = 0
    th_hori_aft = 0
    csAngle = 0

    
    while not rospy.is_shutdown():
        
        timeRef = time.time()
        
        # get demand
        if time.time()-timeLastDemand_heading<timeLastDemand_max:
            demandHeading = headingDemand_cb # [deg] North CW
            haveDemand_heading = True
        else:
            haveDemand_heading = False

        if time.time()-timeLastDemand_depth<timeLastDemand_max:
            demandDepth = depthDemand_cb # [m]
            haveDemand_depth = True
        else:
            haveDemand_depth = False

        if time.time()-timeLastDemand_prop<timeLastDemand_max:
            demandRearProp = rearPropDemand_cb # [-]
            haveDemand_prop = True
        else:
            haveDemand_prop = False

        if time.time()-timeLastDemand_th_hori<timeLastDemand_max:
            th_hori_frt = thruster_cb.thruster0
            th_hori_aft = thruster_cb.thruster1
            haveDemand_th = True
        else:
            haveDemand_th = False

        if time.time()-timeLastDemand_cs_vert<timeLastDemand_max:
            csAngle = csAngle_cb
            haveDemand_cs = True
        else:
            haveDemand_cs = False
            
        # update kinematic parameters and state vector
        if haveDemand_prop:
            if demandRearProp < 10:
                u = 0 # propeller deadband
            else:
                u = demandRearProp*1./22. # estimated from Leo's thesis
            u = myUti.limits(u,0,speedMax)
            X = X+u*dt*sin(heading*pi/180)
            Y = Y+u*dt*cos(heading*pi/180)

        # a priority of actuator demand heading>thrusterSway>thrusterRudderYaw (note: this is not real!)
        # yaw due to heading demand
        v = 0
        if haveDemand_heading:
            errHeading = myUti.computeHeadingError(demandHeading,heading)
            heading = mod( heading+sign(errHeading)*incHeading*dt, 360 )
        # sway and yaw due to thruster demand
        elif haveDemand_th or haveDemand_cs:
            #sway due to thruster
            if th_hori_frt*th_hori_aft>0: # if thrusters are spining in a same direction, do sway
                v = myUti.limits(float(th_hori_frt)/2550.*swayMax,-swayMax,swayMax)
                X = X+v*dt*cos(heading*pi/180)
                Y = Y-v*dt*sin(heading*pi/180)
            #yaw due to thruster and rudder
            elif th_hori_frt*th_hori_aft<=0: # if thrusters are spining in a different direction, do yaw
                incHeadingTH = float(th_hori_frt)/2200.*incHeading*dt
                if haveDemand_prop:
                    incHeadingCS = float(csAngle)/30.*incHeading*dt
                else:
                    incHeadincCS = 0
                heading = mod( heading+incHeadingTH+incHeadingCS, 360 )
             
        if haveDemand_depth:
            errDepth = demandDepth-Z
            Z = Z + sign(errDepth)*incDepth*dt
            Z = myUti.limits(Z,0,depthMax)
        else:
            Z = 0
                
        com.heading = heading
        dep.depth_filt = Z
        
        pos.X = X
        pos.Y = Y
        pos.Z = Z
        pos.forward_vel = u
        pos.sway_vel = v
        
        # publish
        pubCompass.publish(com)
        pubDepth.publish(dep)
        pubPosition.publish(pos)
        
        timeElapse = time.time()-timeRef
        if timeElapse < controlPeriod:
            r.sleep()
        else:
            str = "auv_sim rate does not meet the desired value of %.2fHz: actual control rate is %.2fHz" %(controlRate,1/timeElapse) 
            rospy.logwarn(str)

################################################################################
######## SATURATION AND UPDATE PARAMETERS FROM TOPICS ##########################
################################################################################
   
def rearProp_demand_cb(propd):
    global rearPropDemand_cb
    global timeLastDemand_prop
    rearPropDemand_cb = propd.data
    timeLastDemand_prop = time.time()
    
def depth_demand_cb(depthd):
    global depthDemand_cb
    global timeLastDemand_depth
    depthDemand_cb = depthd.data
    timeLastDemand_depth = time.time()

def heading_demand_cb(headingd):
    global headingDemand_cb
    global timeLastDemand_heading
    headingDemand_cb = headingd.data
    timeLastDemand_heading = time.time()
    
def th_horiz_cb(new_sp):
    global thruster_cb
    global timeLastDemand_th_hori
    thruster_cb = new_sp
    timeLastDemand_th_hori = time.time()
    
def cs_vert_cb(new_angles):
    global csAngle_cb
    global timeLastDemand_cs_vert
    csAngle_cb = new_angles.cs0 # assume the command on top and bottom surface are identical
    timeLastDemand_cs_vert = time.time()
    
    # There is no need to have "hasCsDemand" to check if the demand is available since the thruster and control surface demand are specified at the same place
    
################################################################################
######## INITIALISATION ########################################################
################################################################################

if __name__ == '__main__':
    time.sleep(1) #Allow System to come Online
    rospy.init_node('auvsim')
    
    pubCompass = rospy.Publisher('compass_out', compass)
    pubDepth = rospy.Publisher('depth_out', depth)
    pubPosition = rospy.Publisher('position_dead', position)
    
    rospy.Subscriber('heading_demand', Float32, heading_demand_cb)
    rospy.Subscriber('depth_demand', Float32, depth_demand_cb)
    rospy.Subscriber('prop_demand', Int8, rearProp_demand_cb)
    rospy.Subscriber('TSL_setpoints_horizontal', tsl_setpoints, th_horiz_cb)
    rospy.Subscriber('tail_setpoints_vertical', tail_setpoints, cs_vert_cb)
        
    listenForData()   #Main loop for update the AUV parameters
