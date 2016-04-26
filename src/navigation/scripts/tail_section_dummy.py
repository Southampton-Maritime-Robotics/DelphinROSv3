#!/usr/bin/python

"""
A node that subscribe to the actuator demand for the tail section.
These will get published to the tail_output topic and recorded by the logger.

"""

import rospy
import time
from std_msgs.msg import String
from std_msgs.msg import Int8
from hardware_interfaces.msg import tail_setpoints
from hardware_interfaces.msg import tail_feedback
from hardware_interfaces.msg import status

################################################################################
################################################################################

global timeLastDemandProp
global timeLastDemandCsHor
global timeLastDemandCsVer
global timeLastDemandMax
timeLastDemandProp = time.time()
timeLastDemandCsHor = time.time()
timeLastDemandCsVer = time.time()
timeLastDemandMax = 1 # if there is no new demand for this many seconds, the corresponding actuators will be turned off of set to neutral position
timeLastCommunicationMax = 1

def tail_section_loop(status):
    global b_demand
    global c_demand
    global d_demand
    global e_demand
    global prop_demand

    controlRate = 5. # [Hz] limited by the response rate from arduino in tail section
    controlPeriod = 1/controlRate
    r = rospy.Rate(controlRate)
    
################################################################################
################################################################################
    finsDemand_lim = 30
    
################################################################################
################################################################################

    # to control a timing for status publishing
    timeZero_status = time.time()
    try:
        dt_status = rospy.get_param('status_timing')
    except:
        dt_status = 2.

    while not rospy.is_shutdown():
        # to control a timing for status publishing
        if time.time()-timeZero_status > dt_status:
            timeZero_status = time.time()
            pubStatus.publish(nodeID = 2, status = True)

        timeRef = time.time()

        # watchdog to set prop and fin demands to zero when there is no demand updated for some time
        if time.time()-timeLastDemandCsHor>timeLastDemandMax:
            [c_demand, e_demand] = [0, 0]
        if time.time()-timeLastDemandCsVer>timeLastDemandMax:
            [b_demand, d_demand] = [0, 0]
        if time.time()-timeLastDemandProp>timeLastDemandMax:
            prop_demand = 0
            
        # apply limit to the fin angle demands
        b_demand = limits(b_demand,-finsDemand_lim,finsDemand_lim)
        c_demand = limits(c_demand,-finsDemand_lim,finsDemand_lim)
        d_demand = limits(d_demand,-finsDemand_lim,finsDemand_lim)
        e_demand = limits(e_demand,-finsDemand_lim,finsDemand_lim)

        [b_feedback, c_feedback, d_feedback, e_feedback, prop_rps] = [0, 0, 0, 0, 0]

        ############################# PUBLISH THE INFORMATION ######################################
        pub.publish(b_sp = b_demand,
                    b_fb = b_feedback,
                    c_sp = c_demand,
                    c_fb = c_feedback,
                    d_sp = d_demand,
                    d_fb = d_feedback,
                    e_sp = e_demand,
                    e_fb = e_feedback,
                    prop_sp = prop_demand,
                    prop_rps = prop_rps)
    
        # verify and maintain the loop timing                  
        timeElapse = time.time()-timeRef
        if timeElapse < controlPeriod:
            r.sleep()
        else:
            str = "tail_section rate does not meet the desired value of %.2fHz: actual control rate is %.2fHz" %(controlRate,1/timeElapse) 
            rospy.logwarn(str)
            pubMissionLog.publish(str)
            
################################################################################
################################################################################
    
################################################################################
    
def horizontal_callback(new_angles):
    global c_demand
    global e_demand
    global timeLastDemandCsHor
    
    c_demand = new_angles.cs0
    e_demand = -new_angles.cs1
    timeLastDemandCsHor = time.time()
        
################################################################################
    
def vertical_callback(new_angles):
    global b_demand
    global d_demand
    global timeLastDemandCsVer

    b_demand = -new_angles.cs0
    d_demand = new_angles.cs1
    timeLastDemandCsVer = time.time()

################################################################################
    
def prop_callback(new_prop):
    global prop_demand    
    global timeLastDemandProp
    
    prop_demand = new_prop.data
    timeLastDemandProp = time.time()
    ################################################################################

def limits(value, value_min, value_max):       #Function to contrain within defined limits
    if value < value_min:				   
       value = value_min
    elif value > value_max:
       value = value_max
    return value

################################################################################

def shutdown():
    pubStatus.publish(nodeID = 2, status = False)

################################################################################
################################################################################

if __name__ == '__main__':
    time.sleep(1) #Allow System to come Online
    
    pub = rospy.Publisher('tail_output', tail_feedback)
    pubStatus = rospy.Publisher('status', status)
    pubMissionLog = rospy.Publisher('MissionStrings', String)
    
    rospy.init_node('tail_section')
    rospy.on_shutdown(shutdown)
    
    rospy.Subscriber('tail_setpoints_horizontal', tail_setpoints, horizontal_callback)
    rospy.Subscriber('tail_setpoints_vertical', tail_setpoints, vertical_callback)
    rospy.Subscriber('prop_demand', Int8, prop_callback)
    
    tail_section_loop(status)
