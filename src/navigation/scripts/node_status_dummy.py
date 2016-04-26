#!/usr/bin/python

"""
A node to publish a fake auv system status as to get pass throught the INITIALISE state.

"""

import rospy
import time
from hardware_interfaces.msg import status
from hardware_interfaces.msg import energy_consumed
from std_msgs.msg import String

################### GLOBAL VARIABLES ################### 

def main():
    controlRate = 0.5 # Hz
    r = rospy.Rate(controlRate)
    controlPeriod = 1./controlRate
    while not rospy.is_shutdown():
        timeRef = time.time()
        pubStatus.publish(nodeID = 11, status = True)
        pubStatus.publish(nodeID = 12, status = True)
        
        pubOutput.publish(batteryVol = 99999, 
                          thruster_0 = 0, 
                          thruster_1 = 0, 
                          thruster_2 = 0,
                          thruster_3 = 0, 
                          fins = 0, 
                          prop = 0)
                          
        timeElapse = time.time()-timeRef
        if timeElapse < controlPeriod:
            r.sleep()
        else:
            str = "GPS rate does not meet the desired value of %.2fHz: actual control rate is %.2fHz" %(controlRate,1/timeElapse) 
            rospy.logwarn(str)

################### INITIALISING FUNCTION ################### 
if __name__ == '__main__':
    time.sleep(1) #Allow System to come Online
    
    rospy.init_node('node_status_dummy')
  
    #Define Publishers
    pubStatus = rospy.Publisher('status', status)
    pubOutput = rospy.Publisher('EnergyConsumed', energy_consumed)
    
    main()
