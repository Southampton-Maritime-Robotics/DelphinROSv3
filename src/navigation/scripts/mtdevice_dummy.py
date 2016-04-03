#!/usr/bin/env python

"""
A node to publish a fake compass information that is estimated using mathematical model of the AUV. It is mainly used when testing the dead_reckoner.

"""

import rospy
import time

from hardware_interfaces.msg import status
from hardware_interfaces.msg import compass

##############################
# # # XSensDriver object # # #
##############################
def main_loop():
    controlRate = 20.
    r = rospy.Rate(controlRate)
    
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
            pubStatus.publish(nodeID = 6, status = True)
            
        comOut = compass()
        comOut.roll = 1
        comOut.pitch = -2.0
        comOut.heading = 3.05
        pubCompassOut.publish(comOut)
        r.sleep()
        
            
if __name__=='__main__': 
    time.sleep(1) #Allow System to come Online
    rospy.init_node('xsens_driver')

    pubCompassOut = rospy.Publisher('compass_out',compass)
    pubStatus = rospy.Publisher('status', status)
    
    main_loop()
