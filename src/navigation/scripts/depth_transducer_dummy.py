#!/usr/bin/python

"""
A node that update a current depth of the AUV in according to the depth demand by using a first order transfer function.
If a new depth demand has not been received for some period of time, the demand will be set to zero and the AUV will start surfacing.

NB: the depth extimated here is so wrong and should not be thrusted at all; even so, it is very useful when testing a state machine.

"""

import rospy
import numpy
import time
from std_msgs.msg import String
from std_msgs.msg import Float32
from hardware_interfaces.msg import depth
from hardware_interfaces.msg import status

################################################################

def depthUpdater(y,x):
    # Y(s)/X(s) = 1/(tS+1)
    t = 5
    y_dot = x/t - y/t

    return y_dot

def listenForData():
    depthNow = 0
    depthDemandNow = 0

    global depthDemand_cb
    global timeLastDemand_depth
    depthDemand_cb = depthNow
    timeLastDemand_depth = time.time()
    
    depth_msg = depth()  
    timelastDemand_max = 1 # [sec]  
    
    controlRate = 10. # [Hz]
    controlPeriod = 1./controlRate
    dt = controlPeriod
    r = rospy.Rate(controlRate)
    
    # to control a timing for status publishing
    timeZero_status = time.time()
    try:
        dt_status = rospy.get_param('status_timing')
    except:
        dt_status = 2.
    
    while not rospy.is_shutdown():    
        timeRef = time.time()
        
        if time.time()-timeZero_status > dt_status:
            timeZero_status = time.time()
            pubStatus.publish(nodeID = 5, status = True)
            
        if time.time()-timeLastDemand_depth > timelastDemand_max:
            depthDemandNow = 0
        else:
            depthDemandNow = depthDemand_cb
        
        # form the state vector and implement Runge-Kutta 4th order
        k1 = depthUpdater(depthNow,depthDemandNow)
        k2 = depthUpdater(depthNow+dt/2.*k1,depthDemandNow)
        k3 = depthUpdater(depthNow+dt/2.*k2,depthDemandNow)
        k4 = depthUpdater(depthNow+dt*k3,depthDemandNow)
        
        depth_change = dt/6.*(k1+2*k2+2*k3+k4)
        depthNow = depthNow + depth_change
        
        depth_msg.depth_filt = depthNow
        
        #Publish data
        pub.publish(depth_msg)
        
        timeElapse = time.time()-timeRef
        if timeElapse < controlPeriod:
            rateOK = True
            r.sleep()
        else:
            rateOK = False
            str = "depth_transducer rate does not meet the desired value of %.2fHz: actual control rate is %.2fHz" %(controlRate,1/timeElapse) 
            rospy.logwarn(str)
            pubMissionLog.publish(str)

def depth_demand_cb(depthd):
    global depthDemand_cb
    global timeLastDemand_depth
    depthDemand_cb = depthd.data
    timeLastDemand_depth = time.time()
    
################################################################
def shutdown():
    pubStatus.publish(nodeID = 5, status = False)

################################################################        
#     INITIALISE     ###########################################
################################################################

if __name__ == '__main__':
    time.sleep(1) #Allow System to come Online    
    rospy.init_node('depth_transducer')
    
    pub = rospy.Publisher('depth_out', depth)
    pubMissionLog = rospy.Publisher('MissionStrings', String)
    rospy.Subscriber('depth_demand', Float32, depth_demand_cb)
    pubStatus = rospy.Publisher('status', status)
    
    rospy.on_shutdown(shutdown)
    
    time.sleep(0.3)
    
    listenForData()   #Main loop for receiving data
