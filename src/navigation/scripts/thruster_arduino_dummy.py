#!/usr/bin/python

"""
A node that subscribe to the thruster demand.
These will get published to the TSL_feedback topic and recorded by the logger.

"""

import rospy
import time
import numpy
from hardware_interfaces.msg import tsl_setpoints
from hardware_interfaces.msg import tsl_feedback
from hardware_interfaces.msg import status

from std_msgs.msg import String

global new_data
global current_data

global ThrusterSetpoint_max
ThrusterSetpoint_max = 2500 # a limit in motor speed in a range of [0-2500] rpm %
global thrust_dr_ref
thrust_dr_ref = [0,0,0,1] # a vector to correct thrust_direction
global timeLastDemand_sat
timeLastDemand_sat = 1 # [sec] associated thruster will be stopped if there is no new demand for longer than this many second

############################# MAIN LOOP ############################################    
def motor_control():

    global onOff_horiz
    global onOff_vert
    global current_data
    global delaySerial
    global ThrusterSetpoint_max
    global timeHorizLastDemand
    global timeVertLastDemand
    onOff_horiz = 0
    onOff_vert = 0
    sp_max = 2500.
    sp_min = 145.
    sp_max_arduino = 255.
    slope = sp_max_arduino/(sp_max-sp_min)
    interception = -slope*sp_min
    timeHorizLastDemand = time.time()
    timeVertLastDemand = time.time()

    timeRef = time.time()
    timeStart_rpm = [timeRef,timeRef,timeRef,timeRef]
    timeLim_rpm = 5 # [sec]
    timeElapse_rpm = [0.,0.,0.,0.]
    
    controlRate = 5. # [Hz] limited by the rate of tail section
    controlPeriod = 1./controlRate
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
            pubStatus.publish(nodeID = 1, status = True)
        
        timeRef = time.time()
        
        data_now = current_data #Get the data at the current time. This is in the range of [-2500,2500]
        
        ############################# ON/OFF MSG ######################################    
        thrust_sp = [data_now['thruster0']*onOff_vert, data_now['thruster1']*onOff_vert, data_now['thruster2']*onOff_horiz, data_now['thruster3']*onOff_horiz]
        
        ############################# REMAP THRUSTER SETPOINT ######################################    
        thrust_dr = thrust_dr_ref[:] # create a deep copy of a vector to correct thrust_direction
        thrust_sp_arduino = [0,0,0,0]
        # remap a setpoint from [-2500,-150] U [150,2500] to [0,255] with direction of 0 or 1
        for i in range(len(thrust_sp)):
            # correct direction vector
            if thrust_sp[i] > 0:
                thrust_dr[i] = 1-thrust_dr[i]
                
            # apply setpoint deadband and saturation
            if abs(thrust_sp[i]) < sp_min:
                thrust_sp[i] = 0
            elif abs(thrust_sp[i]) > ThrusterSetpoint_max:
                thrust_sp[i] = ThrusterSetpoint_max*numpy.sign(thrust_sp[i])
                
            # remap from setpoint to arduino signal
            if thrust_sp[i] !=0:
                thrust_sp_arduino[i] = slope*abs(thrust_sp[i])+interception
                
            # double check if the demand is interger and is within a valid range
            if thrust_sp_arduino[i] > 0:
                if thrust_sp_arduino[i] <1: # if a setpoint is not zero but less than 1, force it to one
                    thrust_sp_arduino[i] = 1
                else:
                    thrust_sp_arduino[i] = int(thrust_sp_arduino[i])

        # the smallest value that setpoint in arduino world could be is 1 in whih correspoinds to a RPM of 145 approximately
                
        ############################# SETPOINT MSG ####################################    

        # if there is no update on vert thruster for longer than timeLastDemand_sat, turn off the vert thruster
        if time.time()-timeVertLastDemand > timeLastDemand_sat:
            onOff_vert = 0
        setpoint0= onOff_vert*thrust_sp_arduino[0]
        setpoint1= onOff_vert*thrust_sp_arduino[1]
        
        # if there is no update on horiz thruster for longer than timeLastDemand_sat, turn off the horiz thruster
        if time.time()-timeHorizLastDemand > timeLastDemand_sat:
            onOff_horiz = 0
        setpoint2= onOff_horiz*thrust_sp_arduino[2]
        setpoint3= onOff_horiz*thrust_sp_arduino[3]
                
        speed0 = 0
        speed1 = 0
        speed2 = 0
        speed3 = 0
        
        pub.publish(setpoint0 = thrust_sp[0], 
                    setpoint1 = thrust_sp[1], 
                    setpoint2 = thrust_sp[2], 
                    setpoint3 = thrust_sp[3],
                    speed0 = speed0, 
                    speed1 = speed1, 
                    speed2 = speed2, 
                    speed3 = speed3)
        
        timeElapse = time.time()-timeRef
        if timeElapse < controlPeriod:
            r.sleep()
        else:
            str = "Arduino_thruster rate does not meet the desired value of %.2fHz: actual control rate is %.2fHz" %(controlRate,1/timeElapse) 
            rospy.logwarn(str)
            pubMissionLog.publish(str)
    
############################# CALLBACKS ######################################    

def vert_callback(new_sp):
    global current_data
    global timeVertLastDemand
    global onOff_vert
    onOff_vert = 1
    current_data['thruster0'] = new_sp.thruster0
    current_data['thruster1'] = new_sp.thruster1
    timeVertLastDemand = time.time()

def horiz_callback(new_sp):
    global current_data
    global timeHorizLastDemand
    global onOff_horiz
    onOff_horiz = 1
    current_data['thruster2'] = new_sp.thruster0
    current_data['thruster3'] = new_sp.thruster1
    timeHorizLastDemand = time.time()

def shutdown():
    pubStatus.publish(nodeID = 1, status = False)

############################# SETUP ######################################    

if __name__ == '__main__':
    time.sleep(1) #Allow System to come Online
    rospy.init_node('Arduino_Thruster')
    
    global current_data
    current_data = {'thruster0':0,'thruster1':0,'thruster2':0,'thruster3':0}
    
    rospy.Subscriber('TSL_setpoints_horizontal', tsl_setpoints, horiz_callback)
    rospy.Subscriber('TSL_setpoints_vertical', tsl_setpoints, vert_callback)
    
    pub = rospy.Publisher('TSL_feedback', tsl_feedback)
    pubStatus = rospy.Publisher('status', status)
    pubMissionLog = rospy.Publisher('MissionStrings', String)
    
    rospy.on_shutdown(shutdown) #Defining shutdown behaviour

    motor_control() # entering the main loop
