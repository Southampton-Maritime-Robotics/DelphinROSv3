#!/usr/bin/python

import rospy
import serial
import time
import numpy
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

def setupSerial():
    global serialPort
    serialPort = serial.Serial(port='/dev/usbtail', baudrate='38400', timeout=0.2)
    serialPort.bytesize = serial.EIGHTBITS
    serialPort.stopbits = serial.STOPBITS_ONE
    serialPort.parity = serial.PARITY_NONE
    time.sleep(3)
    return serialPort.isOpen()

################################################################################

def tail_section_loop(status):
    global message
    global b_top
    global c_starb
    global d_bottom
    global e_port
    global prop
    
    freq = 10
    dt = 0
    prop_full_time = 1.0
    prop_on = 0
    
    b_top = 90
    c_starb = 75
    d_bottom = 90
    e_port = 70
    prop = 94
    
################################################################################
################################################################################
        
    while not rospy.is_shutdown():
        
        pubStatus.publish(nodeID = 2, status = status)
        
        time_zero = time.time()
        prop_on = prop_on + dt
        
        if time.time()-timeLastDemandProp>timeLastDemandMax:
            [prop, prop_on] = speedToSetpoint(0, prop_on, prop_full_time)
        else:
            [prop, prop_on] = speedToSetpoint(prop_demand, prop_on, prop_full_time)
        if time.time()-timeLastDemandCsHor>timeLastDemandMax:
            c_starb = angleToSetpoint(0, 70)
            e_port = angleToSetpoint(0, 70)
        if time.time()-timeLastDemandCsVer>timeLastDemandMax:
            b_top = angleToSetpoint(0, 70)
            d_bottom = angleToSetpoint(0, 70)
            c_starb = angleToSetpoint(0, 70)
            e_port = angleToSetpoint(0, 70)
            

        
        message = 'b%03dc%03dd%03de%03df%03d' %(b_top, c_starb, d_bottom , e_port, prop)
        #print 'message: ',message
        try:
            serialPort.write(message)   #POSSIBLE ISSUE STILL FAILS EVEN THIUGH IN TRY!
        except:
            print 'Write error'
    
        try:
            feedback = serialPort.read(25)
            process_feedback(feedback)
        except:
            print 'read error'
        
        dt = time.time() - time_zero
        
        while dt < (1/freq):
            dt = time.time() - time_zero
            time.sleep(0.01)
            
################################################################################
################################################################################

def tailAlive():
    global serialPort
    attempts = 1
    serialPort.write("s")       
    response = serialPort.read(1)
    
    while response != 'R' and attempts <= 5:
        serialPort.write("s") 
        response = serialPort.read(1)
        attempts = attempts + 1
        
    if response == 'R':
        return True
    else:
        return False
    
################################################################################
    
def angleToSetpoint(angle, zero):
    setpoint = zero - (1.832 * angle)
    
    maxlimit = 140
    minlimit = 10
        
    if setpoint > maxlimit:
        setpoint = maxlimit
    elif setpoint < minlimit:
        setpoint = minlimit
        
    return int(setpoint)

################################################################################

def speedToSetpoint(prop_demand, prop_on, prop_full_time):

    prop = prop_demand
    
    if abs(prop) < 10 and prop != 0:
        if prop_on <= (prop_demand*prop_full_time/10):
            prop = 10*numpy.sign(prop)
        else:
            prop = 0
        if prop_on > prop_full_time:
            prop_on = 0
    
    setpoint = 94 - prop
    
    maxlimit = 150
    minlimit = 40
    
    if setpoint > maxlimit:
        setpoint = maxlimit
    elif setpoint < minlimit:
        setpoint = minlimit
   
    return [int(setpoint), prop_on]

################################################################################

def horizontal_callback(new_angles):
    global message
    global c_starb
    global e_port
    global c
    global e
    global timeLastDemandCsHor
    
    timeLastDemandCsHor = time.time()
    
    c = new_angles.cs0
    e = new_angles.cs1
    
    c_starb = angleToSetpoint(new_angles.cs0, 70)
    e_port  = angleToSetpoint(-new_angles.cs1, 70)
    
################################################################################
    
def vertical_callback(new_angles):
    global message
    global b_top
    global d_bottom
    global b
    global d
    global timeLastDemandCsVer
    
    timeLastDemandCsVer = time.time()
        
    b = new_angles.cs0
    d = new_angles.cs1
    
    b_top    = angleToSetpoint(-new_angles.cs0, 80)
    d_bottom = angleToSetpoint(new_angles.cs1, 80)

################################################################################
    
def prop_callback(new_prop):
    global prop_demand    
    global timeLastDemandProp
    
    timeLastDemandProp = time.time()
    prop_demand = new_prop.data
    print 'Prop dmenad = ',prop_demand

################################################################################

def process_feedback(feedback):
     global pub
     global prop_demand
     global b
     global c
     global d
     global e     
     
     b_fb = -0.2073 * float(feedback[0:4]) + 100.07
     c_fb = 0.2088 * float(feedback[4:8]) - 101.92
     d_fb = 0.2055 * float(feedback[8:12]) - 100.37
     e_fb = -0.2117 * float(feedback[12:16]) + 103.03
     
     prop_current = (float(feedback[17:20])-512)*0.0763
     
     if abs(prop_demand) > 0:
        raw = float(feedback[21:24])
        prop_rpm = ((1-1/numpy.exp(0.05*(raw-340)))*(raw*0.45)-80)*numpy.sign(prop_demand)
     else:
        prop_rpm = 0
        
     output = tail_feedback()
     output.bsp = b
     output.b = b_fb
     output.csp = c
     output.c = c_fb
     output.dsp = d
     output.d = d_fb
     output.esp = e
     output.e = e_fb
     output.propSP = prop_demand
     output.current = prop_current
     output.rpm = prop_rpm
     pub.publish(output)
     
     print 'Feedback', output
    
################################################################################

def shutdown():
    pubStatus.publish(nodeID = 2, status = False)
    message = 'b080c070d080e070f094'
    serialPort.flushOutput()
    serialPort.write(message)

################################################################################
################################################################################

if __name__ == '__main__':
    time.sleep(1) #Allow System to come Online    
    global pub
    global prop_demand
    global serialPort
    global b
    global c
    global d
    global e
    
    [b, c, d, e] = (0, 0, 0, 0)
    
    pub = rospy.Publisher('tail_output', tail_feedback, queue_size=3)
    pubStatus = rospy.Publisher('status', status, queue_size=3)
    
    prop_demand = 0
    
    rospy.init_node('tail_section')
    rospy.on_shutdown(shutdown)
    
    port_status = setupSerial()
    str = "Tailsection serial port status = %s. Port = %s" %(port_status, serialPort.portstr)
    
    rospy.Subscriber('tail_setpoints_horizontal', tail_setpoints, horizontal_callback)
    rospy.Subscriber('tail_setpoints_vertical', tail_setpoints, vertical_callback)
    rospy.Subscriber('prop_demand', Int8, prop_callback)

    
    if (port_status and tailAlive()) == True:    
        status = True
        pubStatus.publish(nodeID = 2, status = status)
        rospy.loginfo("Tail section now online")
    else:
        status = False
        pubStatus.publish(nodeID = 2, status = status)
    
    tail_section_loop(status)
 

