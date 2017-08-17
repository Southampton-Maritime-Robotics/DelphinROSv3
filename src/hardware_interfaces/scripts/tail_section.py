#!/usr/bin/python

"""
A driver to communicate with a tail unit that consists of control surfaces and propeller.

### Actuator Naming
b: top fin
c: starboard fin
d: bottom fin
e: port fin
prop: propeller

### usage
-to set
--send: $S{b_setpoint}@{c_setpoint}@{d_setpoint}@{e_setpoint}@{prop_setpoint}@#
--receive: n/a
-to get
--send: $R#
--receive: $b_feedback@c_feedback@d_feedback@e_feedback@prop_rps@#

#### Offset
to account for mechanical offsets of the sternplanes, a sofware offset is applied via ros parameters
In the published message it is applied in the setpoint.
In the feedback the offset is removed to represent the demand that was originally received in this node.
This is necessary in case other nodes check if the sternplane angle is reached.


###################################################
# TODO
- test if 5 sec delay to wait for other systems is necessary
"""

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
timeLastCommunicationMax = 1

# neutral position of fins and propeller demand
b_zero = 95 # [pwm]
c_zero = 75 # [pwm]
d_zero = 75 # [pwm]
e_zero = 75 # [pwm] #
finsDemand_lim = 30 # [deg], the magnet inside tail section may intercept if the bigger limit is used.
prop_zero = 94

class CS_config(object):
    def __init__(self):
        self.limit = 30  # [degrees]
        self.offset_port = rospy.get_param("/control_surface/offset/port")
        self.offset_stbd = rospy.get_param("/control_surface/offset/stbd")
        self.offset_top = rospy.get_param("/control_surface/offset/top")
        self.offset_bottom = rospy.get_param("/control_surface/offset/bottom")

def setupSerial():
    global serialPort
    serialPort = serial.Serial(port='/dev/usbtail', baudrate='9600', timeout=0.2)
    serialPort.bytesize = serial.EIGHTBITS
    serialPort.stopbits = serial.STOPBITS_ONE
    serialPort.parity = serial.PARITY_NONE
    serialPort.flushInput()
    serialPort.flushOutput()
    time.sleep(0.1)
    return serialPort.isOpen()

################################################################################
    
def getTailFeedback():
    # send a request
    try:
        str = '$R#'
        serialPort.write(str)
        time.sleep(0.01) # should be enough for arduino to get a ReqVoltage
    except:
        pass
        
    #initialize an empty string and 4x1 matrix filled with zero
    msg = ""
    data = [-1] * 10 # control surfaces b, c, d, e, and prop: arduino acknowledgement and feedback
    
    try:
        if serialPort.inWaiting(): # if arduino replied
            timeStart = time.time()
            timeElapse = time.time()-timeStart
            found = 0
            timeOut = 0.1 # timeout for serial communication[sec]
            while(timeElapse<timeOut) and not rospy.is_shutdown() and not found:
                if serialPort.inWaiting()>0: 
                    _in = serialPort.read(1)
                    if _in == '$': # searching for a header '$', and continue if found. otherwise, neglect this character
                        data_index = 0
                        timeStart = time.time()
                        timeElapse = time.time()-timeStart    
                        while(timeElapse<timeOut) and not rospy.is_shutdown() and not found: # get the rest of a data packet
                            if serialPort.inWaiting()>0:
                                _in = serialPort.read(1)
                                if _in=='#': # break if found the end of a data packet
                                    found = 1
                                elif _in=='@':
                                    data[data_index] = int(float(msg))
                                    data_index += 1
                                    msg = ""
                                    # found a delimiter. convert a message to int
                                else:    # keep the data in a msg string
                                    msg+=_in
                                timeElapse = time.time()-timeStart # elapsed time for getting the rest of data packet
                timeElapse = time.time()-timeStart # elapsed time for finding a

            if timeElapse>timeOut:
                pass
    except:
        pass
        
    if data[1] != -1:
        print(data[1]) # DDD
        data[1] = -setpointToAngle(data[1],b_zero)
    if data[3] != -1:
        data[3] = setpointToAngle(data[3],c_zero)
    if data[5] != -1:
        data[5] = setpointToAngle(data[5],d_zero)
    if data[7] != -1:
        data[7] = -setpointToAngle(data[7],e_zero)
    if data[9] != -1:
        data[9] = data[9]/19. # gearbox ratio

    return data

################################################################################

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

    timeLastWrite = time.time()
    timeLastRead = time.time() 
    
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
        
        # apply offsets first
        b_demand += cs_cfg.offset_top
        c_demand += cs_cfg.offset_stbd
        d_demand += cs_cfg.offset_bottom
        e_demand += cs_cfg.offset_port

        # apply limit to the fin angle demands
        b_demand = limits(b_demand,-finsDemand_lim,finsDemand_lim)
        c_demand = limits(c_demand,-finsDemand_lim,finsDemand_lim)
        d_demand = limits(d_demand,-finsDemand_lim,finsDemand_lim)
        e_demand = limits(e_demand,-finsDemand_lim,finsDemand_lim)
        
        # determine setpoint
        b_setpoint = angleToSetpoint(b_demand, b_zero)
        c_setpoint = angleToSetpoint(c_demand, c_zero)
        d_setpoint = angleToSetpoint(d_demand, d_zero)
        e_setpoint = angleToSetpoint(e_demand, e_zero)
        prop_setpoint = speedToSetpoint(prop_demand, prop_zero)
        
        message = '$S%d@%d@%d@%d@%d@#' %(b_setpoint, c_setpoint, d_setpoint , e_setpoint, prop_setpoint)
        try:
            serialPort.write(message)
            timeLastWrite = time.time()
        except:
            rospy.logwarn('Arduino write error')

        [b_arduino, b_feedback, 
         c_arduino, c_feedback, 
         d_arduino, d_feedback, 
         e_arduino, e_feedback, 
         prop_arduino, prop_rps] = [-2] * 10  # arduino has not been read
        try:
            [b_arduino, b_feedback, 
            c_arduino, c_feedback, 
            d_arduino, d_feedback, 
            e_arduino, e_feedback, 
            prop_arduino, prop_rps] = getTailFeedback()
            timeLastRead = time.time()
        except:
            rospy.logwarn('arduino read error')
               
        ############################# PUBLISH THE INFORMATION ######################################
        # remove offsets in the feedback
        b_feedback -= cs_cfg.offset_top
        c_feedback -= cs_cfg.offset_stbd
        d_feedback -= cs_cfg.offset_bottom
        e_feedback -= cs_cfg.offset_port

        pub.publish(b_sp = b_demand,
                    b_ack = b_arduino,
                    b_fb = b_feedback,
                    c_sp = c_demand,
                    c_ack = c_arduino,
                    c_fb = c_feedback,
                    d_sp = d_demand,
                    d_ack = d_arduino,
                    d_fb = d_feedback,
                    e_sp = e_demand,
                    e_ack = e_arduino,
                    e_fb = e_feedback,
                    prop_sp = prop_demand,
                    prop_ack = prop_arduino,
                    prop_rps = prop_rps)

        ############################# WATCHDOG to monotor the connection to the tail section ######################################
        if time.time()-timeLastWrite>timeLastCommunicationMax:
            pubStatus.publish(nodeID = 2, status = False)
            str = "cannot send the demand: shutdown the tail_section"
            print str
            rospy.logerr(str)
            pubMissionLog.publish(str)
            rospy.signal_shutdown(str)
        if time.time()-timeLastRead>timeLastCommunicationMax:
            pubStatus.publish(nodeID = 2, status = False)
            str = "cannot get the feedback: shutdown the tail_section"
            print str
            rospy.logerr(str)
            pubMissionLog.publish(str)
            rospy.signal_shutdown(str)
            
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
    
def angleToSetpoint(angle, zero):

    maxlimit = 170
    minlimit = 5

    setpoint = zero - (1.832 * angle)
    setpoint = limits(setpoint,minlimit,maxlimit)
        
    return int(setpoint)
    
################################################################################
    
def setpointToAngle(setpoint, zero):

    angle = (zero - setpoint)/1.832        

    return int(angle)

################################################################################

def speedToSetpoint(prop_demand, prop_zero):

    maxlimit = 150
    minlimit = 40

    if abs(prop_demand)<10: # apply deadband to the propeller demand
        prop_demand = 0
    
    setpoint = prop_zero - prop_demand
    setpoint = limits(setpoint,minlimit,maxlimit)
   
    return int(setpoint)
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
    message = '$S%d@%d@%d@%d@%d@#' %(b_zero, c_zero, d_zero, e_zero, prop_zero)
    time.sleep(0.2) # give enough time for arduino to receive the data packet
    serialPort.flushInput()
    serialPort.flushOutput()
    serialPort.close()
    serialPort.write(message)

################################################################################
################################################################################

if __name__ == '__main__':
    time.sleep(1) #Allow System to come Online
    global serialPort
    
    pub = rospy.Publisher('tail_output', tail_feedback)
    pubStatus = rospy.Publisher('status', status)
    pubMissionLog = rospy.Publisher('MissionStrings', String)
    
    rospy.init_node('tail_section')
    rospy.on_shutdown(shutdown)
    cs_cfg = CS_config()
    
    port_status = setupSerial()
    str = "Tailsection serial port status = %s. Port = %s" %(port_status, serialPort.portstr)
    
    rospy.Subscriber('tail_setpoints_horizontal', tail_setpoints, horizontal_callback)
    rospy.Subscriber('tail_setpoints_vertical', tail_setpoints, vertical_callback)
    rospy.Subscriber('prop_demand', Int8, prop_callback)

    
    if port_status == True:    
        status = True
        pubStatus.publish(nodeID = 2, status = status)
        #rospy.loginfo("Tail section now online")
    else:
        status = False
        pubStatus.publish(nodeID = 2, status = status)
    
    time.sleep(5) # delay to wait for other systems to come alive, might not be necessary?
    
    tail_section_loop(status)
