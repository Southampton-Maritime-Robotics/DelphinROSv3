#!/usr/bin/python

"""
A driver for a costom thruster control board, consisting of an arduino and four maxon board. 

Developed based on "tsl_customer_mission.py" that was originally used to control thrusters via TSL motor control board

# thruster numbering
    0 vert_front
    1 vert_rear
    2 hor_front
    3 hor_rear

# usage
  - get RPM: "faRcs" (the fastest rate is 20Hz approximately)
  - set thrusterSetpoint: "faPspeed_1,direction_1@speed_2,direction_2@speed_3,direction_3@speed_4,direction_4@cs"
    
# convention
+ve demand contribute to +ve force w.r.t. the front-east-down body-fixed frame convertion

######################################
#Modifications
12/6/14: created the thruster interface
5/4/15: maked this node work at 5Hz. Rate is control by rospy.Rate()
10/5/15: compensated the voltage reading with a scale factor of 1.08
4/6/15: remap a setpoint from [-2500,-145] U [145,2500] to [0,255] with direction of either 0 or 1.
5/10/15: removed voltage and amp measurement to energy_monitoring node

# NOTE: setpoint within (-145,145) will be set to 0, i.e. incorporate as a deadband

TODO:
- shrink down the preamble and checksum: fa->$, cs->#
- do not need to subscribe to on-off flag

"""

import rospy
import serial
import time
import numpy
import math
from hardware_interfaces.msg import tsl_setpoints
from hardware_interfaces.msg import tsl_feedback
from hardware_interfaces.msg import status

from std_msgs.msg import UInt8
from std_msgs.msg import Bool
from std_msgs.msg import String

global new_data
global current_data
global serialPort

global delaySerial
global timeOut
delaySerial = 0.01 # [sec] just enough for arduino to get a data packet
timeOut = 0.1 # [sec] timeout to wait for a data packet being sent back from arduino

global ThrusterSetpoint_max
ThrusterSetpoint_max = 2500 # a limit in motor speed in a range of [0-2500] rpm %
global thrust_dr_ref
thrust_dr_ref = [0,0,0,1] # a vector to correct thrust_direction
global timeLastDemand_sat
timeLastDemand_sat = 1 # [sec] associated thruster will be stopped if there is no new demand for longer than this many second

############################# INITIALISE SERIAL ######################################
def init_serial():
    global serialPort
    global current_data
    current_data = {'thruster0':0,'thruster1':0,'thruster2':0,'thruster3':0}
    serialPort = serial.Serial(port='/dev/usbArduino', baudrate='38400', timeout=0.1)
    serialPort.bytesize = serial.EIGHTBITS
    serialPort.stopbits = serial.STOPBITS_ONE
    serialPort.parity = serial.PARITY_NONE
    serialPort.flushInput()
    serialPort.flushOutput()
    time.sleep(0.01)
    
    return serialPort.isOpen()

############################# GET RPM ######################################    

def getRPM():

    # request a RPM measurement
    try:
        str = 'faRcs'
        serialPort.write(str)
        time.sleep(delaySerial) # should be enough for arduino to get a ReqVoltage
    except:
        pass
        
    #initialize an empty string and 4x1 matrix filled with zero
    msg = ""
    rpm = [0,0,0,0]
    
    try:
        if serialPort.inWaiting(): # if arduino replied
            timeStart = time.time()
            timeElapse = time.time()-timeStart
            found = 0
            while(timeElapse<timeOut) and not rospy.is_shutdown() and not found:
                if serialPort.inWaiting()>0: 
                    _in = serialPort.read(1)
                    if _in == '#': # searching for a header '#', and continue if found. otherwise, neglect this character
                        i_rpm = 0
                        timeStart = time.time()
                        timeElapse = time.time()-timeStart    
                        while(timeElapse<timeOut) and not rospy.is_shutdown() and not found: # get the rest of a data packet
                            if serialPort.inWaiting()>0:
                                _in = serialPort.read(1)
                                if _in=='R': # break if found the end of a data packet
                                    found = 1
                                elif _in=='@':
                                    rpm[i_rpm] = int(float(msg))
                                    i_rpm += 1
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
    
    return rpm

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
    
    while not rospy.is_shutdown():      
        
        timeRef = time.time()
        
        pubStatus.publish(nodeID = 1, status = True)
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
        
        setPoints = '%d,%d@%d,%d@%d,%d@%d,%d@' %(setpoint0,thrust_dr[0],setpoint1,thrust_dr[1],setpoint2,thrust_dr[2],setpoint3,thrust_dr[3])
        dataToSend = 'faP' + setPoints # add header to a data packet
        dataToSend = dataToSend + 'cs' # add footer to a data packet
        
        ############################# SET SPEED ######################################    
        serialPort.write(dataToSend)
        time.sleep(delaySerial)

        ############################# GET INFO. ######################################    
        rpm = getRPM()

        ############################# WATCHDOG ######################################
        for i in range(len(rpm)):
            if rpm[i] == -1:
                timeElapse_rpm[i] = time.time()-timeStart_rpm[i]
                if timeElapse_rpm[i] > timeLim_rpm:                
                    pubStatus.publish(nodeID = 1, status = False)
                    str = "motor is not functioning: shutdown the ArduinoMaxon"
                    rospy.logerr(str)
                    pubMissionLog.publish(str)
                    rospy.signal_shutdown(str)
            else:
                timeStart_rpm[i] = time.time()

        ############################# PUBLISH INFO. ######################################
        
        speed0 = rpm[0]
        speed1 = rpm[1]
        speed2 = rpm[2]
        speed3 = rpm[3]
        
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

def motor_shutdown():
    global delaySerial
    # construct a string to shutdown the motor
    setPoints = ''
    setPoints = '0,0@0,0@0,0@0,0@'
    dataToSend = 'faP' + setPoints # add header to a data packet
    dataToSend = dataToSend + 'cs' # add footer to a data packet
    # send a shutdown command
    serialPort.write(dataToSend)
    time.sleep(delaySerial)

def shutdown():
    motor_shutdown()
    serialPort.flushInput()
    serialPort.flushOutput()
    pubStatus.publish(nodeID = 1, status = False)
    serialPort.close()

############################# SETUP ######################################    

if __name__ == '__main__':
    time.sleep(1) #Allow System to come Online
    rospy.init_node('Arduino_Thruster')    
    
    global current_data
    
    port_status = init_serial()
    
    rospy.Subscriber('TSL_setpoints_horizontal', tsl_setpoints, horiz_callback)
    rospy.Subscriber('TSL_setpoints_vertical', tsl_setpoints, vert_callback)
    
    pub = rospy.Publisher('TSL_feedback', tsl_feedback)
    pubStatus = rospy.Publisher('status', status)
    pubMissionLog = rospy.Publisher('MissionStrings', String)
    
    if port_status == True:
        pubStatus.publish(nodeID = 1, status = True)
        rospy.loginfo("ArduinoMaxon board now online")
    else:
        pubStatus.publish(nodeID = 1, status = False)
    
    rospy.on_shutdown(shutdown) #Defining shutdown behaviour
    motor_shutdown() # Shut down motors just in case.

    motor_control() # entering the main loop
