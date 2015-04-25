#!/usr/bin/python

"""
# this code lets ROS node communicate with an arduino in which then talks with maxon motor control board. It is a modified version of the "tsl_customer_mission.py" that was originally used to control thrusters via TSL motor control board

# modification:
  - 12 June 2014:
  - 5 April 2015: make this node work at 10Hz. Rate is control by rospy.Rate()

# usage
  - get voltage: "faVcs"
  - get current: "faIcs"
  - get RPM: "faRcs" (20Hz approximately)
  - set thrusterSetpoint: "faPspeed_1,direction_1@speed_2,direction_2@speed_3,direction_3@speed_4,direction_4@cs"

# the smallest value that setpoint in arduino world could be is 1 in whih correspoinds to a RPM of 138 approximately 

# define
 thruster numbering
    0 vert_front
    1 vert_rear
    2 hor_front
    3 hor_rear
# convention
    [+ve demand] gives [-ve thrust and +ve sway motion] relative to the front-east-down body fix frame

"""

import rospy
import serial
import time
import numpy
import math
import string
from hardware_interfaces.msg import tsl_setpoints
from hardware_interfaces.msg import tsl_feedback
from hardware_interfaces.msg import status

from std_msgs.msg import UInt8
from std_msgs.msg import Bool

global new_data
global current_data
global serialPort
global onOff

global delaySerial
global timeOut
delaySerial = 0.01 # [sec] just enough for arduino to get a data packet
timeOut = 0.1 # [sec] timeout to wait for a data packet being sent back from arduino

global ThrusterSetpoint_max
ThrusterSetpoint_max = 255 # in a range of [0-255]
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
    print serialPort.isOpen()
    return serialPort.isOpen()

############################# INITIALISE BOARD ######################################    
def init_board():
    global serialPort

    # check board status
    serialPort.flushInput() # flush to be safe
    motor_shutdown() # Shut down motors just in case.

    time.sleep(1) # put a delay to give enought time for the board to response for voltage reading
    voltage = getVoltage() #Gets voltage data from board to check that board is sending and recieving messages
    return True
    if voltage == 0:
        return False
    else:
        return True

############################# GET VOLTAGE ######################################    

def getVoltage():

    global timeOut # timeout to wait for a data packet being sent back from arduino
    global delaySerial
    
    # request a voltage measurement
    try:
        str = 'faVcs'
        serialPort.write(str)
        time.sleep(delaySerial) # should be enough for arduino to get a ReqVoltage
    except:
        pass
        
    #empty a temporary string and set voltage to zero by default
    msg = ""
    vol = 0
    
    try:
        if serialPort.inWaiting(): # if arduino replied        
            
            timeStart = time.time()
            timeElapse = time.time()-timeStart   
            found = 0 
            while(timeElapse<timeOut) and not rospy.is_shutdown() and not found:
                if serialPort.inWaiting()>0: 
                    _in = serialPort.read(1)
                    if _in == '#': # searching for a header '#', and continue if found. otherwise, neglect this data packet
                        timeStart = time.time()
                        timeElapse = time.time()-timeStart    
                        while(timeElapse<timeOut) and not rospy.is_shutdown() and not found: # get the rest of a data packet
                            if serialPort.inWaiting()>0:
                                _in = serialPort.read(1)
                                if _in=='V': # break if found the end of a data packet
                                    found = 1
                                    break
                                else: # keep the data in a msg string
                                    msg+=_in
                                timeElapse = time.time()-timeStart # elapsed time for getting the rest of data packet
                        try :
                            vol = int(float(msg)*1000)
                        except :
                            pass
                        
                timeElapse = time.time()-timeStart # elapsed time for finding a header
    except:
        pass
    
    return vol # return in mili vol
    
############################# GET AMPARE ######################################    

def getAmp():

    global timeOut
    global delaySerial
    
    # request a current measurement
    try:
        str = 'faIcs'
        serialPort.write(str)
        time.sleep(delaySerial) # should be enough for arduino to get a ReqVoltage
    except:
        pass
        
    #empty a temporary string and set voltage to zero by default
    msg = ""
    amp = 0
    
    try:
        if serialPort.inWaiting(): # if arduino replied
            
            timeStart = time.time()
            timeElapse = time.time()-timeStart    
            found = 0
            while(timeElapse<timeOut) and not rospy.is_shutdown() and not found:
                if serialPort.inWaiting()>0: 
                    _in = serialPort.read(1)
                    if _in == '#': # searching for a header '#', and continue if found. otherwise, neglect this data packet
                        timeStart = time.time()
                        timeElapse = time.time()-timeStart    
                        while(timeElapse<timeOut) and not rospy.is_shutdown() and not found: # get the rest of a data packet
                            if serialPort.inWaiting()>0:
                                _in = serialPort.read(1)
                                if _in=='A': # break if found the end of a data packet
                                    found = 1
                                else: # keep the data in a msg string
                                    msg+=_in
                                timeElapse = time.time()-timeStart # elapsed time for getting the rest of data packet
                        try :
                            amp = int(float(msg)*1000)
                        except :
                            pass
                        
                timeElapse = time.time()-timeStart # elapsed time for finding a header
    except:
        pass
            
    return amp
    
############################# GET RPM ######################################    

def getRPM():

    global timeOut
    global delaySerial

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
    except:
        pass
    
    return rpm

############################# MAIN LOOP ############################################    
def motor_control(status):

    global onOff_horiz
    global onOff_vert
    global current_data
    global delaySerial
    global ThrusterSetpoint_max
    global timeHorizLastDemand
    global timeVertLastDemand
    onOff_horiz = 1
    onOff_vert = 1
    sp_max = 2300
    sp_max_arduino = 255
    time_zero = time.time()
    timeHorizLastDemand = time.time()
    timeVertLastDemand = time.time()

    #parameters to utilize watchdog
    timeStart_vol = time.time()
    timeLim_vol = 5 # [sec]
    voltage_min = 19000 # [mV]
    timeElapse_rpm = [0.0,0.0,0.0,0.0]
    timeStart_rpm = [timeStart_vol,timeStart_vol,timeStart_vol,timeStart_vol]
    timeLim_rpm = timeLim_vol
    
    controlRate = 10. # [Hz]
    controlPeriod = 1./controlRate
    r = rospy.Rate(controlRate)
    
    while not rospy.is_shutdown():      
        
        timeRef = time.time()
        
        pubStatus.publish(nodeID = 1, status = status)
        data_now = current_data #Get the data at the current time. This is in the range of [-2500,2500] 
        
        ############################# ON/OFF MSG ######################################    
        thrust_sp = [data_now['thruster0']*onOff_vert, data_now['thruster1']*onOff_vert, data_now['thruster2']*onOff_horiz, data_now['thruster3']*onOff_horiz]
        
        ############################# REMAP THRUSTER SETPOINT ######################################    
        thrust_dr = thrust_dr_ref[:] # create a deep copy of a vector to correct thrust_direction
        # remap a setpoint from [-2500, 2500] to [0,255] with direction of 0 or 1

        for i in range(len(thrust_sp)):
            if thrust_sp[i] > 0:
                thrust_dr[i] = 1-thrust_dr[i]
            thrust_sp[i] = float(abs(thrust_sp[i]))/sp_max*sp_max_arduino
            if thrust_sp[i] > 0:
                if thrust_sp[i] <1: # if a setpoint is not zero but less than 1, force it to one
                    thrust_sp[i] = 1
                else:
                    thrust_sp[i] = int(thrust_sp[i])
        # the smallest value that setpoint in arduino world could be is 1 in whih correspoinds to a RPM of 138 approximately
                
        ############################# SETPOINT MSG ####################################    

        for i in range(len(thrust_sp)):
            if thrust_sp[i] > ThrusterSetpoint_max:
                thrust_sp[i] = ThrusterSetpoint_max

        # Note: onOff flag will be pulled off by the stop function when terminate the mission
        if time.time()-timeVertLastDemand < timeLastDemand_sat:
            setpoint0= onOff_vert*thrust_sp[0]
            setpoint1= onOff_vert*thrust_sp[1]
        else: # if there is no update on vert thruster for longer than timeLastDemand_sat, turn off the vert thruster
            setpoint0= 0
            setpoint1= 0
        if time.time()-timeHorizLastDemand < timeLastDemand_sat:
            setpoint2= onOff_horiz*thrust_sp[2]
            setpoint3= onOff_horiz*thrust_sp[3]
        else: # if there is no update on horiz thruster for longer than timeLastDemand_sat, turn off the horiz thruster
            setpoint2= 0
            setpoint3= 0
            
        setPoints = '%d,%d@%d,%d@%d,%d@%d,%d@' %(setpoint0,thrust_dr[0],setpoint1,thrust_dr[1],setpoint2,thrust_dr[2],setpoint3,thrust_dr[3])
        dataToSend = 'faP' + setPoints # add header to a data packet
        dataToSend = dataToSend + 'cs' # add footer to a data packet        
        
        ############################# SET SPEED ######################################    
        serialPort.write(dataToSend)
        time.sleep(delaySerial)

        ############################# GET INFO. ######################################    
        voltage = getVoltage()
        amp = getAmp()    
        rpm = getRPM()

        ############################# WATCHDOG ######################################    
        if voltage < voltage_min:
            timeElapse_vol = time.time() - timeStart_vol
            if timeElapse_vol > timeLim_vol:
                status = False
                str = "voltage is too low: shutdown the ArduinoMaxon"
                rospy.logerr(str)
                shutdown()
        else:
            timeStart_vol = time.time()
            for i in range(len(rpm)):
                if rpm[i] == -1:
                    timeElapse_rpm[i] = time.time()-timeStart_rpm[i]
                    if timeElapse_rpm[i] > timeLim_rpm:
                        status = False
                        str = "motor is not functioning: shutdown the ArduinoMaxon"
                        rospy.logerr(str)
                        shutdown()
                else:
                    timeStart_rpm[i] = time.time()

        ############################# PUBLISH INFO. ######################################
            
        current0 = amp
        current1 = 0
        current2 = 0
        current3 = 0 

        speed0 = rpm[0]
        speed1 = rpm[1]
        speed2 = rpm[2]
        speed3 = rpm[3]
        
        pub.publish(setpoint0 = setpoint0*(2*thrust_dr[0]-1), setpoint1 = setpoint1*(2*thrust_dr[1]-1), setpoint2 = setpoint2*(2*thrust_dr[2]-1), setpoint3 = setpoint3*(2*thrust_dr[3]-1), 
            speed0 = speed0, speed1 = speed1, speed2 = speed2, speed3 = speed3, 
            current0 = current0, current1 = current1, current2 = current2, current3 = current3, 
            voltage = voltage)
        
        timeElapse = time.time()-timeRef
        if timeElapse < controlPeriod:
            r.sleep()
        else:
            str = "Arduino_thruster rate does not meet the desired value of %.2fHz: actual control rate is %.2fHz" %(controlRate,1/timeElapse) 
            rospy.logwarn(str)
    
############################# CALLBACKS ######################################    

def vert_callback(new_sp):
    global current_data
    global timeVertLastDemand
    current_data['thruster0']= new_sp.thruster0
    current_data['thruster1'] = new_sp.thruster1
    timeVertLastDemand = time.time()

def horiz_callback(new_sp):
    global current_data
    global timeHorizLastDemand
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
    serialPort.flushOutput()
    serialPort.write(dataToSend)
    time.sleep(delaySerial)

def shutdown():
    motor_shutdown()
    serialPort.flushInput()
    serialPort.flushOutput()
    pubStatus.publish(nodeID = 1, status = False)
    rospy.loginfo("ArduinoMaxon goes offline")
    serialPort.close()

def onOff_horiz_callback(new_onOff_horiz):
    global onOff_horiz
    onOff_horiz = new_onOff_horiz.data
    
def onOff_vert_callback(new_onOff_vert):
    global onOff_vert
    onOff_vert = new_onOff_vert.data

############################# SETUP ######################################    

if __name__ == '__main__':
    time.sleep(1) #Allow System to come Online
    rospy.init_node('Arduino_Thruster')    
    
    global current_data
    global pub
    global pubStatus
    
    port_status = init_serial()
    
    rospy.Subscriber('TSL_onOff_horizontal', Bool, onOff_horiz_callback)
    rospy.Subscriber('TSL_onOff_vertical', Bool, onOff_vert_callback)
    rospy.Subscriber('TSL_setpoints_horizontal', tsl_setpoints, horiz_callback)
    rospy.Subscriber('TSL_setpoints_vertical', tsl_setpoints, vert_callback)
    pub = rospy.Publisher('TSL_feedback', tsl_feedback)
    pubStatus = rospy.Publisher('status', status)
    
    board_status = init_board()
    
    if board_status and port_status == True:
        status = True
        pubStatus.publish(nodeID = 1, status = status)
        rospy.loginfo("ArduinoMaxon board now online")
    else:
        status = False
        pubStatus.publish(nodeID = 1, status = status)
    
    rospy.on_shutdown(shutdown) #Defining shutdown behaviour
    motor_control(status) # entering the main loop


