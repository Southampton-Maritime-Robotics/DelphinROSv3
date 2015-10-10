#!/usr/bin/python

"""
A node to monitor
- voltage from battery
- current consumed by thrusters (one for each)
- current cunsumed by fins (all together)
- current consumed by propeller

# thruster numbering
    0 vert_front
    1 vert_rear
    2 hor_front
    3 hor_rear

# usage
  - get voltage: "faVcs"
  - get current: "faIcs" # [thruster0, thruster1, thruster2, thruster3, fins, prop]
    
######################################
#Modifications

"""

import rospy
import serial
import time
import numpy
from hardware_interfaces.msg import status
from hardware_interfaces.msg import energy_consumed

from std_msgs.msg import String

global serialPort
global delaySerial
global timeOut
delaySerial = 0.01 # [sec] just enough for arduino to get a data packet
timeOut = 0.1 # [sec] timeout to wait for a data packet being sent back from arduino

############################# INITIALISE SERIAL ######################################
def init_serial():
    global serialPort
    serialPort = serial.Serial(port='/dev/usbEnergyMonitor', baudrate='9600', timeout=0.1)
    serialPort.bytesize = serial.EIGHTBITS
    serialPort.stopbits = serial.STOPBITS_ONE
    serialPort.parity = serial.PARITY_NONE
    serialPort.flush()
    return serialPort.isOpen()

############################# GET VOLTAGE ######################################    

def getVoltage():
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
                
            if timeElapse>timeOut:
                vol = -1
    except:
        pass
    
    return vol # return in mili vol
    
############################# GET AMPARE ######################################    

def getAmp():
    # request a current measurement
    try:
        str = 'faIcs'
        serialPort.write(str)
        time.sleep(delaySerial) # should be enough for arduino to get a ReqVoltage
    except:
        pass
        
    #empty a temporary string and set voltage to zero by default
    msg = ""
    amp = [0,0,0,0,0,0]
    
    try:
        if serialPort.inWaiting(): # if arduino replied
            
            timeStart = time.time()
            timeElapse = time.time()-timeStart    
            found = 0
            while(timeElapse<timeOut) and not rospy.is_shutdown() and not found:
                if serialPort.inWaiting()>0: 
                    _in = serialPort.read(1)
                    if _in == '#': # searching for a header '#', and continue if found. otherwise, neglect this data packet
                        i_amp = 0
                        timeStart = time.time()
                        timeElapse = time.time()-timeStart
                        while(timeElapse<timeOut) and not rospy.is_shutdown() and not found: # get the rest of a data packet
                            if serialPort.inWaiting()>0:
                                _in = serialPort.read(1)
                                if _in=='A': # break if found the end of a data packet
                                    found = 1
                                elif _in=='@':
                                    amp[i_amp] = int(float(msg))
                                    i_amp += 1
                                    msg = ""
                                else: # keep the data in a msg string
                                    msg+=_in
                                timeElapse = time.time()-timeStart # elapsed time for getting the rest of data packet
                        
                timeElapse = time.time()-timeStart # elapsed time for finding a header
            
            if timeElapse>timeOut:
                amp = [-1,-1,-1,-1,-1,-1]
                
    except:
        amp = [-1,-1,-1,-1,-1,-1]
            
    return amp

############################# MAIN LOOP ############################################    
def main_loop():

    #parameters to utilize watchdog
    try: 
        voltage_min = rospy.get_param('min-motor-voltage')
    except:
        voltage_min = 19000 # [mV]
        
    timeStart_vol = time.time()
    timeLim_vol = 5 # [sec]
    timeLim_com = timeLim_vol
    
    controlRate = 5. # [Hz] limited by the rate of tail section
    controlPeriod = 1./controlRate
    r = rospy.Rate(controlRate)
    
    while not rospy.is_shutdown():
        
        timeRef = time.time()
        
        pubStatus.publish(nodeID = 12, status = True)        

        ############################# GET INFO. ######################################    
        voltage = getVoltage()
        amp = getAmp() # this is a list

        ############################# WATCHDOG ######################################    
        if voltage < voltage_min:
            timeElapse_vol = time.time() - timeStart_vol
            if timeElapse_vol > timeLim_vol:
                pubStatus.publish(nodeID = 12, status = False)
                if voltage == -1:
                    str = "energy_monitor has lost a communication to arduino"
                else:
                    str = "voltage is too low: shutdown the energy_monitor"
                rospy.logerr(str)
                pubMissionLog.publish(str)
                rospy.signal_shutdown(str)
        else:
            timeStart_vol = time.time()
            
        ############################# PUBLISH INFO. ######################################
        
        
        pubOutput.publish(batteryVol = voltage, 
                          thruster_0 = amp[0], 
                          thruster_1 = amp[1], 
                          thruster_2 = amp[2],
                          thruster_3 = amp[3], 
                          fins = amp[4], 
                          prop = amp[5])

        timeElapse = time.time() - timeRef
        if timeElapse < controlPeriod:
            r.sleep()
        else:
            str = "energy_monitor rate does not meet the desired value of %.2fHz: actual control rate is %.2fHz" %(controlRate,1/timeElapse) 
            rospy.logwarn(str)
            pubMissionLog.publish(str)
    
############################# CALLBACKS ######################################

def shutdown():
    serialPort.close()
    pubStatus.publish(nodeID = 12, status = False)
    rospy.loginfo("energy_monitor goes offline")

############################# SETUP ######################################

if __name__ == '__main__':
    time.sleep(3) #Allow System to come Online
    rospy.init_node('Energy_Monitor')
    
    _ready = 0
    for _nTry in range(10): # try to connect to arduino this many time
        port_status = init_serial()
        voltage = getVoltage()
        if port_status == True and voltage != -1:
            ready = 1
            break
        else:
            serialPort.close()
        time.sleep(0.05)

    pubStatus = rospy.Publisher('status', status)
    pubOutput = rospy.Publisher('EnergyConsumed', energy_consumed)
    pubMissionLog = rospy.Publisher('MissionStrings', String)
    
    rospy.on_shutdown(shutdown) # Defining shutdown behaviour
    
    if _ready: # if the connection to arduino is established
        pubStatus.publish(nodeID = 12, status = True)
        rospy.loginfo("energy monitor now online")
    else:
        pubStatus.publish(nodeID = 12, status = False)
    main_loop() # entering the main loop