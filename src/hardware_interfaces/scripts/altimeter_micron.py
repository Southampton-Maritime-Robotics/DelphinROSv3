#!/usr/bin/python

"""
A driver for an altertude w.r.t. to the sea bottom.

The sensor is very noisy, therefore, filterred by the Polynomial-Type (PT) filtering technique.

#######################################################
# Modifications to code
4/4/2015: control rate by rospy.Rate()

# TODO
- must include a pitch angle of the AUV into the altitude measurement.

"""

import rospy
import serial
import time
import numpy
from std_msgs.msg import String
from hardware_interfaces.msg import altitude
from hardware_interfaces.msg import status

################################################################
def setUpSerial(): # set up the serial port
    global serialPort
    serialPort = serial.Serial(port='/dev/ttyS0', baudrate='9600', timeout=0.1) #SHOULD NORMALLY BE ttyS0!!
    serialPort.bytesize = serial.EIGHTBITS
    serialPort.stopbits = serial.STOPBITS_ONE
    serialPort.parity = serial.PARITY_NONE
####    print serialPort.portstr
####    print serialPort.isOpen()
    return serialPort.isOpen()

################################################################
def validDataCheck():
    #Check string is of expected format ie of the form *.* m
    #global serialPort
    timeout   = 8
    time_zero = time.time()
    
    attempts = 0
    
    while attempts < 5:
        while serialPort.inWaiting() <= 8 and time.time()-time_zero < timeout:   
            pass
        
        while serialPort.inWaiting() > 8 and time.time()-time_zero < timeout:                           # while there is data to be read - read the line
            attempts = attempts + 1
            data = serialPort.readline()
            r = numpy.size(data)
            try:
                if data[6] == 'm':
                    return True
            except:
                pass

    rospy.loginfo("Altimeter failed to come online")
    return False

################################################################
def filter(altitude):    
    global Dx
    global Dy
    
    Dy[1:array_length] = Dy[0:(array_length-1)]
    if altitude > 0.5:
        Dy[0] = altitude
        
    [der, altitude_filt] = numpy.polyfit(Dx, Dy, 1)
    altitude_der = -der
    
    return [altitude_filt, altitude_der]
    
################################################################
def listenForData(status):
    global serialPort
    global Dx
    global Dy
    
    sample    = numpy.zeros(20)
    timeout   = 2
    time_zero = time.time()
    
    controlRate = 1. # Hz
    controlPeriod = 1/controlRate
    r = rospy.Rate(controlRate)
    
    # to control a timing for status publishing
    timeZero_status = time.time()
    try:
        dt_status = rospy.get_param('status_timing')
    except:
        dt_status = 2.
        
    while not rospy.is_shutdown():
    
        while serialPort.inWaiting() > 8:
            # to control a timing for status publishing
            if time.time()-timeZero_status > dt_status:
                timeZero_status = time.time()
                pubStatus.publish(nodeID = 3, status = True)
            
            try:                       # while there is data to be read - read the line
                data = serialPort.readline()
                if data[6:7] == 'm':
                    print 'data = ',data[0:6]
                    
                    altitude =  float(data[0:6])
                    [altitude_filt, altitude_der] = filter(altitude)

                    print 'altitude = ',altitude
                    pub.publish(altitude = altitude, altitude_filt = altitude_filt, altitude_der = altitude_der)
                    time_zero = time.time()
            except:
                print 'passed'
                pass
            
        if time.time() - time_zero > timeout:
            pubStatus.publish(nodeID = 3, status = False)
            rospy.loginfo("Altimeter has gone offline")
        
        timeElapse = time.time()-time_zero
        if timeElapse<controlPeriod:
            r.sleep()
        else:
            str = "Altimeter_micron rate does not meet the desired value of %.2fHz: actual control rate is %.2fHz" %(controlRate,1/timeElapse) 
            rospy.logwarn(str)
            pubMissionLog.publish(str)
        
################################################################
def shutdown():
    serialPort.flushInput()
    serialPort.flushOutput()
    pubStatus.publish(nodeID = 3, status = False)
    serialPort.close()

################################################################        
#     INITIALISE     ###########################################
################################################################
if __name__ == '__main__':
    
    time.sleep(1) #Allow System to come Online    
    rospy.init_node('MicronEchoSounder')
    rospy.on_shutdown(shutdown)         #Defining shutdown behaviour
    
    pub = rospy.Publisher('altimeter_out', altitude)
    pubMissionLog = rospy.Publisher('MissionStrings', String)
    pubStatus = rospy.Publisher('status', status)
    
    global array_length
    global Dx
    global Dy
    array_length = 10
    Dx = range(0, array_length, 1)
    Dy = numpy.zeros([array_length])
    
    port_status = setUpSerial()

    str = "Altimeter port status = %s. Port = %s" %(port_status, serialPort.portstr)
    rospy.loginfo(str)
    time.sleep(0.3)
    string_status=validDataCheck()
    
    if (port_status and string_status) == True:    
        status = True
        pubStatus.publish(nodeID = 3, status = status)
        rospy.loginfo("Altimeter online")
    else:
        status = False
        pubStatus.publish(nodeID = 3, status = status)
    
    listenForData(status)
