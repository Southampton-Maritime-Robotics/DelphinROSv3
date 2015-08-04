#!/usr/bin/python

"""
A driver to turn LED torch on/off.

May not functioning!

"""

import rospy
import serial
import time
import numpy
from std_msgs.msg import Bool


################################################################
def setUpSerial(): # set up the serial port
    global serialPort
    serialPort = serial.Serial(port='/dev/usblight', baudrate='38400', timeout=0.5) #SHOULD NORMALLY BE ttyS3!!
    serialPort.bytesize = serial.EIGHTBITS
    serialPort.stopbits = serial.STOPBITS_ONE
    serialPort.parity = serial.PARITY_NONE
####    print "Initialised arduino serial."

    return serialPort.isOpen()


################################################################
def light_callback(data):
    
    if data.data == True:
        serialPort.write('L1')
####        print 'Light switched on'
    else:
        serialPort.write('L0')
####        print 'Light switched off'


################################################################
def phone_callback(data):
    
    if data.data == True:
        serialPort.write('P')
####        print 'Phone switching on'
    else:
        pass
        
################################################################
def shutdown():
    serialPort.flushInput()
    serialPort.flushOutput()
    serialPort.close()

        
################################################################        
#     INITIALISE     ###########################################
################################################################
if __name__ == '__main__':
    time.sleep(0.5) #Allow System to come Online    
    rospy.init_node('light_Arduino')
    
    rospy.Subscriber('light_onOff', Bool, light_callback)

    rospy.on_shutdown(shutdown)         #Defining shutdown behaviour  

    port_status = setUpSerial()
    
    print 'light driver online'
    
    rospy.spin()
        
    
#        global pubStatus
    
