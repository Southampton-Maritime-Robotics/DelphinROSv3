import roslib; roslib.load_manifest('hardware_interfaces')
import rospy
import serial
import time
import numpy as np
import csv
import string
from hardware_interfaces.msg import gyro


def readData():
    
    vScale    = 5.0/2**24                                                   # 5V / 24 bit resolution
    degScale  = 0.02                                                     # 20mV/^o/s
    zero      = -86034269.2345679
    rate      = 0.0
    heading   = 0.0
    stringtime= 0.0
    time_zero = time.time()
    first     = True
    
    
    while not rospy.is_shutdown():  
        
        t = time.time()
        
        dataRaw = serialPort.readline()
        #print dataRaw
        stringtime_old = stringtime
        stringtime     = time.time()-time_zero
        
        try:
            split_data = string.split(dataRaw,',')		 
            x = np.shape(split_data)

            if x[0] == 2:
                
                rate_old = rate                
                data     = float(split_data[0])
                temp     = float(split_data[1])
                
                data2    = data - zero
                voltage  = data2*vScale
                rate     = -voltage*degScale*10 #Minus because its mounted upside down!
                
                print 'Rate = ',rate
                print 'Temp raw = ',temp
                
                pub.publish(raw = data, rate = rate, temp = temp)
                
               
                            
            else:
                first = False
                                                                    

        except ValueError:
            pass

################################################################
def setUpSerial(): # set up the serial port
    global serialPort
    serialPort = serial.Serial(port='/dev/usbGyro', baudrate='9600', timeout=0.5)
    serialPort.bytesize = serial.EIGHTBITS
    serialPort.stopbits = serial.STOPBITS_ONE
    serialPort.parity = serial.PARITY_NONE

    return serialPort.isOpen()

################################################################
def shutdown():
    serialPort.flushInput()
    serialPort.flushOutput()
    serialPort.close()

