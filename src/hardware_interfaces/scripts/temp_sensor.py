#!/usr/bin/python
import roslib; roslib.load_manifest('hardware_interfaces')
import rospy
import serial
import time
import numpy
import math
import string
from std_msgs.msg import Float32
from pylab import *


#######################################################
#Modifications to code
# 17/5/2012 new node to read temperature sensor



################### GLOBAL VARIABLES ################### 

global serialPort
global temp

################### SERIAL SETUP ################### 

def setUpSerial():
    global serialPort
    serialPort = serial.Serial(port='/dev/ttyS1', baudrate='9600', timeout=0.1) # may need to change timeout if having issues!
    serialPort.bytesize = serial.EIGHTBITS
    serialPort.stopbits = serial.STOPBITS_ONE
    serialPort.parity = serial.PARITY_NONE
    print "Initialised Temperature Sensor (ttyS1)"
    print serialPort.portstr
    print serialPort.isOpen()

    time.sleep(1)

################## Set update interval

    try:
	serialPort.write('Set Interval (1) \r\n')

    except:
	print 'write error'
        
################# Check updating at required 
    check=False
    counter=time.time()	
    while check==False and not rospy.is_shutdown():	

        

	try:
            while serialPort.inWaiting() > 0: #and serialPort.read(1) == '$': #while there is data to be read - read the line...
                        
                data = serialPort.readline(size = None, eol = '\n')		#Read in line of data
                split_data = string.split(data,'\t')
		temp=split_data[4]
		temp=float(temp)		
		print temp
                pub.publish(temp)
        	
		time_delay=time.time()-counter
		
		counter=time.time()

		time.sleep(0.5)

		if time_delay <5:
			check=True
			print 'Temperature Sensor Update rate changed to 1 Hz'			
			return serialPort.isOpen()
	        else:
			serialPort.write('Set Interval (1) \r\n')
	except:
		dummy=1



################### MAIN LOOP ################### 

  

def listenForData():

    global serialPort
    global pub
            
    while not rospy.is_shutdown():
        try:
            while serialPort.inWaiting() > 0: #and serialPort.read(1) == '$': #while there is data to be read - read the line...
                        
                data = serialPort.readline(size = None, eol = '\n')		#Read in line of data
                split_data = string.split(data,'\t')
		temp=split_data[4]
		temp=float(temp)		
		print temp
                pub.publish(temp)
                
                
        except:
            #print 'read error'
		time.sleep(0.01)
	time.sleep(0.5)

	

################### SHUTDOWN FUNCTION ################### 
def shutdown():
    global serialPort    
    serialPort.flushInput()
    serialPort.flushOutput()
    serialPort.close()

################### INITIALISING FUNCTION ################### 

if __name__ == '__main__':
    time.sleep(4) #Allow System to come Online    
    global pub    
    rospy.init_node('temp_sensor')
    rospy.on_shutdown(shutdown)         #Defining shutdown behaviour  

  
    #Define Publishers
    pub = rospy.Publisher('water_temp', Float32, queue_size=3) 
    time.sleep(1)
    #Setup serial port and check its status
    port_status = setUpSerial()    
    str = "Temperature Sensor port status = %s. Port = %s" %(port_status, serialPort.portstr)
    rospy.loginfo(str)
    
    listenForData()                     #Main loop for receiving data

    
