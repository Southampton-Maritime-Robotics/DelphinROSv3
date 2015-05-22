#!/usr/bin/python

"""
######################################
Driver for a pressure transducer

This node was for publishing the depth from pressure transducer and orientation from compass and gyro.
Since we have xsens installed to the Delphin2, this node only responsible for providing depth information.

keys parameters that sill in use
-depth: the raw measurement took from the pressure transducer
-depth_calib: the depth that calibrated with the pitch angle of the AUV
-depth_filt: the depth reading that is filtered by the polynomial-type (PT) filter
-depth_der: the derivative of depth that is determined from the gradient of PT filter

######################################
#Modifications
9 Apr 2015: alter the way to control sampling rate with rospy.rate()
24 Apr 2015: calibrate the depth reading using the pitch angle and shift between center of rotation and location of depth transducer 
22 May 2015: have this node publishes directly to the depth_out rather than send to the xsens and then publish

"""

import rospy
import numpy
import serial
import time
import math
from re import findall
from hardware_interfaces.msg import compass
from hardware_interfaces.msg import depth
from hardware_interfaces.msg import status

global serialPort

################################################################
def setUpSerial():
    global serialPort
    serialPort = serial.Serial(port='/dev/usboceanserver', baudrate='115200', timeout=0.01) # may need to change timeout if having issues!
    serialPort.bytesize = serial.EIGHTBITS
    serialPort.stopbits = serial.STOPBITS_ONE
    serialPort.parity = serial.PARITY_NONE
    print "Initialised OceanServer serial."
    print serialPort.portstr
    print serialPort.isOpen()
    serialPort.flushInput()
    serialPort.flushOutput()
    return serialPort.isOpen()
    
################################################################
def listenForData(status):
    global serialPort
    global pitch_callback
    
    pitch_callback = 0.
    
    controlRate = 10. # [Hz]
    controlPeriod = 1/controlRate
    r = rospy.Rate(controlRate)

    #### FILTER INFO ####
    depth_array_length = 20
    Dx = numpy.zeros([depth_array_length],float) # time vector for PT filtering
    Dy = numpy.zeros([depth_array_length],float) # depth vector for PT filtering
    for i in range(0,depth_array_length): 
        Dx[i] = i
    
    depth_old = 0.0
    rateOK = True
    
    L_sensor    = 1.2   # a location of the depth sensor w.r.t. AUV nose. [m] measured
    L_ref        = 0.65  # a reference point w.r.t. AUV nose [m] approximate
    L_shift     = L_sensor-L_ref # [m]
    
    #####################
    
    while not rospy.is_shutdown():    
        timeRef = time.time()
        try:
#            time.sleep(0.01)  # Prevents node from using 100% CPU!!
            while serialPort.inWaiting() > 0 and serialPort.read(1) == '$':     #while there is data to be read - read the line...
                
                pubStatus.publish(nodeID = 5, status = status)
                
                dataRaw = serialPort.readline()
                                
                data = numpy.array((findall('[-]*\d+.\d+',dataRaw)), numpy.float)
                try:
                     
                    depth       = (data[4]*42.045)- 0.84+0.53  -0.21  -0.05   #Convert ADC value to depth value
                    
                    depth_calib = depth - L_shift*math.sin(pitch_callback*math.pi/180.)# depth that takes into account the pitch angle of the AUV

                    #### DEPTH FILTER ####
                    if rateOK:
                        T = controlPeriod
                    else:
                        T = timeElapse
                    Dx_real = Dx * T
                    
                    Dy[1:depth_array_length] = Dy[0:(depth_array_length-1)]
                    Dy[0] = depth_calib
                    [der, depth_filt] = numpy.polyfit(Dx_real, Dy, 1)
                    depth_der = -der
                    
                    depth_msg.depth = depth # the depth that is directly determined from a static pressure.
                    depth_msg.depth_calib = depth_calib # the depth with pitch angle correction
                    depth_msg.depth_filt = depth_filt # depth_calib that is filtered by PT-filter.
                    depth_msg.depth_der = depth_der # derivative of depth_calib: a by-product of the PT-filter.
                    
                    #Publish data to compass_out
                    pub.publish(depth_msg)
                    
                except ValueError: 
                    print 'not a float'
                
        except:
            print 'read error'
        
        timeElapse = time.time()-timeRef
        if timeElapse < controlPeriod:
            rateOK = True
            r.sleep()
        else:
            rateOK = False
            str = "compass_oceanserver rate does not meet the desired value of %.2fHz: actual control rate is %.2fHz" %(controlRate,1/timeElapse) 
            rospy.logwarn(str)

def compass_callback(data):
    global pitch_callback
    pitch_callback = data.pitch # pitch angle measured by xsens

################################################################

def validDataCheck():

    attempts = 1
    
    while attempts < 5:
        
        while not serialPort.read(1) == '$':
            pass
        #The following line has been left for clarity of the change required with new Python versions
        #dataRaw = serialPort.readline(size = None, eol = '\n')   #This line MUST be left commented!
        #The following line contains the actual change required with Python v2.6+; the command reads in line of the data
        dataRaw = serialPort.readline()  #serialIO.readline()
        data = findall('[-]*\d+.\d+',dataRaw)
        
        #Uncomment the following line only during debugging
        #print data
        if len(data) == 13:
            return True
    
    return False
    
################################################################
def shutdown():
    serialPort.flushInput()
    serialPort.flushOutput()
    pubStatus.publish(nodeID = 5, status = False)
    serialPort.close()

################################################################        
#     INITIALISE     ###########################################
################################################################

if __name__ == '__main__':
    time.sleep(1) #Allow System to come Online    
    rospy.init_node('OceanServer_compass')
    
    global pub
    global serialPort
    global depth_msg
    depth_msg = depth()
    
    pub = rospy.Publisher('depth_out', depth)
    rospy.Subscriber('compass_out', compass, compass_callback) 
    pubStatus = rospy.Publisher('status', status)
    
    rospy.on_shutdown(shutdown)
    
    port_status = setUpSerial()
#    str = "OceanServer port status = %s. Port = %s" %(port_status, serialPort.portstr)
#    rospy.loginfo(str)
    time.sleep(0.3)
    string_status=validDataCheck()
    
    if (port_status and string_status) == True:    
        status = True
        pubStatus.publish(nodeID = 5, status = status)
        rospy.loginfo("OceanServer online")
    else:
        status = False
        pubStatus.publish(nodeID = 5, status = status)
    
    listenForData(status)   #Main loop for receiving data
