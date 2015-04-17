#!/usr/bin/python

"""
######################################
This node used to play an important role for publishing the compass_out topic
Since we have xsens installed to the Delphin2, this node only responsible for producing a filtered depth and its derivative.
The actual node that publish to compass_out topic is xsens which give a more accurate and reliable orientation of the AUV.

######################################
#Modifications
9 Apr 2015: alter the way to control sampling rate with rospy.rate()
"""

import rospy
import numpy
import serial
import time
import math
from re import findall
from hardware_interfaces.msg import compass
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
    
    controlRate = 20. # [Hz]
    controlPeriod = 1/controlRate
    r = rospy.Rate(controlRate)
    
#    time_zero = time.time()
  
    #### FILTER INFO ####
    depth_array_length = 20
    Dx = numpy.zeros([depth_array_length],float)
    Dy = numpy.zeros([depth_array_length],float)
    for i in range(0,depth_array_length): 
        Dx[i] = i
    
    pitch_array_length = 10
    Px = numpy.zeros([pitch_array_length],float)
    Py = numpy.zeros([pitch_array_length],float)
    for i in range(0,pitch_array_length): 
        Px[i] = i
        
    depth_old = 0.0
    rateOK = True
    
    #####################   
    
    while not rospy.is_shutdown():    
        timeRef = time.time()
        com = compass()
        try:
#            time.sleep(0.01)  # Prevents node from using 100% CPU!!
            while serialPort.inWaiting() > 0 and serialPort.read(1) == '$':     #while there is data to be read - read the line...
                
                pubStatus.publish(nodeID = 5, status = status)
                
                dataRaw = serialPort.readline()
                                
                data = numpy.array((findall('[-]*\d+.\d+',dataRaw)), numpy.float)
                try:

#                    dt = time.time() - time_zero
#                    print dt
#                    time_zero = time.time()
                    
#                    pitch_raw   = data[2]
#                    
#                    pitch       = pitch_raw-180
#                    if pitch <-180:
#                        pitch=pitch%360
#                        
#                    #pitch = pitch_raw
#                    
#                    roll_raw    = data[1]    
#                    roll        =-roll_raw     
                    
                    #roll = roll_raw
                              
#                    temperature = data[3]          
                    depth       = (data[4]*42.045)- 0.84+0.53  -0.21  -0.05   #Convert ADC value to depth value
                    
                    # maybe, this help to remove spike
#                    if (abs(depth - depth_old)) > 1.0:
#                        depth = depth_old
                                        
#                    m           = data[5]
#                    mx          = data[7]
#                    my          =-data[6]  
#                    mz          =-data[8]  
#                    
#                    mx_raw      = data[6]
#                    my_raw      = data[7]
#                    mz_raw      = data[8]
#                    
#                    mx = mx_raw
#                    my = my_raw
#                    mz = mz_raw
#                    
#                                        
#                    heading     = calibrate(mx_raw,my_raw,mz_raw, pitch_raw, roll_raw)
#                    
#                    a           = data[9]
#                    ax          = data[11]
#                    ay          =-data[10]
#                    az          =-data[12]
                                        
                    #### DEPTH FILTER ####
                    if rateOK:
                        T = controlPeriod
                    else:
                        T = timeElapse
                    Dx_real = Dx * T
                    
                    Dy[1:depth_array_length] = Dy[0:(depth_array_length-1)]
                    Dy[0] = depth
                    [der, depth_filt] = numpy.polyfit(Dx_real, Dy, 1)
                    depth_der = -der
                    
                    com.depth = depth
                    com.depth_filt = depth_filt
                    com.depth_der = depth_der
                    
                    #depth_filt = depth
                                        
                    #### PITCH FILTER ####
#                    Px_real = Px * controlPeriod
#                    Py[1:pitch_array_length] = Py[0:(pitch_array_length-1)]
#                    Py[0] = pitch
#                    [der, pitch_filt] = numpy.polyfit(Px_real, Py, 1)
#                    pitch_der = -der                    
                                        
#                    print '*******'
#                    print 'heading %f' %(heading)
##                    print 'pitch %f' %(pitch)                    
#                    print 'pitch (filtered) %f' %(pitch)
##                    print 'pitch_der (filtered) %f' %(pitch_der)
#                    print 'roll %f' %(roll)
#                    print 'temperature %f' %(temperature)
##                    print 'depth %f' %(depth)
#                    print 'depth (filtered) %f' %(depth)
     
                    #Publish data to compass_out
                    pub.publish(com)
                    
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

################################################################

def calibrate(mx,my,mz, pitch, roll):

    pitch = numpy.radians(roll)
    roll  = numpy.radians(pitch)

        
    U = numpy.matrix([[0.00583711864398940, 0.000619779486949836, 9.27558703674774e-06],[0, 0.00478543996712305, -2.33933064978229e-06],[0, 0, 6.05590876286348e-05]])
    
    c = numpy.matrix([[-51.8129873735954],[-27.4232376953900],[13686.8142270094]])

    
    v = numpy.matrix([[mx],[my],[mz]])
    
    w = numpy.dot(U,(v - c))  #Normalised m terms
    
    eqn22_topline = (w[2]*numpy.sin(roll) - w[1]*numpy.cos(roll))
    
    eqn22_botline = (w[0]*numpy.cos(pitch) + w[1]*numpy.sin(pitch)*numpy.sin(roll) + w[2]*numpy.sin(pitch)*numpy.cos(roll))
    
    heading2 = math.atan2(eqn22_topline,eqn22_botline)*180/numpy.pi
    
    heading3 = (-heading2 + 90)%360
    
    return heading3
    
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
    
    pub = rospy.Publisher('compass_old', compass)   
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
