#!/usr/bin/python

"""
Driver for a pressure transducer to measure depth

keys parameters 
-depth: the raw measurement from the pressure transducer
-depth_calib: the depth calibrated with the pitch angle of the AUV
-depth_filt: the depth reading that is filtered by the polynomial-type (PT) filter
-depth_der: the derivative of depth that is determined from the gradient of PT filter

######################################
# TODO 

"""

import rospy
import numpy
import serial
import time
import math
from re import findall
from std_msgs.msg import String
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
    print "Initialised DepthTransducer serial."
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
    
    # to control a timing for status publishing
    timeZero_status = time.time()
    dt_status = rospy.get_param('status_timing')
    depth_glitch_delta = rospy.get_param('depth/GlitchDelta')

    depth_base = None # for calibrating the depth sensor: ensuring zero depth when launch at surface
    
    while not rospy.is_shutdown():    
        timeRef = time.time()
        previous_depth = 0  # for removing glitches, compare current sensor read to previous
        try:
#            time.sleep(0.01)  # Prevents node from using 100% CPU!!
            while serialPort.inWaiting() > 0 and serialPort.read(1) == '$':     #while there is data to be read - read the line...
                # to control a timing for status publishing
                if time.time()-timeZero_status > dt_status:
                    timeZero_status = time.time()
                    pubStatus.publish(nodeID = 5, status = True)
                
                dataRaw = serialPort.readline()
                                
                data = numpy.array((findall('[-]*\d+.\d+',dataRaw)), numpy.float)
                try:
                    if depth_base == None:
                        #Convert ADC value to depth value: the last term is for calibration
                        depth_base = (data[4]*42.045) - 0.15 # assume that the auv is already submerge by 0.15m
                        depth_read = 0
                    else:
                        #Convert ADC value to depth value: the last term is for calibration
                        depth_read = (data[4]*42.045) - depth_base

                    # check if a glitch to a too large value happened
                    # only a step *up* to a *larger* depth value will be detected
                    if (depth_read - previous_depth) > depth_glitch_delta:
                        depth = previous_depth
                    else:
                        depth = depth_read
                    previous_depth = depth_read  # use read value, in case of actual change it will be used next time
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
                    
                    #Publish data
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
            str = "depth_transducer rate does not meet the desired value of %.2fHz: actual control rate is %.2fHz" %(controlRate,1/timeElapse) 
            rospy.logwarn(str)
            pubMissionLog.publish(str)

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
    rospy.init_node('depth_transducer')
    global serialPort
    global depth_msg
    depth_msg = depth()
    
    pub = rospy.Publisher('depth_out', depth)
    pubMissionLog = rospy.Publisher('MissionStrings', String)
    rospy.Subscriber('compass_out', compass, compass_callback) 
    pubStatus = rospy.Publisher('status', status)
    
    rospy.on_shutdown(shutdown)
    
    port_status = setUpSerial()
    time.sleep(0.3)
    string_status=validDataCheck()
    
    if (port_status and string_status) == True:    
        status = True
        pubStatus.publish(nodeID = 5, status = status)
        rospy.loginfo("depth_transducer online")
    else:
        status = False
        pubStatus.publish(nodeID = 5, status = status)
    
    listenForData(status)   #Main loop for receiving data
