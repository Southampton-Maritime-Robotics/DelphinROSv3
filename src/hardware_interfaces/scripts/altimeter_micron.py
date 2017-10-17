#!/usr/bin/python

"""
A driver for an altertude w.r.t. to the sea bottom.

- tritech micron altimeter at baud rate 9600, using 3P2 message type ("xxx.xxm")
- the tritech micron has a fixed factory set rate which cannot be changed
- the maximum rate was determined by increasing the rate until the elapsed time becomes too large;
  at a rate of ~5 Hz the rate can no longer be kept, probably due to the fixed sensor rate
- 4 Hz was selected as a feasible, reasonable rate.
The sensor is very noisy, therefore, filtered by the Polynomial-Type (PT) filtering technique.
Note: The altitude measurement is only accurate up to ~0.3 m! After this value it seems to often jump to 0.

#######################################################
# TODO
- include a pitch angle of the AUV into the altitude measurement;
  though depending on the purpose of the altitude measurement this may make more or less sense:
  the altimeter is currently right next to the camera, so controlling for a camera altitude based
  on the altimeter distance rather than an altitude compensated for pitch may make more sense

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
    
    control_rate = 4.  # Hz
    controlPeriod = 1/control_rate
    r = rospy.Rate(control_rate)
    
    # to control a timing for status publishing
    timeZero_status = time.time()
    dt_status = rospy.get_param('status_timing')

    altitude_glitch_delta = rospy.get_param('altimeter/GlitchDelta')

    while not rospy.is_shutdown():
        previous_altitude = 0.
    
        while serialPort.inWaiting() > 8:
            # to control a timing for status publishing
            if time.time()-timeZero_status > dt_status:
                timeZero_status = time.time()
                pubStatus.publish(nodeID = 3, status = True)
            
            try:                       # while there is data to be read - read the line
                data = serialPort.readline()
                if data[6:7] == 'm':
                    print 'data = ', data[0:6]

                    altitude = float(data[0:6])
                    if previous_altitude - altitude < altitude_glitch_delta:
                        [altitude_filt, altitude_der] = filter(altitude)
                    else:
                        # use previous altitude to ignore altitude glitch to too low altitude
                        # if there was actually a 1 m jump in altitude, the next measurement will
                        # not be ignored as well, since the current altitude is still used as previous altitude
                        [altitude_filt, altitude_der] = filter(previous_altitude)
                    previous_altitude = altitude

                    print 'altitude = ', altitude
                    pub.publish(altitude=altitude, altitude_filt=altitude_filt, altitude_der=altitude_der)
                    time_zero = time.time()
            except:
                print 'passed'
                pass
            
        if time.time() - time_zero > timeout:
            pubStatus.publish(nodeID = 3, status = False)
            rospy.loginfo("Altimeter has gone offline")
        
        time_elapsed = time.time()-time_zero
        print("time elapsed: " + str(time_elapsed))

        if time_elapsed < controlPeriod:
            r.sleep()
        else:
            print_txt = "Altimeter_micron rate does not meet the desired value of %.2fHz: actual control rate is %.2fHz" %(control_rate,1/time_elapsed)
            rospy.logwarn(print_txt)
            pubMissionLog.publish(print_txt)
        
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

    output_str = "Altimeter port status = %s. Port = %s" %(port_status, serialPort.portstr)
    rospy.loginfo(output_str)
    time.sleep(0.3)
    string_status = validDataCheck()
    
    if (port_status and string_status) is True:
        status = True
        pubStatus.publish(nodeID = 3, status = status)
        rospy.loginfo("Altimeter online")
    else:
        status = False
        pubStatus.publish(nodeID = 3, status = status)
    
    listenForData(status)
