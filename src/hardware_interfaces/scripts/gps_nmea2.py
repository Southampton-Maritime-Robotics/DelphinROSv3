#!/usr/bin/python

"""
A driver for GPS.

The gps device works at 1Hz by default. Within one cycle, it provices an information via a number of message type (see a variable named "identifier" in this file). The driver is written in a way that it will publish only once in one cycle when the getAll flag is raise.

#######################################################
#Modifications to code
15/2/2012 converted speed to m/s
6/1/2012 modified calculation of X and Y variables
6/1/2012 modified code so position is always published
6/1/2012 lat/long origin now read from parameter server
10/4/2015 have this node publish only when the getAll flag is raise. 
16/4/2015 control rate by rospy.Rate()

########################################################
#Notes
X corresponds to East
Y corresponds to North
GPS update is constrains to be prercisely 1Hz. It can be less frequence if required.

"""

import rospy
import serial
import time
import numpy
import math
import string
from pylab import *
from hardware_interfaces.msg import gps
from hardware_interfaces.msg import status

################### GLOBAL VARIABLES ################### 

global serialPort
global identifier
global latitude
global lat_NorS
global longitude
global long_WorE
global time_gps
global number_of_satelites
global fix
global speed #in knots

################### SERIAL SETUP ################### 

def setUpSerial():
    global serialPort
    serialPort = serial.Serial(port='/dev/usbgps', baudrate='4800', timeout=0.1) # may need to change timeout if having issues!
    serialPort.bytesize = serial.EIGHTBITS
    serialPort.stopbits = serial.STOPBITS_ONE
    serialPort.parity = serial.PARITY_NONE
    print "Initialised GPS Sgpggaerial."
    print serialPort.portstr
    print serialPort.isOpen()
    return serialPort.isOpen()

################### Functions to process different types of incoming message ###################  

def gpgga(string):
    global latitude
    global lat_NorS
    global longitude
    global long_WorE
    global time_gps
    global number_of_satelites
    global fix

    try:
        time_gps  = float(string[1])
        latitude  = string[2]
        lat_NorS  = string[3]
        longitude = string[4]
        long_WorE = string[5]
        number_of_satelites = int(string[7])
        fix = int(string[6])
    except: 
        pass

def gpgsa(string):
    pass  # a=0 #Incoming message contains details of satelites IDs.  This is currently of no use to us so will be ignored

def gprmc(string):
    global latitude
    global lat_NorS
    global longitude
    global long_WorE
    global time_gps
    global speed #in m/s
    global getAll
    
    getAll = True

    try:
        time_gps  = float(string[1])
        latitude  = string[3]
        lat_NorS  = string[4]
        longitude = string[5]
        long_WorE = string[6]
        speed     = float(string[7])*0.514444444    # Convert from knots to m/s
    except:
        pass

    #magnetic_variation = float(string[10])		#Information doesnt appear to be received, perhaps a different type of GPS is needed
    #magvar_WorE = string[11]       

def gpgsv(string):
    global number_of_satelites
    if len(string[3]) == 1:
        number_of_satelites = int(string[3])

################### MAIN LOOP ################### 

def validDataCheck():
    global serialPort
    global identifier
    attempt     = 0
    attempt_lim = 5

    while attempt < attempt_lim:
        while not serialPort.read(1) == '$':
            pass
        #The following line has been left for clarity of the change required with new Python versions
        ##data = serialPort.readline(size = None, eol = '\n')   #Do NOT uncomment this line!
        data = serialPort.readline()                        
        split_data = string.split(data,',')	#Split message by comma separator
        print 'DataCHECK: ', data

        message_identifier = split_data[0]	
        if message_identifier in identifier:
            return True
        else:
            if attempt >= attempt_lim:
                return False    
            else:
                attempt = attempt + 1                           ##or += attempt

def listenForData(status):

    #Establish Global Variables
    global      latitude
    global      lat_NorS
    global      longitude
    global      long_WorE
    global      time_gps	
    global      number_of_satelites 
    global      fix
    global      speed
    global      getAll

    gpsOut = gps() # initialise message structure

    #Initialise values to zero - important as different messages contain different values
    latitude		=	'0'	
    latitude_decimal	=	0
    lat_NorS		=	'0'
    longitude		=	'0'
    longitude_decimal	=	0
    long_WorE		=	'0'
    time_gps		=	0
    number_of_satelites =	0
    fix			=	0
    speed 		= 	0
    X			= 	0
    Y			= 	0

    controlRate = 1. # Hz
    r = rospy.Rate(controlRate)
    controlPeriod = 1/controlRate
    
    try: 
        lat_orig = rospy.get_param('lat_orig')
        long_orig = rospy.get_param('long_orig')
    except:
        lat_orig = 50.9567
        long_orig = -1.36735

    first_reading = True
    mean_earth_radius = 6370973.27862					#metres
    
    while not rospy.is_shutdown():

        timeRef = time.time()
        try:

            # reset the "get all data" flag
            getAll = False
            
            while serialPort.inWaiting() > 0 and serialPort.read(1) == '$': #while there is data to be read - read the line...
            
                serialPort.flushInput()
                
                timeRef = time.time()
                
                pubStatus.publish(nodeID = 4, status = status)
                data = serialPort.readline() 		#Read in line of data
                split_data = string.split(data,',')				#Split message by comma separator
                print 'Data: ', data

                message_identifier = split_data[0]				#Message identifier is the first part of the message and defines message format
                identifier[message_identifier](split_data)			#Process message according to indentifier type
                
                #### Function to convert latitude to decimal degrees ####
                if len(latitude) == 9:
                    minutes = numpy.float64(latitude[2:9])/60                   #numpy.float64 inserted instead of standard float
                    degrees = int(latitude[0:2])
                    latitude_decimal = degrees+minutes
                    if lat_NorS == 'S':
                        latitude_decimal = -latitude_decimal
                
                #### Function to convert longitude to decimal degrees ####
                if len(longitude) == 10:
                    minutes = numpy.float64(longitude[3:10])/60                 #numpy.float64 inserted instead of standard float
                    degrees = int(longitude[0:3])
                    longitude_decimal = degrees+minutes
                    if long_WorE == 'W':
                        longitude_decimal = -longitude_decimal

                #### Calculate X Y co-ordinates from original position ####
                try: #first_reading == False and len(latitude) == 9 and len(longitude) == 10:
                    Range=distanceTwoLatLong(lat_orig,latitude_decimal,long_orig,longitude_decimal)
                    Bearing=bearingTwoLatLong(lat_orig,latitude_decimal,long_orig,longitude_decimal)
                    BearingRad=math.radians(Bearing) 
                    X = Range*sin(BearingRad)
                    Y = Range*cos(BearingRad)
                except:
                    pass
                
                #Publish data to gps_out
                if getAll:
                    gpsOut.latitude = latitude_decimal 
                    gpsOut.longitude = longitude_decimal 
                    gpsOut.time = time_gps
                    gpsOut.number_of_satelites = number_of_satelites
                    gpsOut.fix = fix
                    gpsOut.speed = speed
                    gpsOut.x = X
                    gpsOut.y = Y
                    pub.publish(gpsOut)
                    print "========= get all =========="
                    
                    timeElapse = time.time()-timeRef
                    if timeElapse < controlPeriod:
                        r.sleep()
                    else:
                        str = "GPS rate does not meet the desired value of %.2fHz: actual control rate is %.2fHz" %(controlRate,1/timeElapse) 
                        rospy.logwarn(str)
                        pubMissionLog.publish(str)

        except:
            print 'read error'

def distanceTwoLatLong(lat1,lat2,lon1,lon2): #returns distance between two locations in lat/long in meters.
# Code from http://www.movable-type.co.uk/scripts/latlong.html
    R = 6371000 #Radius of the earth in m
    dLat = math.radians(lat2-lat1)
    dLon = math.radians(lon2-lon1)
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)

    a = math.sin(dLat/2)*math.sin(dLat/2)+math.sin(dLon/2)*math.sin(dLon/2)*math.cos(lat1)*math.cos(lat2)
    c = 2*math.atan2(math.sqrt(a),math.sqrt(1-a)) 
    d = R*c
    return d

def bearingTwoLatLong(lat1,lat2,lon1,lon2): #returns bearing between two locations in lat/long in degrees.
# Code from http://www.movable-type.co.uk/scripts/latlong.html
    dLat = math.radians(lat2-lat1)
    dLon = math.radians(lon2-lon1)
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dLon);
    brng = math.atan2(y, x)
    brng= math.degrees(brng)

    return brng

################### SHUTDOWN FUNCTION ################### 
def shutdown():
    serialPort.flushInput()
    serialPort.flushOutput()
    pubStatus.publish(nodeID = 4, status = False)
    serialPort.close()

################### INITIALISING FUNCTION ################### 

if __name__ == '__main__':
    time.sleep(4) #Allow System to come Online    
    global identifier
    global pub
    global pubMissionLog
    rospy.init_node('gps_sensor')
    rospy.on_shutdown(shutdown)         #Defining shutdown behaviour  
       
    identifier = {'GPGGA' : gpgga,
                'GPGSA' : gpgsa,
                'GPRMC' : gprmc,
                'GPGSV' : gpgsv,
    }
  
    #Define Publishers
    pubStatus = rospy.Publisher('status', status)
    pub = rospy.Publisher('gps_out', gps)
    pubMissionLog = rospy.Publisher('MissionStrings', String)
    time.sleep(1)
    #Setup serial port and check its status
    port_status = setUpSerial()    
    str = "GPS port status = %s. Port = %s" %(port_status, serialPort.portstr)
    rospy.loginfo(str)
    
    #Initial Data Test
    
    string_status=validDataCheck()
    
    if (port_status and string_status) == True:   
        status = True 
        pubStatus.publish(nodeID = 4, status = status)
        rospy.loginfo("GPS online")
    else:
        status = False
        pubStatus.publish(nodeID = 4, status = status)    

    listenForData(status)                     #Main loop for receiving data

