#!/usr/bin/python

"""
The one and only node that subscribe to all topics. 
When the new message is receive it will be kept in the .csv file for post-processing.

To log new topic
-import the message
-initialise a subscriber that subscribe to a particular topic with a particular message structure
-create a callback function that open, write and close the corresponding .csv file.

"""
import rospy
import csv
import time
import numpy
import os
from datetime import datetime
from hardware_interfaces.msg import status

from xsens_driver.msg import IMU_msg
from hardware_interfaces.msg import compass
from hardware_interfaces.msg import depth
from hardware_interfaces.msg import tsl_setpoints
from hardware_interfaces.msg import tsl_feedback
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from std_msgs.msg import String
from hardware_interfaces.msg import tail_setpoints
from hardware_interfaces.msg import tail_feedback
from hardware_interfaces.msg import position
from hardware_interfaces.msg import gps
from hardware_interfaces.msg import altitude
from hardware_interfaces.msg import sonar
from hardware_interfaces.msg import dead_reckoner
from lowlevel_controllers.msg import heading_control
from lowlevel_controllers.msg import depth_pitch_control
from hardware_interfaces.msg import camera_info
from hardware_interfaces.msg import sonar_data
from hardware_interfaces.msg import SMS
from hardware_interfaces.msg import gyro

from hardware_interfaces.msg import ForcesAndMoments    #Only temporary

global tempWriter
global tempFile
global compassWriter
global compassFile
global tslHorzWriter
global tslHorzFile
global tslHorz
global tslVertWriter
global tslVertFile
global tslVert
global tsl_feedback
global heading_demand
global depth_demand
global prop_demand
global tailFile
global tailWriter
global tail_sp
global positionFile
global positionWriter
global gpsFile
global altimeterFile
global sonarFile
global sonarWriter
global time_zero
global pathFile
global log_folder

##############################################################

def callback_compass(compass_data):

    stringtime = time.time()-time_zero
    compassList = [stringtime, 
                   compass_data.heading, 
                   compass_data.roll, 
                   compass_data.pitch, 
                   compass_data.temperature, 
                   compass_data.ax, 
                   compass_data.ay, 
                   compass_data.az, 
                   compass_data.angular_velocity_x, 
                   compass_data.angular_velocity_y, 
                   compass_data.angular_velocity_z]

    with open('%s/compassLog.csv' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(compassList)
        except ValueError:
            print 'writerow error'

def callback_depth(depth_data):

    stringtime = time.time()-time_zero
    depthList = [stringtime, 
                   depth_data.depth, 
                   depth_data.depth_calib, 
                   depth_data.depth_filt, 
                   depth_data.depth_der]

    with open('%s/depthLog.csv' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(depthList)
        except ValueError:
            print 'writerow error'

##############################################################

#def temp_callback(temp_data):
#    #outputs temperature sensor data to file
#    stringtime = time.time()-time_zero
#    tempList = [stringtime, temp_data.data]

#    with open('%s/tempLog.csv' %(dirname), "a") as f:
#        try:
#            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
#            Writer.writerow(tempList)
#        except ValueError:
#            print 'writerow error'

##############################################################
        
def tsl_feedback_callback(data):
    
    stringtime = time.time()-time_zero
    tslList = [stringtime, 
               data.setpoint0, 
               data.setpoint1, 
               data.setpoint2, 
               data.setpoint3, 
               data.speed0, 
               data.speed1, 
               data.speed2, 
               data.speed3, 
               data.current0, 
               data.current1, 
               data.current2, 
               data.current3, 
               data.voltage]
    
    with open('%s/thrusterLog.csv' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(tslList)
        except ValueError:
            print 'writerow error'
            
##############################################################
            
def tail_feedback_callback(data):

    stringtime = time.time()-time_zero
    tailList = [stringtime, 
                data.bsp, 
                data.b, 
                data.csp, 
                data.c, 
                data.dsp, 
                data.d, 
                data.esp, 
                data.e, 
                prop_demand, 
                data.current, 
                data.rpm]
        
    with open('%s/tailLog.csv' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(tailList)
        except ValueError:
            print 'writerow error'

##############################################################

def headingdemand_callback(data):
    #updates heading_demand - to be included in horizontal thruster log file
    global heading_demand
    heading_demand = data.data
        
def depthdemand_callback(data):
    #updates depth_demand - to be included in vertical thruster log file
    global depth_demand
    depth_demand = data.data

def propdemand_callback(data):
    #updates prop_demand - to be included in tail section log file
    global prop_demand
    prop_demand = data.data
    
##############################################################

def position_callback(position):
    
    stringtime = time.time()-time_zero
    positionList = [stringtime, 
                    position.X, 
                    position.Y, 
                    position.Z, 
                    position.forward_vel, 
                    position.sway_vel, 
                    position.lat, 
                    position.long, 
                    position.altitude, 
                    position.ValidGPSfix]
    
    with open('%s/positionLog.csv' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(positionList)
        except ValueError:
            print 'writerow error'

##############################################################

def mission_callback(message): 
    stringtime = time.time()-time_zero
    missionList = [stringtime, message.data]
    
    with open('%s/mission.txt' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(missionList)
        except ValueError:
            print 'writerow error'

##############################################################

def gps_callback(gps):
    stringtime = time.time()-time_zero
    gpsList = [stringtime, 
               gps.latitude, 
               gps.longitude, 
               gps.time, 
               gps.number_of_satelites, 
               gps.fix,gps.speed,
               gps.x,gps.y]  
    
    with open('%s/gpsLog.csv' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(gpsList)
        except ValueError:
            print 'writerow error'

##############################################################

def altimeter_callback(altimeter):
    stringtime = time.time()-time_zero
    altimeterList=[stringtime, 
                   altimeter.altitude, 
                   altimeter.altitude_filt, 
                   altimeter.altitude_der]
    
    with open('%s/altimeterLog.csv' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(altimeterList)
        except ValueError:
            print 'writerow error'

##############################################################

def sonar_callback(sonar): ## Problem with logging raw data

    try:
        stringtime = time.time()-time_zero
        
        raw = list(numpy.fromstring(sonar.rawData, dtype = numpy.uint8))
                
        sonarData = [stringtime, sonar.transBearing, sonar.pitch, sonar.TargetRange, sonar.meanIntinsity ,raw]
        
        with open('%s/sonarLogwithRaw.csv' %(dirname), "a") as f:
            try:
                Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                Writer.writerow(sonarData)
            except ValueError:
                print 'writerow error'
        
        sonarData = [stringtime, sonar.transBearing, sonar.pitch, sonar.TargetRange, sonar.meanIntinsity]
        with open('%s/sonarLog.csv' %(dirname), "a") as f:
            try:
                Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                Writer.writerow(sonarData)
            except ValueError:
                print 'writerow error'
                
    except ValueError:
        print 'Problem with sonar logger!!!'
        
##############################################################
        
####def reckoner_callback(data): 
####    global KMLtime
####    global reckonertime
####    stringtime = time.time()-time_zero
####        
####    reckonerList = [stringtime, data.X ,data.Y ,data.Z ,data.distance ,data.heading ,data.pitch ,data.roll ,data.velX ,data.velY ,data.velZ ,data.velP ,data.velH ,data.latitude ,data.longitude ,data.altitude ,data.ValidGPSfix, data.X_dead, data.Y_dead, data.velX_dead, data.velY_dead, data.temperature, data.frame0,data.frame1, data.velP_dead] 
####    
####    if time.time() - reckonertime >= 0.2:
####        with open('%s/reckonerLog.csv' %(dirname), "a") as f:
####            try:
####                Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
####                Writer.writerow(reckonerList)
####            except ValueError:
####                print 'writerow error'
####        reckonertime = time.time()
####            
####    #Update KML file every 5s
####    if time.time()-KMLtime>5:
####        pathLine = '%.32f,%.32f,5 \n' %(data.latitude, data.longitude)
####        
####        with open('%s/path.kml' %(dirname), "a") as f:
####            try:
####                f.write(pathLine)
####            except ValueError:
####                print 'writerow error'
####    
####        KMLtime=time.time()
        
##############################################################

def headingPID_callback(data): 
    stringtime = time.time()-time_zero
    
    headingPIDList = [stringtime, 
                      data.heading,          # no need
                      data.speed,
                      data.heading_demand,
                      data.sway_demand,
                      data.error,            # no need
                      data.int_error,        # no need
                      data.der_error,
                      data.deadzone,         # no need
                      data.CS_Pgain,         # no need
                      data.CS_Igain,         # no need
                      data.CS_Dgain,         # no need
                      data.CS_Smax ,         # no need
                      data.CS_Pterm, 
                      data.CS_Iterm,
                      data.CS_Dterm, 
                      data.CS_demand, 
                      data.Thrust_Pgain,     # no need 
                      data.Thrust_Igain,     # no need
                      data.Thrust_Dgain,     # no need
                      data.Thrust_Smax,      # no need
                      data.Thrust_Pterm, 
                      data.Thrust_Iterm, 
                      data.Thrust_Dterm, 
                      data.Thrust_heading,
                      data.thruster0, 
                      data.thruster1]
    
    with open('%s/headingPIDLog.csv' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(headingPIDList)
        except ValueError:
            print 'writerow error'

def depth_pitch_PID_callback(data): 
    stringtime = time.time()-time_zero
        
    depth_pitch_PIDList = [stringtime, 
                           data.depth,               # no need
                           data.depth_demand,
                           data.error_depth,         # no need
                           data.int_error_depth,     # no need
                           data.der_error_depth,     # no need
                           data.deadzone_Depth,      # no need
                           data.Depth_Pgain,         # no need
                           data.Depth_Igain,         # no need
                           data.Depth_Dgain,         # no need
                           data.Depth_Pterm,
                           data.Depth_Iterm,
                           data.Depth_Dterm,
                           data.Depth_Thrust,
                           data.pitchBias,
                           data.pitchBiasMax,        # no need
                           data.pitchBiasGain_P,       # no need
                           data.pitch,               # no need
                           data.pitch_demand,
                           data.error_pitch,         # no need
                           data.int_error_pitch,     # no need
                           data.der_error_pitch,     # no need
                           data.deadzone_Pitch,      # no need
                           data.Pitch_Pgain,         # no need
                           data.Pitch_Igain,         # no need
                           data.Pitch_Dgain,         # no need
                           data.Pitch_Pterm, 
                           data.Pitch_Iterm, 
                           data.Pitch_Dterm, 
                           data.cr, 
                           data.crMax,               # no need
                           data.Thrust_Smax,         # no need
                           data.thruster0, 
                           data.thruster1,
                           data.CS_Smax,             # no need
                           data.CS_Pgain,            # no need
                           data.CS_Igain,            # no need
                           data.CS_Dgain,            # no need
                           data.CS_Pterm, 
                           data.CS_Iterm, 
                           data.CS_Dterm, 
                           data.CS_demand,
                           data.pitchBiasGain_D]
    
    with open('%s/depthPitchPIDLog.csv' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(depth_pitch_PIDList)
        except ValueError:
            print 'writerow error'
            
##############################################################
            
def camera_callback(data): 
    stringtime = time.time()-time_zero
    #print 'Camera data!!!!'
        
    camera_info_List = [stringtime, data.cam0, data.cam1, data.cam0frame, data.cam1frame, data.cam0size, data.cam1size, data.latitude, data.longitude, data.depth, data.altitude, data.speed, data.heading]
    
    with open('%s/camera_infoLog.csv' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(camera_info_List)
        except ValueError:
            print 'writerow error'
            
##############################################################
            
def SMS_callback(data): 
    stringtime = time.time()-time_zero
    #print 'Camera data!!!!'
        
    sms_info_List = [stringtime, data.signal, data.depth, data.latitude, data.longitude]
    
    with open('%s/sms_infoLog.csv' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(sms_info_List)
        except ValueError:
            print 'writerow error'

##############################################################

def shutdown():
    #shutdown behaviour - close all files
    #print 'shutting down'
    with open('%s/path.kml' %(dirname), "a") as f:
        try:  
            f.write('</coordinates>\n </LineString>\n </Placemark>\n </kml>\n')
        except ValueError:
            print 'write error'

################################################################################
####### INITIALISE #############################################################
################################################################################

if __name__ == '__main__':
    rospy.init_node('Logger')
    
    stringtime = datetime.now()
    stringtime = stringtime.strftime('%Y-%m-%d_%H-%M-%S')
    rospy.loginfo('Logger started at %s.'%(stringtime))
    pub_folder = rospy.Publisher('folder', String)
    pub_vidfolder = rospy.Publisher('vidfolder', String)
    pubStatus = rospy.Publisher('status', status)
    
    global heading_demand
    global depth_demand
    global prop_demand
    global tail_horizontal_setpoints
    global tail_vertical_setpoints
    global tsl_Vert
    global tsl_Horz
    global time_zero
    global KMLtime
    global lat_orig
    global long_orig
    
    heading_demand = -1
    depth_demand   = -1
    prop_demand    = -1
    tail_horizontal_setpoints = tail_setpoints()
    tail_vertical_setpoints   = tail_setpoints()
    tsl_Vert  = tsl_setpoints()
    tsl_Horz  = tsl_setpoints()
    time_zero = time.time()
    KMLtime   = time.time()
    reckonertime = time.time()
    
    try:
        lat_orig  = rospy.get_param('lat_orig')
        long_orig = rospy.get_param('long_orig')
    except:
        lat_orig  = 0
        long_orig = 0
  
################################################################################
######## FOLDERS ###############################################################
################################################################################
  
    #define files and writers
    logfolder =  'logFiles'   
    dirname   = logfolder + '/' + stringtime
    vidfolder = dirname + '/' + 'videos'
    
    if not os.path.isdir(logfolder):
        print 'made logfolder'
        os.mkdir(logfolder)
    if not os.path.isdir(dirname):
        print 'made test folder'
        os.mkdir(dirname)
    if not os.path.isdir(vidfolder):
        print 'made video folder'
        os.mkdir(vidfolder)

    pub_folder.publish(dirname)
    pub_vidfolder.publish(vidfolder)
    
################################################################################
######## KML POSITION HEADER ###################################################
################################################################################

    with open('%s/path.kml' %(dirname), "a") as f:
            try:  
                f.write('<?xml version=\'1.0\' encoding=\'UTF-8\'?>\n')
                f.write('<kml xmlns=\'http://earth.google.com/kml/2.0\'><Placemark>\n')
                f.write('<description>Route generated by Delphin2</description>\n')
                f.write('<name>Delphin2 Path</name>\n')
                f.write('<LookAt>\n')
                str = '<longitude>%s</longitude>\n' %long_orig
                f.write(str)
                str = '<latitude>%s</latitude>\n' %lat_orig
                f.write(str)
                f.write('<range>400</range>\n')
                f.write('<tilt>25</tilt>\n')
                f.write('<heading>-5</heading>\n')
                f.write('</LookAt><visibility>1</visibility>\n')
                f.write('<open>0</open>\n')
                f.write('<Style>\n')
                f.write('<LineStyle>\n')
                f.write('<color>7f0000ff</color>\n')
                f.write('<width>4</width>\n')
                f.write('</LineStyle>\n')
                f.write('</Style>\n')
                f.write('<LineString>\n')
                f.write('<altitudeMode>relativeToGround</altitudeMode>\n')
                f.write('<coordinates>\n')
            except ValueError:
                print 'write error'

################################################################################
######## SUBSCRIBERS ###########################################################
################################################################################

    rospy.Subscriber('compass_out', compass, callback_compass)
    rospy.Subscriber('depth_out', depth, callback_depth)
    rospy.Subscriber('TSL_feedback', tsl_feedback, tsl_feedback_callback)        
    rospy.Subscriber('heading_demand', Float32, headingdemand_callback)
    rospy.Subscriber('depth_demand', Float32, depthdemand_callback)
    rospy.Subscriber('prop_demand', Int8, propdemand_callback)
    rospy.Subscriber('tail_output', tail_feedback, tail_feedback_callback)
    rospy.Subscriber('position_dead', position, position_callback)
    rospy.Subscriber('gps_out', gps, gps_callback)
    rospy.Subscriber('altimeter_out',altitude, altimeter_callback)
    rospy.Subscriber('sonar_processed', sonar_data, sonar_callback)
    rospy.Subscriber('MissionStrings', String, mission_callback)
##    rospy.Subscriber('dead_reckoner', dead_reckoner, reckoner_callback) 
    rospy.Subscriber('Heading_controller_values', heading_control, headingPID_callback)
    rospy.Subscriber('Depth_pitch_controller_values', depth_pitch_control, depth_pitch_PID_callback)
    
    rospy.Subscriber('camera_info', camera_info, camera_callback)
    rospy.Subscriber('SMS_info', SMS, SMS_callback)
#    rospy.Subscriber('water_temp', Float32, temp_callback)
    
    rospy.on_shutdown(shutdown)

    str = "Logger online - output directory: %s" %(dirname)
    rospy.loginfo(str)
    
    time.sleep(3)
    
    for i in range(15): # publish node status 10 times consecutively
        pubStatus.publish(nodeID = 10, status = True)
        time.sleep(0.1)
    
    rospy.spin()
