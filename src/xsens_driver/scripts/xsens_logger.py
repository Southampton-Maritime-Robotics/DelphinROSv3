#!/usr/bin/python
import rospy
import csv
import time
import numpy
import os
from datetime import datetime

from xsens_driver.msg import IMU_msg

from std_msgs.msg import Float32
from std_msgs.msg import Int8
from std_msgs.msg import String

global time_zero
global pathFile
global log_folder


##############################################################
def callback_IMU_msg(imu):
    stringtime = time.time()-time_zero
    lineList = [stringtime,
    			imu.temperature,
    			imu.orientation_roll,
    			imu.orientation_pitch,
    			imu.orientation_yaw,
    			imu.angular_velocity_x,
    			imu.angular_velocity_y,
    			imu.angular_velocity_z,
    			imu.linear_acceleration_x,
    			imu.linear_acceleration_y,
    			imu.linear_acceleration_z]

    with open('%s/IMU_Log.csv' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(lineList)
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
    
    global time_zero
    time_zero = time.time()
  
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
        
    time.sleep(5)
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
####                str = '<longitude>%s</longitude>\n' %long_orig
####                f.write(str) 
####                str = '<latitude>%s</latitude>\n' %lat_orig
####                f.write(str) 
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

    rospy.Subscriber('IMU_information', IMU_msg, callback_IMU_msg)
    rospy.Subscriber('GPS_information', GPS_msg, callback_GPS_msg)  

    str = "Logger online - output directory: %s" %(dirname)
    rospy.loginfo(str)
    
    rospy.on_shutdown(shutdown)

    rospy.spin()
    
