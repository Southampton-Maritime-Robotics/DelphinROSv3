#!/usr/bin/python
import roslib
roslib.load_manifest('hardware_interfaces')
import rospy
import sys
import cv
import time
import os
import numpy
#from opencv.highgui import *
from std_msgs.msg import String
from hardware_interfaces.msg import camera
from hardware_interfaces.msg import camera_info
from hardware_interfaces.msg import dead_reckoner

global time_zero
global frame_rate

time_zero = time.time()
frame_rate = 5.0

global width
global height

#width = 704 
#height= 576 

width = 352
height= 288

capture0 = cv.CaptureFromCAM(0)
capture1 = cv.CaptureFromCAM(1)
cv.SetCaptureProperty(capture0, CV_CAP_PROP_FRAME_WIDTH, width)  
cv.SetCaptureProperty(capture0, CV_CAP_PROP_FRAME_HEIGHT, height)
cv.SetCaptureProperty(capture1, CV_CAP_PROP_FRAME_WIDTH, width)
cv.SetCaptureProperty(capture1, CV_CAP_PROP_FRAME_HEIGHT, height)

################################################################################
################################################################################

def record():
    global camera0_on
    global camera1_on
    time_zero = time.time()
    font      = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1, 8) #Creates a font        
    output    = camera_info()
    frame0    = 0
    frame1    = 0
    size0_old = 0
    size1_old = 0
    frame0size= 0
    frame1size= 0
    
    rospy.loginfo('Camera node ready')
    
################################################################################
    while not rospy.is_shutdown():
        time.sleep(0.01)
        dt = time.time() - time_zero
        
                
        while (dt >= 1.0/frame_rate) and (camera0_on or camera1_on):
            time_zero = time.time()
            dt = 0.0
            
            
            ####################################################################
            if camera0_on == True:
                cam0   = cv.QueryFrame(capture0)
                frame0 = frame0 + 1
                str0   = '%s' %(frame0)
                cv.PutText(cam0,str0, (10,height-15),font, [255, 255, 255]) #Draw the text
                cv.WriteFrame(camera0_video, cam0)
                
                size0      = os.path.getsize(file0)
                frame0size = size0 - size0_old
                size0_old  = size0
                
                output.cam0 = numpy.int8(camera0_on)
                output.cam0frame = frame0
                output.cam0size  = frame0size
                    #print output
                
                
            ####################################################################
            
            ####################################################################
            if camera1_on == True:
                cam1 = cv.QueryFrame(capture1)
                frame1 = frame1 + 1
                str1 = '%s' %(frame1)
                cv.PutText(cam1,str1,(10,height-15),font, [255, 255, 255]) #Draw the text
                cv.WriteFrame(camera1_video, cam1)
                
                size1      = os.path.getsize(file1)
                frame1size = size1 - size1_old
                size1_old  = size1
                output.cam1frame = frame1
                output.cam1 = numpy.int8(camera1_on)
                output.cam1size  = frame1size

            ####################################################################
            
            ####################################################################
            if camera0_on or camera1_on:
                try:
                    output.latitude  = latitude
                    output.longitude = longitude
                    output.depth     = depth
                    output.altitude  = altitude
                    output.speed     = speed
                    output.heading   = heading

                    pub.publish(output)
                except ValueError:
                    print 'Problem with camera publisher'

            ####################################################################
                
                                
################################################################################
################################################################################

def camera_callback(data):
    global camera0_on
    global camera1_on
    global camera0_video
    global camera1_video
    global file0
    global file1

    
    if data.camera == 0:
        if data.record == True:
            camera0_on = True
            file0      = (folder + '/' + data.filename)
            camera0_video = cv.CreateVideoWriter((folder + '/' + data.filename), cv.CV_FOURCC('J','P','G','L'), frame_rate, (width,height), 1) 
            rospy.loginfo('Camera0 recording!')
        else:
            rospy.loginfo('Camera0 recording stopped!')
            camera0_on = False
        
    if data.camera == 1:
        if data.record == True:
            camera1_on = True
            file1      = (folder + '/' + data.filename)
            camera1_video = cv.CreateVideoWriter((folder + '/' + data.filename), cv.CV_FOURCC('J','P','G','L'), frame_rate, (width,height), 1) 
            rospy.loginfo('Camera1 recording!')
        else:
            rospy.loginfo('Camera1 recording stopped!')
            camera1_on = False

################################################################################

def DR_callback(data):
    global altitude
    global depth
    global heading
    global latitude
    global longitude
    global speed
    
    altitude  = data.altitude
    depth     = data.Z
    heading   = data.heading
    latitude  = data.latitude
    longitude = data.longitude
    speed     = data.velX
    
################################################################################

def folder_callback(data):
    global folder
    folder = data.data
    print 'Video folder address received'
    
################################################################################
    
def shutdown():
    time.sleep(2)
    
################################################################################

if __name__ == '__main__':
    rospy.init_node('Camera')
    
    rospy.Subscriber('Camera', camera, camera_callback)
    rospy.Subscriber('dead_reckoner', dead_reckoner, DR_callback)
    rospy.Subscriber('vidfolder', String, folder_callback)
    pub  = rospy.Publisher('camera_info', camera_info)
    
    global camera0_on
    global camera1_on
    global altitude
    global depth
    global heading
    global latitude
    global longitude
    global speed
    
    
    altitude  = '0.0'
    depth     = '0.0'
    heading   = '0.0'
    latitude  = '0.0'
    longitude = '0.0'
    speed     = '0.0'
    
    camera0_on = False
    camera1_on = False    
    
    rospy.Subscriber('Camera', camera, camera_callback)
    rospy.Subscriber('dead_reckoner', dead_reckoner, DR_callback)
    rospy.Subscriber('vidfolder', String, folder_callback)
    pub  = rospy.Publisher('camera_info', camera_info)
    
    rospy.loginfo('Camera node online')
        
    rospy.on_shutdown(shutdown)

    record()
