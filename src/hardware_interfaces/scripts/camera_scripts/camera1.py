#!/usr/bin/python
import roslib
roslib.load_manifest('hardware_interfaces')
import rospy
import sys
import cv
import time
import os
import numpy
from opencv.highgui import *
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

capture1 = cv.CaptureFromCAM(1)
cv.SetCaptureProperty(capture1, CV_CAP_PROP_FRAME_WIDTH, width)
cv.SetCaptureProperty(capture1, CV_CAP_PROP_FRAME_HEIGHT, height)

################################################################################
################################################################################

def record():
    global camera1_on
    time_zero = time.time()
    font      = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1, 8) #Creates a font        
    output    = camera_info()
    frame1    = 0
    size1_old = 0
    frame1size= 0
    
    rospy.loginfo('Camera1 node ready')
    
################################################################################
    while not rospy.is_shutdown():
        time.sleep(0.01)
        dt = time.time() - time_zero
        
                
        while (dt >= 1.0/frame_rate) and camera1_on:
            time_zero = time.time()
            dt = 0.0
            
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
            ####################################################################
 
                                
################################################################################
################################################################################

def camera_callback(data):
    global camera1_on
    global camera1_video
    global file1
       
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
    
    camera1_on = False    
    
    rospy.loginfo('Camera1 node online')
        
    rospy.on_shutdown(shutdown)

    record()
