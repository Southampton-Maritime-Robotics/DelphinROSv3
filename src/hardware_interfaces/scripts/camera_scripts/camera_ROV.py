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
from std_msgs.msg import Bool

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

window_size_scale = 1.0

cv.NamedWindow('Forwards Cam', 0)
cv.ResizeWindow('Forwards Cam',int(width*window_size_scale),int(height*window_size_scale))

cv.NamedWindow('Downwards Cam', 0)
cv.ResizeWindow('Downwards Cam',int(width*window_size_scale),int(height*window_size_scale))


################################################################################
################################################################################

def record():
    global camera0_on
    global camera1_on
    pub_light.publish(1)
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
            if camera0_on == True and camera1_on == True:
                cam0   = cv.QueryFrame(capture0)
                cv.ShowImage('Forwards Cam', cam0)
                frame0 = frame0 + 1
                str0   = '%s' %(frame0)
                cv.PutText(cam0,str0, (10,height-15),font, [255, 255, 255]) #Draw the text
                cv.WriteFrame(camera0_video, cam0)
                

                cam1 = cv.QueryFrame(capture1)
                #cv.ShowImage('Downwards Cam', cam1)
                frame1 = frame1 + 1
                str1 = '%s' %(frame1)
                cv.PutText(cam1,str1,(10,height-15),font, [255, 255, 255]) #Draw the text
                cv.WriteFrame(camera1_video, cam1)
                
                c = cv.WaitKey(10)
                
                output.cam1frame = frame1
                output.cam0frame = frame0
                pub.publish(output)
                
            ####################################################################
                
                                
################################################################################
################################################################################
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
    global camera0_on
    global camera1_on
    global camera0_video
    global camera1_video
    global file0
    global file1
    
    folder = data.data
    
    camera0_on = True
    file0      = (folder + '/forwards.avi')
    camera0_video = cv.CreateVideoWriter((folder + '/' + 'forwards.avi'), cv.CV_FOURCC('J','P','G','L'), frame_rate, (width,height), 1) 
        
    camera1_on = True
    file1      = (folder + '/downwards.avi')
    camera1_video = cv.CreateVideoWriter((folder + '/' + 'downwards.avi'), cv.CV_FOURCC('J','P','G','L'), frame_rate, (width,height), 1) 
    
    print 'Video folder address received'
    
################################################################################
    
def shutdown():
    time.sleep(2)
    
################################################################################

if __name__ == '__main__':
    rospy.init_node('Camera')
    
    rospy.Subscriber('dead_reckoner', dead_reckoner, DR_callback)
    rospy.Subscriber('vidfolder', String, folder_callback)
    pub  = rospy.Publisher('camera_info', camera_info)
    pub_light= rospy.Publisher('light_onOff', Bool)
    
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
    
    
    rospy.loginfo('Camera node online')
        
    rospy.on_shutdown(shutdown)

    record()
