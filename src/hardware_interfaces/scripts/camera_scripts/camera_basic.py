#!/usr/bin/python
import roslib
roslib.load_manifest('hardware_interfaces')
import rospy
import sys
import cv
import time
import os
import csv
import numpy
from opencv.highgui import *
from std_msgs.msg import String

#savevideo = cv.CreateVideoWriter('testvideo.mpg', cv.CV_FOURCC('P','I','M','1'), 20.0, (320,240), 1) 


#cv.NamedWindow('cam1', cv.CV_WINDOW_AUTOSIZE)
#cv.NamedWindow('cam2', cv.CV_WINDOW_AUTOSIZE)
#cv.NamedWindow('cam3', cv.CV_WINDOW_AUTOSIZE)



#capture1 = cv.CaptureFromCAM(1)
#capture2 = cv.CaptureFromCAM(2)
#capture3 = cv.CaptureFromCAM(3)

#cv.SetCaptureProperty(capture0, CV_CAP_PROP_FRAME_WIDTH, 352)
#cv.SetCaptureProperty(capture0, CV_CAP_PROP_FRAME_HEIGHT, 288)
#cv.SetCaptureProperty(capture1, CV_CAP_PROP_FRAME_WIDTH, 352)
#cv.SetCaptureProperty(capture1, CV_CAP_PROP_FRAME_HEIGHT, 288)
#cv.SetCaptureProperty(capture2, CV_CAP_PROP_FRAME_WIDTH, 352)
#cv.SetCaptureProperty(capture2, CV_CAP_PROP_FRAME_HEIGHT, 288)
#cv.SetCaptureProperty(capture3, CV_CAP_PROP_FRAME_WIDTH, 352)
#cv.SetCaptureProperty(capture3, CV_CAP_PROP_FRAME_HEIGHT, 288)

def repeat():
    
    framerate = 5
    width = 704 #352
    height= 576 #288
    
    time_zero = time.time()
    #cv.NamedWindow('cam0', cv.CV_WINDOW_AUTOSIZE)
    capture0 = cv.CaptureFromCAM(1)
    cv.SetCaptureProperty(capture0, CV_CAP_PROP_FRAME_WIDTH, width)
    cv.SetCaptureProperty(capture0, CV_CAP_PROP_FRAME_HEIGHT, height)
    
    #lossless = cv.CreateVideoWriter('xvid.avi', cv.CV_FOURCC('X','V','I','D'), framerate, (width,height), 1) 
    lossless = cv.CreateVideoWriter('lossless.avi', cv.CV_FOURCC('J','P','G','L'), framerate, (width,height), 1) 
    
    
    
    font      = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1, 8) #Creates a font        

    
    size0_old = 0
    size1_old = 0
    
    frame = 0
    
    while True:
        while time.time() - time_zero > 1/framerate:
            frame = frame + 1
            time_zero = time.time()
            
            frame0 = cv.QueryFrame(capture0)
            #print frame0
            
            str0   = '%s' %(frame)
            cv.PutText(frame0,str0, (10,height-15),font, [255, 255, 255]) #Draw the text
            
            #cv.WriteFrame(xvid, frame0)
            cv.WriteFrame(lossless, frame0)
            
#            size0      = os.path.getsize('xvid.avi')
#            frame0size = size0 - size0_old
#            size0_old  = size0
            
#            print 'Xvid size = ', frame0size
            
            size1      = os.path.getsize('lossless.avi')
            frame1size = size1 - size1_old
            size1_old  = size1
            
            print 'Lossless size = ', frame1size
            
#            sms_info_List = [frame, frame0size, frame1size]
            
#            with open('cam_infoLog.csv', "a") as f:
#                try:
#                    Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
#                    Writer.writerow(sms_info_List)
#                except ValueError:
#                    print 'writerow error'

            #cv.ShowImage('cam0', frame0)
            #cv.ShowImage('cam1', frame1)
    #        cv.ShowImage('cam2', frame2)
    #        cv.ShowImage('cam3', frame3)
    #        cv.SaveImage('cam0.png', frame0)
    #	cv.SaveImage('cam1.png', frame1)
            c = cv.WaitKey(10)
        

if __name__ == '__main__':
    
    
    repeat()
