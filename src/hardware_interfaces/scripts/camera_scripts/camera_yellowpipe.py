#!/usr/bin/python
import roslib
roslib.load_manifest('hardware_interfaces')
import rospy
import numpy as np
import cv
import time
import random
import sys
from scipy import weave
from scipy.weave import converters
from hardware_interfaces.msg import pipe_camera

################################################################################
#### INITIALISE ################################################################
################################################################################

def run():
    
    lim_hue_min = 15 * 180.0/360.0                                                   # Min hue value
    lim_hue_max = 40 * 180.0/360.0                                                     # Max hue value
    lim_sat     = 70 * 255.0/100.0                                                      # Min sat value
    lim_lum     = 100 * 255.0/100.0
    
    ######## height/width thresholds < which blobs will not be acknolodged #####
    threshWidth = 30
    threshHeight = 30
    
    r = imageHSL.height
    c = imageHSL.width
    B = np.zeros([r,c],np.uint8)
    
    time_zero = time.time()
    dt = 0.1
        
    ######## Start main loop ###################################################
    while not rospy.is_shutdown():
        
        while time.time() - time_zero >= dt:
            print 'Frequency = ',1/(time.time()-time_zero)
            print '###################'
            
            time_zero = time.time()
            
        ######## Get and prepare image #############################################
            rawImage=cv.QueryFrame(g_capture)
            
            cv.CvtColor(rawImage, rawImage, cv.CV_BGR2HLS)                         # Convert input image from BGR colour space to HSL
            cv.PyrDown(rawImage, imageHSL)                                         # Downsamples image

        ######## Detect color pixels ###############################################    
            x = cv2array(imageHSL)                                                 # Convert image into numpy array

            code = """
                       double h, l, s;            
                       
                       for (int i=0; i<r; i++) {
                           for (int j=0; j<c; j++) {
                                h = x(i,j,0);
                                l = x(i,j,1);
                                s = x(i,j,2);
                                
                                                
                                if ((h >= lim_hue_min) && (h <= lim_hue_max) && (l <= lim_lum) && (s >= lim_sat)) {
                                    B(i,j) = 255;
                                }
                                else {
                                    B(i,j) = 0;
                                }
                           }
                        }
                                                       
                       return_val = 1;
                       """  

            h = weave.inline(code,
                           ['x','B', 'r', 'c','lim_hue_min','lim_hue_max','lim_lum','lim_sat'],
                           type_converters = converters.blitz,
                           compiler = 'gcc')
                           
            binary2 = array2cv(B)      
            #print 'Time new = ',time.time() - time_zero
            
        ######### Smooth and detect contours #######################################
            
            cv.Smooth(binary2, binarySmooth, cv.CV_MEDIAN, 9, 9);                   # Smooth image with a median filter to remove noise
            
            storeYellowBlobs = cv.CreateMemStorage(0)                              # Define memory storage - may not be needed? 0 = approx 64k memory
            contours = cv.FindContours(binarySmooth, storeYellowBlobs, cv.CV_RETR_EXTERNAL, cv.CV_CHAIN_APPROX_SIMPLE)
            
        ######## Detect color pixels ###############################################
            
            bestBlobSize = 0                                                       # Initialise
            bestRect = (0,0,0,0)                                                   # (x, y, width, height)
            
            while (type(contours) != type(None)) and (len(contours)): 

                rect = cv.BoundingRect(contours, 0)	                               # Fit rectangle around contour value 0

                if (rect[2] > threshWidth and rect[3] > threshHeight):             # Check dimensions of rectangle are greater than thresholds
                                        
                    color = cv.CV_RGB(255,255,255)
                    cv.DrawContours(yellowBlobs, contours, color, color, -1, cv.CV_FILLED, 8 ) #Draw contours 
                
                    blobSize = cv.Round(cv.ContourArea(contours))                  # Area of blob
                    
                    if (blobSize > bestBlobSize):                                  # Check if it is the largest blob on this image so far
                        bestRect = rect
                        bestBlobSize = blobSize 
                        targetID = 2 
                        
                contours = contours.h_next()

        ######## Fit rectangle around blob #########################################
            cv.Rectangle(yellowBlobs, (bestRect[0],bestRect[1]),(bestRect[0]+bestRect[2],bestRect[1]+bestRect[3]), cv.CV_RGB(255,255,255))
            cv.Line(yellowBlobs, (0,80), (yellowBlobs.width, 80), cv.CV_RGB(255,0,0) , 1, 8)

        ######## Fit rectangle around blob #########################################
            cv.ShowImage('orange_Blob', yellowBlobs);
            cv.ShowImage('Image', imageHSL) #Show the image

        ######## Check location of centre of blob ##################################
            yellowCount = splitNxN(4) # Split the image into a 4x4 grid

            centreX = (bestRect[0] + bestRect[2])/2 #FOR RETURNING TO ROS
            centreY = (bestRect[1] + bestRect[3])/2 #bestRect[2] = height, bestRect[3] = width

        ######## PUBLISH ###########################################################
            camera_data.pixelCount = yellowCount
            camera_data.bestBlobSize = bestBlobSize
            camera_data.centreX = centreX
            camera_data.centreY = centreY
            cam_pub.publish(camera_data)
            
            cv.WaitKey(10)
            cv.Zero(yellowBlobs)                                                   # Clear matrix - otherwise end up with a tracer
        
################################################################################
################################################################################

def splitNxN(n):
    """Splits an image into an NxN grid, returning an array of the
    number of yellow pixels in each grid area."""
    
    yellowCount = []
    
    w_step = yellowBlobs.width/n
    h_step = yellowBlobs.height/n

    for i in range(n):
        tx = w_step * i
        for j in range(n):
            ty = h_step * j
            subRect = cv.GetSubRect(binarySmooth, (tx, ty, w_step, h_step))
            yellowCount.append(cv.CountNonZero(subRect))


    return yellowCount

################################################################################
################################################################################

def cv2array(im):
  depth2dtype = {
        cv.IPL_DEPTH_8U: 'uint8',
        cv.IPL_DEPTH_8S: 'int8',
        cv.IPL_DEPTH_16U: 'uint16',
        cv.IPL_DEPTH_16S: 'int16',
        cv.IPL_DEPTH_32S: 'int32',
        cv.IPL_DEPTH_32F: 'float32',
        cv.IPL_DEPTH_64F: 'float64',
    }

  arrdtype=im.depth
  a = np.fromstring(
         im.tostring(),
         dtype=depth2dtype[im.depth],
         count=im.width*im.height*im.nChannels)
  a.shape = (im.height,im.width,im.nChannels)
  return a

def array2cv(a):
  dtype2depth = {
        'uint8':   cv.IPL_DEPTH_8U,
        'int8':    cv.IPL_DEPTH_8S,
        'uint16':  cv.IPL_DEPTH_16U,
        'int16':   cv.IPL_DEPTH_16S,
        'int32':   cv.IPL_DEPTH_32S,
        'float32': cv.IPL_DEPTH_32F,
        'float64': cv.IPL_DEPTH_64F,
    }
  try:
    nChannels = a.shape[2]
  except:
    nChannels = 1
  cv_im = cv.CreateImageHeader((a.shape[1],a.shape[0]),
          dtype2depth[str(a.dtype)],
          nChannels)
  cv.SetData(cv_im, a.tostring(),
             a.dtype.itemsize*nChannels*a.shape[1])
  return cv_im

################################################################################
#### INITIALISE ################################################################
################################################################################

if __name__ == '__main__':
    
    rospy.init_node('camera_yellowpipe')
    
    global cam_pub
    global imageHSL
    global yellowBlobs
    global binary
    global binarySmooth
    global camera_data
    global g_capture
    
    g_capture    = cv.CaptureFromCAM(1)  
    camera_data  = pipe_camera()
    cam_pub      = rospy.Publisher('pipe_camera', pipe_camera)
    
    initImage    = cv.QueryFrame(g_capture)  
    imageHSL     = cv.CreateImage((initImage.width/2, initImage.height/2) ,cv.IPL_DEPTH_8U, 3)
    yellowBlobs  = cv.CreateImage(cv.GetSize(imageHSL), cv.IPL_DEPTH_8U, 1)
    binary       = cv.CreateImage(cv.GetSize(imageHSL), cv.IPL_DEPTH_8U, 1)
    binarySmooth = cv.CreateImage(cv.GetSize(imageHSL), cv.IPL_DEPTH_8U, 1)
    
    cv.NamedWindow('Image', cv.CV_WINDOW_AUTOSIZE)
    cv.NamedWindow('orange_Blob', cv.CV_WINDOW_AUTOSIZE)
    
    run()
