#!/usr/bin/python

'''
Possibly, a part of the sonar driver.

This node may not functioning!

'''

import rospy
import time
import numpy
import csv
from std_msgs.msg import String
from hardware_interfaces.msg import sonar
from hardware_interfaces.msg import sonar_data
from hardware_interfaces.msg import dead_reckoner

### FUTURE DEVELOPMENT IDEAS ###
# Variable blanking distance
# Variable sonar return threshold

################################################################
def callback(msgData):    
    ### Called when new data from the sonar is recieved ###
    rawData = msgData.data
    msgData = numpy.fromstring(msgData.data, dtype=numpy.uint8)                 # Convert from string to UINT8 numpy array
   
    pingPwr = msgData[44:-1]                                                    # Extract sonar return data, ignoring message header
    print pingPwr
    
#    ADInterval     = rospy.get_param("/ADInterval")                             # Not currently in use, see manual for more info
#    GlitchCount    = rospy.get_param("/GlitchCount")                            # Not currently in use, see manual for more info
#    ThresholdCount = GlitchCount-1                                              # Not currently in use, see manual for more info
    
    Threshold = rospy.get_param("/Threshold")                                   # Value above which is considered an object of interest
    BlankDist = rospy.get_param("/BlankDist")                                   # Distance from sonar head within which returns are ignored
    Range     = rospy.get_param("/Range")                                       # Set range of sonar ping (e.g. 20 metres)
    NBins     = rospy.get_param("/NBins")                                       # Number of bins, higher number for higher resolution but at expense of more noise
    
    BinLength = Range/float(NBins)                                              # Distance each bin covers
        
    StartBin = int(numpy.ceil(BlankDist/BinLength))                             # Calculates index of first bin after the blanking distance
      
    ReturnIndexes = numpy.flatnonzero(pingPwr[StartBin:-1]>Threshold)           # Finds and returns indices of bins with value higher than threshold
    
    ReturnIndexes = ReturnIndexes + StartBin                                    # Includes number of bins in blanking distance to give the true distance              

    transBearing = ((float(msgData[40]+(msgData[41]*256))/6400.0)*360)          # Convert back from 16th of a gradian to degrees
    transBearing = transBearing %360                                            # Mod 360

    if ReturnIndexes != []:                                                     # If no bins exceed threshold then TargetRange = -1 (no returns)
        TargetRange = ReturnIndexes[0] * BinLength                              # If there is a return, take the first return and calc distance (m) using length of each bin
    else: 
        TargetRange = -1
        
    meanIntinsity = numpy.mean(pingPwr[StartBin:-1])                            # Calculate the mean intensity of returns beyond the blanking distance. May be of interest in identifying areas of interest

    print 'bearing:',transBearing
    print 'range:',TargetRange
    print 'current time:', time.time()
    print '_____________'
    
    pub.publish(transBearing = transBearing, pitch = pitch, TargetRange = TargetRange, meanIntinsity = meanIntinsity, rawData = rawData)

    #csvWriter.writerow([transBearing, TargetRange, time.time()])                # Writing to logfile for debugging -> should be sent to logger.py in the future

################################################################
def uint8_to_uint16(input8):
    # converts two uint8s to one uint16
    output16 = input8[0]+(input8[1]*256)
    return output16

################################################################
def DR_callback(data):
    global pitch    
    pitch  = data.pitch

################################################################
def detectObstacle():
    global csvWriter
    global pub
    global pitch
    pitch = 0.0
    
    rospy.init_node('sonar_detectObstacle', anonymous = True)
    rospy.Subscriber('sonar_output', String, callback)
    rospy.Subscriber('dead_reckoner', dead_reckoner, DR_callback)
    pub  = rospy.Publisher('sonar_processed', sonar_data)

    #sonarFile = open('sonarlog.csv','w')
    #csvWriter = csv.writer(sonarFile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    time.sleep(0.01)
    rospy.spin()

################################################################
if __name__ == '__main__':
    global pub
    detectObstacle()

################################################################
##### OLD CODE THAT MAY PROVE USEFUL IN THE FUTURE #############
################################################################

#def compass_callback(compass_data):
#    global compass_heading
#    compass_heading = compass_data.heading
    
#    BinLength = ((uint8_to_uint16(ADInterval)*0.000000640)*1500)/2
#    StartBin = int(numpy.ceil(BlankDist/BinLength))
    
#    # Which bins are above the detection threshold?
#    detectIdx = numpy.flatnonzero(pingPwr[StartBin:-1]>Threshold)
    
#    detectIdx = numpy.array([1, 2, 3, 4, 5, 10, 12, 15, 17])
    
#    # Case where no glitch detection is in place
#    if ThresholdCount == 0:
#        floorIdx = detectIdx[0]+StartBin-1
#        targetRange = (floorIdx*Range)/NBins
#        print targetRange
#        transBearing = msgData[40]+(msgData[41]*256)
#        print transBearing
#        return
    
    
#    ####################BELOW CODE DOES NOT WORK!  GlitchCount must be set to 1
    
#    diffIdx = numpy.diff(detectIdx)
#    print 'DiffIdx:'
#    print diffIdx
    
#    # Setup a vector of zeros
#    tmpVec = numpy.zeros(detectIdx.size-1)
    
#    #Calculate a vector of 0s and 1s from diffIdx where 1=1 and 0 = anything else
#    tmpVec2 = numpy.flatnonzero(diffIdx == 1)
#    tmpVec[tmpVec2] = 1
#    tmpVec3 = numpy.zeros((tmpVec.size+ThresholdCount-1,1))
#    print tmpVec3
#    print numpy.zeros((5-1,1))
#    print tmpVec.conj().transpose()
#    print numpy.zeros((ThresholdCount-idx,1))
        
#    for idx in range(1,ThresholdCount):
#        tmpVec3 = tmpVec3+[numpy.vstack((numpy.zeros((idx-1,1)),tmpVec.conj().T,numpy.zeros((ThresholdCount-idx,1))))]    
        
#    #tmpVec[tmpVec2] = numpy.ones(tmpVec2.size)

#    print tmpVec3
#    print tmpVec
    
#    print diffIdx.size
#    print tmpVec.size    
    

#    #print detectIdx

#    print BlankDist
#    print BinLength


#    print StartBin
#    print Threshold
#    print GlitchCount
#    #transBearing = msgData[27]+(msgData[28]*256)
#    transBearing = msgData[40]+(msgData[41]*256)
    
#    print transBearing
