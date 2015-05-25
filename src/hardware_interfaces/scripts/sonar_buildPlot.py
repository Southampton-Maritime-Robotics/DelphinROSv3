#!/usr/bin/python

'''
USED TO BUILD PLOT FROM SONAR DATA.
NOT IN USE.

Possibly, this node may not working!

'''
import rospy
import sys
import serial
import time
import numpy

from DelphinROSv2.msg import sonar_polarRange
from DelphinROSv2.msg import sonar_pingPlot

global pingStore_x
global pingStore_y

################################################################
def callback(msgData):    
    global pingStore_x
    global pingStore_y    

    transBearing = msgData.transBearing
    rangeToTarget = msgData.rangeToTarget
    heading = msgData.heading
    
    # convert to cartesian
    transBearing_inGlobal = (transBearing+heading)%360

    #print transBearing_inGlobal
    if transBearing_inGlobal > 15 and transBearing_inGlobal < 25:
	print '20degreesish =', transBearing_inGlobal
	print 'range at 20degreesish =', rangeToTarget
	print '-----------------'
    
    if transBearing_inGlobal > 75 and transBearing_inGlobal < 85:
	print '80degreesish =', transBearing_inGlobal
	print 'range at 80degreesish =', rangeToTarget
	print '-----------------'

    if transBearing_inGlobal > 135 and transBearing_inGlobal < 145:
	print '140degreesish =', transBearing_inGlobal
	print 'range at 140degreesish =', rangeToTarget
	print '-----------------'

    #calculate how many pings we are expecting
    LLim = rospy.get_param("/LLim") 
    RLim = rospy.get_param("/RLim")  
    step = rospy.get_param("/step")
    step = ((step /6400.0)*360)#)%360 # convert step to degrees
    
    if LLim > RLim:
        numberOfPings = (((360-LLim)+RLim)/step) #*2
    else:
        numberOfPings = (abs(RLim-LLim)/step) #*2

    transBearing_inGlobal = transBearing_inGlobal * numpy.pi / 180.0
    pingStore_x.append(rangeToTarget*numpy.cos(transBearing_inGlobal))
    pingStore_y.append(rangeToTarget*numpy.sin(transBearing_inGlobal))

    #if we have enough pings, publish to RANSAC
    if len(pingStore_x) >= numberOfPings:
        pub.publish(pingStore_x=pingStore_x, pingStore_y=pingStore_y)
        pingStore_x = []
        pingStore_y = []

################################################################
if __name__ == '__main__':
    global pub
    global pingStore_x
    global pingStore_y
    pingStore_x = []
    pingStore_y = []

    rospy.init_node('sonar_buildPlot')
    
    pub = rospy.Publisher('sonar_cartesianRanges', sonar_pingPlot)
    rospy.Subscriber('sonar_polarRange', sonar_polarRange, callback)
    rospy.spin()



