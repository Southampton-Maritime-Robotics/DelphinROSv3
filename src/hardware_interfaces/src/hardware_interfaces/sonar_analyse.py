import numpy
import rospy
import time

"""
VERSION AS OF 18/06/2015

Useful classes, functions for analysing sonar data
sonar object contains the raw data as received from a ROS message,
splits it into relevant information

Current version is for Tritech Micron only

sonarPing: Information that is contained within one ping as the raw bins and then the information bits that are in it
sonarPlotData: collection onf sonar Pings and further things for plotting, e.g. polar plot data
sonarEvaluate: Further data that is needed for sonar evaluation, e.g. pitch and evaluation results
"""


class SonarPing(object):
    """
    class for splitting the Tritech Micron Ping into the information contained in it
    using the original message published by the sonar node
    NOTE: the unprocessed information is published since it is more useful for debugging purposes
    TODO: Is it really???
    """
    def __init__(self, rawData):
        self.bins =[] 
        self.header = []
        data = numpy.fromstring(rawData.data, dtype=numpy.uint8)
        # now split the most recent dataset:
        self.header = data[0:52]   # 13 byte header are read from sonar
        self.bins = data[52:]     # the rest is bins
        self.NBins = float(self.header[42] + self.header[43]*256)
        self.transducerBearing = ((float(self.header[40]+(self.header[41]*256))/6400.0)*360)%360
        self.LLim =  ((float(self.header[35]+(self.header[36]*256))/6400.0)*360)%360
        self.RLim =  ((float(self.header[37]+(self.header[38]*256))/6400.0)*360)%360
        self.ADInterval = float(self.header[33]+(self.header[34]*256))
        self.pingRange = self.ADInterval * 0.000000640 * self.NBins * 1500. /2
        self.pingPower = data[44:-1]


class SonarEvaluate(object):
    """
    Inspect a given sonar ping/set of sonar pings
    to get various types of further information
    
    """
    def __init__(self):
        self.BlankDist = rospy.get_param("/sonar/analyse/BlankDist")
        self.Threshold = rospy.get_param("/sonar/analyse/Threshold")




    def detect_obstacle(self, sonarPing):
        """
        calculate distance at which obstacle is detected
        based on deprecated sonar_detectobstacle.py

        Future ideas:
        # Variable blanking distance
        # Variable sonar return threshold

        Parameters that exist but are currently not used:
        # ADInterval = rospy.get_param("/ADInterval")
        # GlitchCount = rospy.get_param("/GlitchCount")
        # ThresholdCount = GlitchCount - 1

        """

        BinLength = sonarPing.pingRange/float(sonarPing.NBins)              # Distance each bin covers
        StartBin = int(numpy.ceil(self.BlankDist/BinLength)) # Index of first bin after blanking distance

        # return indices of bins with value above threshold:
        ReturnIndexes = numpy.flatnonzero(sonarPing.pingPower[StartBin:-1]>self.Threshold)
        # include number of bins in blanking distance for true distance
        ReturnIndexes = ReturnIndexes + StartBin  
        # Convert back from 16th of a gradian to degrees
        #transBearing =        transBearing = transBearing %360 

        # calculate target range in metres
        if ReturnIndexes != []: 
            # calculate distance to first bin that is above threshold
            TargetRange = ReturnIndexes[0] * BinLength  
        else: 
            TargetRange = -1 # indicate no returns
                                                
        # mean intensity of bins beyond the blanking disance
        # may be of interest in identifying areas of interest
        # meanIntensity = numpy.mean(self.pingPower[StartBin:-1]) 
        # TODO fix this
        meanIntensity = 0
        pitch = 0

        #rospy.logdebug('bearing:',transBearing)
        #rospy.logdebug('range:',TargetRange)
        #rospy.logdebug(time.time())
        #self.targetRange.append(TargetRange)
        #    self.fixedAngleTarget.append(TargetRange)
        #else:
        #    self.fixedAngleTarget.append(self.fixedAngleTarget[-1])
        results = [sonarPing.transducerBearing, pitch, TargetRange, meanIntensity]
        return results

