import numpy
import rospy
import time

"""
VERSION AS OF 18/06/2015

Useful classes, functions for analysing sonar data
sonar object contains the raw data as received from a ROS message,
splits it into relevant information

Current version is for Tritech Micron only
"""
# TODO split into sonarPing sonarPlotData and sonarEvaluate
"""
sonarPing: Information that is contained within one ping as the raw bins and then the information bits that are in it
sonarPlotData: collection onf sonar Pings and further things for plotting, e.g. polar plot data
sonarEvaluate: Further data that is needed for sonar evaluation, e.g. pitch and evaluation results
"""

class sonar:
    def __init__(self):
        self.raw = []           # raw string of data as read from ros publisher
        #self.timestamp = [1]     # timestamp associated with collected messages
        self.allData = []       # all data from collected messages, extracted as array
        self.bins = []          # bin part of messages
        self.header = []        # header part of messages
        self.angleStep = 4      # degrees per sonar angle step
        # TODO get the value of the angle step from sonar data
        self.angle = []         # angle at which measurement was made
        self.pingPower = []     # power of sonar for this bin
        self.polarImage = numpy.zeros((500, 500))

    def add_message(self, message):
        """
        fill in new information for a new message
        """
        self.raw.append(message.data)
        self.allData.append(numpy.fromstring(self.raw[-1], dtype=numpy.uint8))
        # now split the most recent dataset:
        self.header.append(self.allData[-1][0:52])   # 13 byte header are read from sonar
        self.bins.append(self.allData[-1][52:])      # the rest is bins
        angle= (self.header[-1][41]*255 + self.header[-1][40])/16
        self.angle.append(angle)       # this is where some of the angle seems to be
        self.pingPower.append(self.allData[-1][44:-1])
        self.update_polar()
        # TODO smaria:
        # better understand what is contained in the sonar data

    def update_polar(self):
        """
        update the polar plot with the most recent measurement
        """
        new_bins = self.bins[-1]
        for theta in numpy.arange(self.angle[-1] - self.angleStep/2, self.angle[-1] + self.angleStep/2, 0.1):
            sine = numpy.sin(theta*numpy.pi/180)
            cosine = numpy.cos(theta*numpy.pi/180)

            for idx, amplitude in enumerate(new_bins):
                idx +=25  # keep a black circle at the center'
                x_coord = int(idx * cosine) + 250
                y_coord = int(idx * sine) + 250
                self.polarImage[x_coord, y_coord] = amplitude

    def detect_obstacle(self, pitch):
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
        """
        # These parameters can not be read when using rosbags!
        Threshold = rospy.get_param("/Threshold")   # Value above which something is considered an object of interest
        BlankDist = rospy.get_param("/BlankDist")   # Distance from sonar head within which returns are ignored
        Range = rospy.get_param("/Range")           # Set range of sonar pin (in metres)
        NBins = rospy.get_param("/NBins")           # Number of bins: higher number == higher resolution, more noise
        """
        Threshold = 10
        BlankDist = 0.6
        Range = 20
        NBins = 200
        # TODO put these values in messages so rosbag will record them; e.g. by publishing the parameters the sonar used with the raw sonar data, if its not allready in there!

        BinLength = Range/float(NBins)              # Distance each bin covers
        StartBin = int(numpy.ceil(BlankDist/BinLength)) # Index of first bin after blanking distance

        # return indices of bins with value above threshold:
        ReturnIndexes = numpy.flatnonzero(self.pingPower[-1][StartBin:-1]>Threshold)
        # include number of bins in blanking distance for true distance
        ReturnIndexes = ReturnIndexes + StartBin  
        # Convert back from 16th of a gradian to degrees
        transBearing = ((float(self.allData[-1][40]+(self.allData[-1][41]*256))/6400.0)*360)
        transBearing = transBearing %360 

        # calculate target range in metres
        if ReturnIndexes != []: 
            # calculate distance to first bin that is above threshold
            TargetRange = ReturnIndexes[0] * BinLength  
        else: 
            TargetRange = -1 # indicate no returns
                                                
        # mean intensity of bins beyond the blanking disance
        # may be of interest in identifying areas of interest
        meanIntensity = numpy.mean(self.pingPower[-1][StartBin:-1]) 

        rospy.logdebug('bearing:',transBearing)
        rospy.logdebug('range:',TargetRange)
        rospy.logdebug(time.time())
        results = [transBearing, pitch, TargetRange, meanIntensity]
        return results

