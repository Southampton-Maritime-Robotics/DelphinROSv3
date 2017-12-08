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

import numpy
import rospy
import warnings

global driftOffset
driftOffset = rospy.get_param("/sonar/Driftoffset")
if driftOffset != 0:
    warnings.warn("Caution: A drift offset has been set")

class SonarConfiguration(object):
    def __init__(self):
        """
        Some of the sonar configurations need to be read from rosparam;
        this class mostly exists so the sonar configuration can be read for use in the analysis
        """

class SonarPing(object):
    """
    class for splitting the Tritech Micron Ping into the information contained in it
    using the original message published by the sonar node
    NOTE: the unprocessed information is published since it is more useful for debugging purposes
    TODO: Is it really???
    """
    def __init__(self, rawData):
        """
        Read one dataset from string as published by sonar
        :param rawData: 
        """
        global driftOffset
        self.bins =[]
        self.header = []
        data = numpy.fromstring(rawData.data, dtype=numpy.uint8)
        # only fill the dataset, if the packet contains bins
        if len(data) > 200:
            self.hasBins = 1
            # now split the most recent dataset:
            self.header = data[0:52]   # 13 byte header are read from sonar
            self.bins = data[52:]     # the rest is bins 
            self.NBins = float(self.header[42] + self.header[43]*256)
            self.transducerBearing = ((float(self.header[40]+(self.header[41]*256))/6400.0)*360 + driftOffset)%360
            self.LLim =  ((float(self.header[35]+(self.header[36]*256))/6400.0)*360)%360
            self.RLim =  ((float(self.header[37]+(self.header[38]*256))/6400.0)*360)%360
            self.ADInterval = float(self.header[33]+(self.header[34]*256))
            self.pingRange = self.ADInterval * 0.000000640 * self.NBins * 1500. /2
            self.pingPower=data[52:]
        else:
            self.hasBins = 0

        def read_return_information(self):
            """
            The mtHeadData contains information that is usually not used but that might be useful for debugging
            self.header contains 52 bytes, these are made of:
            self.header[0:13]: {1 to 13} relevant for serial communication
            self.header[13:]: (14 ->} Device Parameters/Reply Data
            
            Currently the following Device Para
            self.header[15]: Byte {16} Device Type, 11 is DST
            self.header[16]: Byte {17} 'Head Status'
            self.header[17]: Byte{18} Sweep Code
            self.header[18:20]: Byte{19, 20} 'HdCtrl' bytes
               the default values are added to the detailed analysis of the HdCtrl,
               they are in brackets if they may be changed without referring the docs for details
            self.header[26]: Byte{27} Gain setting
            :return: dict of all values that were extracted
            """
            result = {'Device Type': self.header[15],
                      'Gain setting': self.header[26] * 210
                      }
            byte = 2**8
            bitset = bin(self.header[16] + byte)  # add byte to make sure the bin value is full length
            result['HdPwrLoss'] = bitset[-1]
            result['MotErr'] = bitset[-2]

            sweep_code = {0: 'Scanning_Normal',
                          1: 'Scan at left limit',
                          2: 'Scan at right limit',
                          5: 'Scan at centre'}
            if self.header[17] in sweep_code:
                result['Sweep Code'] = sweep_code[self.header[17]]
            else:
                result['Sweep Code'] = ("Invalid Sweep code: " + str(self.header[17]))

            result['HdCtrl, MSB left'] = bin(self.header[18] + byte)[-8:] + bin(self.header[19] + byte)[-8:]
            hdctrl = {0: 'adc8on(=0)',
                      1: 'cont(=0)',
                      2: 'scanright(=0)',
                      3: 'invert=0',
                      4: 'motoff=0',
                      5: 'txoff=0',
                      6: 'spare=0',
                      7: 'chan2=0',
                      8: 'raw=1',
                      9: 'hasmot=1',
                      10:'applyoffset=0',
                      11: 'pingpong=0',
                      12: 'stareLLim=0',
                      13: 'ReplyASL=1',
                      14: 'ReplyThr=0',
                      15: 'IgnoreSensor=0'}
            for idx in hdctrl:
                result['HdCtrl: ' + hdctrl[idx]] = result['HdCtrl, MSB left'][-idx]

            return result





class SonarEvaluate(object):
    """
    Inspect a given sonar ping/set of sonar pings
    to get various types of further information
    
    """
    def __init__(self):
        self.RangeFudgeFactor = rospy.get_param("/sonar/analyse/RangeFudgeFactor")
        # The blanking distance is applied pre re-scaling, but estimated based on scaled data
        self.BlankDist = rospy.get_param("/sonar/analyse/BlankDist")/self.RangeFudgeFactor  
        self.UseSlidingThreshold = rospy.get_param("/sonar/analyse/UseSlidingThreshold")
        self.MaxThreshold = rospy.get_param("/sonar/analyse/Threshold")
        self.BaseThreshold = rospy.get_param("/sonar/analyse/BaseThreshold")
        self.SlideThreshold = rospy.get_param("/sonar/analyse/SlideThreshold")
        self.SoundspeedInWater = rospy.get_param("/sonar/SoundspeedWater")
        self.rotationDirection = rospy.get_param("/sonar/rotation/Direction")
        self.rotationOffset = rospy.get_param("/sonar/rotation/Offset")


    def get_thresholds(self, sonarPing):
        """
        Calculate array of threshold for the sonar return
        If the sliding thresholding is not used, apply a constant threshold instead.
        """
        thresholds = []
        length = len(sonarPing.pingPower)
        if not self.UseSlidingThreshold:
            thresholds = [self.MaxThreshold] * length
        else:
            reduction_per_bin = self.SlideThreshold * sonarPing.pingRange/sonarPing.NBins
            for i in range(length):
                next_threshold = max(self.BaseThreshold, (self.MaxThreshold - reduction_per_bin * i))
                thresholds.append(next_threshold)
        return thresholds

    def detect_obstacle(self, sonarPing, depth):
        depth = depth 
        """
        calculate distance at which obstacle is detected
        based on deprecated sonar_detectobstacle.py

        Parameters that exist but are currently not used:
        # GlitchCount = rospy.get_param("/GlitchCount")
        This might be interesting in connection with the capability of the sonar to repeat a ping,
        though I am not sure if the micron is one of the sonar that can do repeats, and of course this looses time
        """
        if sonarPing.hasBins:
            BinLength = sonarPing.ADInterval / float(self.SoundspeedInWater)/2.    # Make sure the division is by float
            StartBin = int(numpy.ceil(self.BlankDist/BinLength)) # Index of first bin after blanking distance
            thresholds = self.get_thresholds(sonarPing)



            # return indices of bins with value above threshold:
            ReturnIndexes = numpy.flatnonzero(numpy.greater(sonarPing.pingPower, thresholds))
            detections = numpy.greater(sonarPing.pingPower, thresholds)
            detections = numpy.multiply(detections, thresholds)

            target_range = -1
            # calculate target range in metres
            if ReturnIndexes != []:
                # remove indeces that are continuous
                ReturnIndexes = remove_continued(ReturnIndexes)
 
                idx = 0
                while ((idx < len(ReturnIndexes))
                       and (ReturnIndexes[idx] < StartBin)):
                    idx += 1
                if idx < len(ReturnIndexes):
                    target_range = ReturnIndexes[idx] * BinLength * self.RangeFudgeFactor

                    # check if this is just a reflection of the surface
                    if abs(target_range - depth) < 0.1:
                        idx += 1
                        print("depth ???")
                        if len(ReturnIndexes) > idx:
                            target_range = ReturnIndexes[idx] * BinLength * self.RangeFudgeFactor
                        else:
                            target_range = -1
                    
            # mean intensity of bins beyond the blanking disance
            # may be of interest in identifying areas of interest
            meanIntensity = numpy.mean(sonarPing.pingPower[StartBin:-1])

            # apply offsets so the bearing angle can be turned into rotation around a given axis
            # in the Delphin2 Coordinate system
            bearing_in_delphin2ks = (sonarPing.transducerBearing - self.rotationOffset) * self.rotationDirection
            results = [bearing_in_delphin2ks, target_range, meanIntensity]
        else:
            results = [0, 0, 0]
        return results, detections

def remove_continued(index_list):
    new_list = [index_list[0]]
    previous = index_list[0]
    for value in index_list:
        if value - previous > 3:
            new_list.append(value)
        previous = value
    return new_list

