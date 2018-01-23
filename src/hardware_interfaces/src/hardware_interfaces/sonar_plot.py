import numpy as np
import rospy
import time
import collections


"""
VERSION AS OF 2016-04-26

"""

class SonarPlot():
    """
    Class for plotting sonar data from rosbags
    (or live, once ROS wireless nodes have been implemente on delphin2?)
    uses objects from the SonarPing class for getting the sonar data
    """

    def __init__(self):
        self.memory = rospy.get_param("/sonar/plot/Memory")
        self.angleOfInterest = rospy.get_param("sonar/plot/AngleOfInterest")
        self.transducerBearing = collections.deque(maxlen=self.memory)# angle at which measurement was made, in [degrees]
        self.pingRange = collections.deque([0], maxlen = self.memory)
        self.targetRange = collections.deque([0]*self.memory, self.memory)  # initialise for nicer plots at startup

        self.bins = collections.deque(maxlen=self.memory)
        self.polarResolution = 1
        self.angleStep = 0.1
        self.Detections = collections.deque(maxlen = self.memory)


    def add_data(self, sonarPing):
        """
        add data, if any data was containted in the current return message form sonar
        """
        if sonarPing.hasBins:
            self.transducerBearing.append(sonarPing.transducerBearing)
            self.pingRange.append(sonarPing.pingRange)
            self.bins.append(sonarPing.bins)

            # if the range changes, the plot needs to be updated
            if len(self.bins[-1])  != self.polarResolution:
                # clear all but the current bins
                self.bins = collections.deque([self.bins[-1]], maxlen=self.memory)
                self.polarResolution = len(self.bins[-1])
                self.polarImage = np.zeros([self.polarResolution * 2, self.polarResolution * 2, 3], dtype=np.uint8)
                # add grid for 0, 90, 180 and 270 degrees
                for x in range(2, self.polarResolution * 2 - 2):
                    self.polarImage[x, self.polarResolution] = [200, 0, 0]
                    self.polarImage[x, self.polarResolution + 1] = [200, 0, 0]
                    self.polarImage[x, self.polarResolution - 1] = [200, 0, 0]
                    self.polarImage[self.polarResolution, x] = [200, 0, 0]
                    self.polarImage[self.polarResolution + 1, x] = [200, 0, 0]
                    self.polarImage[self.polarResolution - 1, x] = [200, 0, 0]

            self.update_polar()  

    def add_ping_to_polar(self, back_count, color_weight):
        """
        add one ping to polar plot
        use the given color mix of r, g, b
        color values between [0, 0, 0] and [1, 1, 1] give weight to return amplitude
        """

        new_bins = self.bins[-back_count]

        # determine how much the sonar transducer moved since last measurement
        if len(self.bins) > 1:
            self.angleStep = abs(self.transducerBearing[-back_count] - self.transducerBearing[-back_count - 1])
            # check for transition over 360 degree
            if self.angleStep > 300:
                self.angleStep = 360 - self.angleStep

        for idx, amplitude in enumerate(new_bins):
            # for each bin, get an list of pixels for plotting, 
            # increasing the entries in the list based on the distance from the centre
            if (len(self.Detections) >= 1) and (self.Detections[-1][idx] > 1.) : 
                plot_color = [1, 0, 0]
            else:
                plot_color = color_weight
            #plot_color = color_weight
            angleCoverage = [
                #self.transducerBearing[-1] - self.angleStep/2. + self.angleStep * n/np.sqrt(idx)
                (self.transducerBearing[-back_count] + self.angleStep/2. + self.angleStep * n/np.sqrt(idx))%360
                for n in range(int(np.sqrt(idx)))
                ]

            for theta in angleCoverage:
                sine = np.sin(np.radians(theta))
                cosine = np.cos(np.radians(theta))

                # polar plot around centre of plotting are, making sure the plotting area is not exceeded
                # (rounding errors sometimes lead to
                x_coord = int(idx * cosine) + self.polarResolution  # polar plot around centre of plotting area
                y_coord = - int(idx * sine) + self.polarResolution
                gridlines = [self.polarResolution, self.polarResolution + 1, self.polarResolution -1]
                if (x_coord in gridlines) or (y_coord in gridlines):
                    self.polarImage[x_coord, y_coord] = [200, 0, 0]
                else:
                    self.polarImage[x_coord, y_coord] = np.multiply(amplitude, plot_color)




    def update_polar(self):
        """
        Update polar plot with most recent measurement
        """
        # whilst more updates would be nicer, they slow down the plotting
        # at least with the current implementation
        if len(self.bins)>2:
            self.add_ping_to_polar(2, [1., 1., 1.])
            self.add_ping_to_polar(1, [0., 1., 1.])


#    def update_last_pings(self):
#        """
#        show a plot of all returns of the last n angles
#        HERE
#        keep track of one specific angle,
#        if a measurement at that angle was made, update plot
#        TODO surely this can be done more flexible to different angles
#        """
#        self.lastPing1 = [1, 2, 3, 3, 4, 5, 8, 10, 2]
