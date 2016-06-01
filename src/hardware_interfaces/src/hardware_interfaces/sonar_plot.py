import numpy as np
import rospy
import time

from hardware_interfaces import sonar_analyse

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
        self.transducerBearing = [] # angle at which measurement was made, in [degrees]
        # TODO switch to [radians]
        self.pingRange = []
        self.targetRange = []
        self.fixedAngleTarget = []
        self.bins = []
        self.polarResolution = 1
        self.angleStep = 0.1

    def add_data(self, sonarPing):
        """
        add data
        """
        self.transducerBearing.append(sonarPing.transducerBearing)
        self.pingRange.append(sonarPing.pingRange)
        self.bins.append(sonarPing.bins)

        # if the range changes, the plot needs to be updated
        if len(self.bins[-1])  != self.polarResolution:
            self.polarResolution = len(self.bins[-1]) 
            self.polarImage = np.zeros((self.polarResolution * 2, self.polarResolution * 2))

        self.update_polar()
        self.update_angleOfInterest()


    def update_polar(self):
        """
        Update polar plot with most recent measurement
        """
        new_bins = self.bins[-1]
        
        # determine how much the sonar transducer moved since last measurement
        if len(self.bins)>1:
            self.angleStep = self.transducerBearing[-1] - self.transducerBearing[-2]  

        for idx, amplitude in enumerate(new_bins):
            # for each bin, get an list of pixels for plotting, 
            # increasing the entries in the list based on the distance from the centre
            angleCoverage = [
                self.transducerBearing[-1] - self.angleStep/2. + self.angleStep * n/np.sqrt(idx)
                for n in range(int(np.sqrt(idx)))
                ]
            for theta in angleCoverage:
                sine = np.sin(np.radians(theta))
                cosine = np.cos(np.radians(theta))

                x_coord = int(idx * cosine) + self.polarResolution  # polar plot around centre of plotting area
                y_coord = int(idx * sine) + self.polarResolution
                self.polarImage[x_coord, y_coord] = amplitude



    def update_angleOfInterest(self):
        """
        keep track of one specific angle,
        if a measurement at that angle was made, update plot
        TODO surely this can be done more flexible to different angles
        """
        if ((self.transducerBearing[-1] - abs(self.angleStep/2.) < self.angleOfInterest) and 
           (self.angleOfInterest < self.transducerBearing[-1] + abs(self.angleStep/2.))):
            self.fixedAngleTarget.append(self.targetRange[-1])
        else:
            if len(self.fixedAngleTarget) > 1:
                self.fixedAngleTarget.append(self.fixedAngleTarget[-1])
            else:
                self.fixedAngleTarget.append(-1)