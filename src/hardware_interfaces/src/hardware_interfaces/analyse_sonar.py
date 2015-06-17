import numpy

"""
Useful classes, functions for analysing sonar data
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

