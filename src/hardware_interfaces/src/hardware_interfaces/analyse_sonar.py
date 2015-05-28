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
        self.angle = []         # angle at which measurement was made

    def add_message(self, message):
        """
        fill in new information for a new message
        """
        self.raw.append(message.data)
        self.allData.append(numpy.fromstring(self.raw[-1], dtype=numpy.uint8))
        # now split the most recent dataset:
        self.header.append(self.allData[-1][0:52])   # 13 byte header are read from sonar
        self.bins.append(self.allData[-1][52:])      # the rest is bins
        angle= self.header[-1][41]*255 + self.header[-1][40]
        self.angle.append(angle)       # this is where some of the angle seems to be
        # TODO smaria:
        # better understand what is contained in the sonar data


