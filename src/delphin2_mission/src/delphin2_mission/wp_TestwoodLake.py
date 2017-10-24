#=========================
# Waypoints needed for the test at Eastleight lake
#-------------------------

from __future__ import division
import numpy

class wp(object):
    def __init__(self):
        ### define a key waypoints and paths in according to the mission requirement
        # waypoints
        self.O = numpy.array([0.,0.]) # home: shifted from the origin a little to make sure it will not collide with the pier
#        self.pathTest = numpy.array([[0., 0.],
#                                     [0., 20.],
#                                     [20., 20.],
#                                     [20., 0.],
#                                     [0., 0.]]).T

        # approach home going away from the pontoon to ensure vehicle safety
        self.home = numpy.array([[5., 7.],
                                 [-5., 3.]]).T

        # go to start point of diving section
        self.to_start = numpy.array([[0., 0.],
                                     [5., 7.],
                                     [25., 100.]]).T

        self.to_start_A = numpy.array([[0., 0.],
                                       [5., 7.],
                                       [-17., 117.],
                                       ]).T

        self.to_start_Areverse = numpy.array([[55., 205.],
                                              [50., 200.]]).T

        self.path_A = numpy.array([[0., 0.],
                                   [5., 7.],
                                   [-17., 117.],
                                   [50., 200.],
                                   [55., 205.],
                                   [50., 200.],
                                   [-17., 117.]
                                   ]).T

        self.to_start_B = numpy.array([[0., 0.],
                                       [5., 7.],
                                       [0., 100.],
                                       ]).T

        self.to_start_Breverse = numpy.array([[0., 275.],
                                              [0., 270.]]).T

        self.path_B = numpy.array([[0., 0.],
                                   [5., 7.],
                                   [0., 100.],
                                   [0., 270.],
                                   [0., 275.],
                                   [0., 270.],
                                   [0., 100.]
                                   ]).T

        self.test = numpy.array([[0., 0.],
                                 [25., 25.],
                                 #[0., 25.],
                                 [0., 0.]]).T

        self.longLawnMower = numpy.array([[0., 0.],
                                          #[0., 50.],
                                          #[-125., 300.],
                                          #[-100., 300.],
                                          #[50., 0.],
                                          #[75., 0.],
                                          ##[-50., 250.],
                                          #[-25., 250.],
                                          #[100., 0.],
                                          #[100., 50.],
                                          #[0., 250.],
                                          #[25., 250.],
                                          #[125., 50.],
                                          #[150., 50.],
                                          [75., 200.],
                                          [0., 0.]]).T
        self.horizontalLawnMower = numpy.array([[0., 0.],
                                                [0., 200.],
                                                [-125., 275.],
                                                [25., 275.],
                                                [25., 250.],
                                                [-125., 250.],
                                                [-125., 225.],
                                                [75., 225.],
                                                [75., 200.],
                                                [-75., 200.],
                                                [-50., 175.],
                                                [100., 175.],
                                                [125., 150.],
                                                [-50., 150.],
                                                [-50., 125.],
                                                [150., 125.],
                                                [150., 100.],
                                                [-25., 100.],
                                                [-25., 75.],
                                                [150., 75.],
                                                [150., 50.],
                                                [0., 50.],
                                                [0., 25.],
                                                [125., 25.],
                                                [110., 0.],
                                                [0., 0.]]).T

