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

        # NOTE: during high water levels in January this was a better home coordinate
        # self.home = numpy.array([[0., 5.],
        #                         [-5., -5.]]).T

        # go to start point of diving section
        self.to_start = numpy.array([[0., 0.],
                                     [5., 7.],
                                     [25., 100.]]).T

        self.to_start_A = numpy.array([[-12., 112.],
                                       [-17., 117.]]).T

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

        self.to_start_B = numpy.array([[0., 95.],
                                       [0., 100.]]).T

        self.to_start_Breverse = numpy.array([[0., 235.],
                                              [0., 230.]]).T

        self.path_B = numpy.array([[0., 0.],
                                   [5., 7.],
                                   [0., 100.],
                                   [0., 230.],
                                   [0., 235.],
                                   [0., 230.],
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


        self.path_over_step = numpy.array([#[100, 135],
                                           #[ 20, 100],
                                           ## [ 75, 170],
                                           # [-10, 130],
                                           # [ 50, 200],
                                           # [-50, 160],
                                           # [ 20, 220],
                                           [-70, 200],
                                           [-25, 250],
                                           [-75, 210],
                                           [-50, 260],
                                           [-50, 175],
                                           [  0, 280],
                                           [-25, 140],
                                           [ 30, 210],
                                           [  0, 100],
                                           [ 60, 180],
                                           [ 40,  80],
                                           [ 90, 150]]).T
