#=========================
# Waypoints needed for the test at Eastleight lake
#-------------------------

from __future__ import division
import numpy

class wp(object):
    def __init__(self):
        ### define a key waypoints and paths in according to the mission requirement
        # waypoints
        self.O = numpy.array([4.,0.]) # home: shifted from the origin a little to make sure it will not collide with the pier
        self.A = numpy.array([-28.,-20.]) # reference point A
        self.B = numpy.array([-1.,50.]) # reference point B
        self.M = numpy.array([(self.A[0]+self.B[0])/2., (self.A[1]+self.B[1])/2.]) # mid-point between A and B
        # reference paths
        self.pathAtoB = numpy.vstack((self.A,self.B)).T
        self.pathBtoA = numpy.vstack((self.B,self.A)).T
        self.pathMtoO = numpy.vstack((self.M,self.O)).T
        self.pathMtoA = numpy.vstack((self.M,self.A)).T
        self.pathOtoM = numpy.vstack((self.O,self.M)).T
        self.pathTest = numpy.vstack((self.M,self.B,self.A,self.B)).T

        self.path_S_shaped = numpy.array([[-3.77039131177951, 36.2047531320966],
                                         [-5.73836130583437, 44.0978496620225],
                                         [-12.9209041044603, 47.9168754113641],
                                         [-20.5650525464916, 45.1346329121524],
                                         [-23.6123771120926, 37.5922399417409],
                                         [-20.0463449387670, 30.2807904775761],
                                         [-11.3581490090425, 21.9662932676627],
                                         [-7.79211683571697, 14.6548438034979],
                                         [-10.8394414013180, 7.11245083308638],
                                         [-18.4835898433492, 4.33020833387466],
                                         [-25.6661326419751, 8.14923408321632],
                                         [-27.6341026360300, 16.0423306131422]]).T

        self.path_lawn_mowing = numpy.array([[4.5096,   37.8109],
                                             [-12.8109, 47.8109],
                                             [-17.8109, 39.1506],
                                             [-0.4904,  29.1506],
                                             [-5.4904,  20.4904],
                                             [-22.8109, 30.4904],
                                             [-26.3109, 24.4282],
                                             [-8.9904,  14.4282],
                                             [-12.4904, 8.3660],
                                             [-29.8109, 18.3660],
                                             [-31.8109, 14.9019],
                                             [-14.4904, 4.9019],
                                             [-16.4904, 1.4378],
                                             [-33.8109, 11.4378]]).T
