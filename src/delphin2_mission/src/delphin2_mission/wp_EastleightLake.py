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
        self.pathTest = numpy.vstack((self.A,self.M,self.O,self.B,self.M,self.O)).T

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

        self.path_lawn_mowing = numpy.array([[-2.03390953511021, 46.0528306622187],
                                            [-21.7300645953544, 49.5257942155573],
                                            [-23.4665463720237, 39.6777166854352],
                                            [-3.77039131177951, 36.2047531320966],
                                            [-5.50687308844882, 26.3566756019745],
                                            [-25.2030281486930, 29.8296391553131],
                                            [-26.5922135700284, 21.9511771312155],
                                            [-6.89605850978426, 18.4782135778768],
                                            [-8.28524393111971, 10.5997515537792],
                                            [-27.9813989913639, 14.0727151071178],
                                            [-29.0232880573655, 8.16386858904454],
                                            [-9.32713299712129, 4.69090503570593],
                                            [-10.3690220631229, -1.21794148236732],
                                            [-30.0651771233670, 2.25502207097129],
                                            [-30.7597698340348, -1.68420894107754],
                                            [-11.0636147737906, -5.15717249441615],
                                            [-11.7582074844583, -9.09640350646498],
                                            [-31.4543625447025, -5.62343995312638],
                                            [-31.8016589000363, -7.59305545915079],
                                            [-12.1055038397922, -11.0660190124894],
                                            [-12.4528001951260, -13.0356345185138],
                                            [-32.1489552553702, -9.56267096517521]]).T
