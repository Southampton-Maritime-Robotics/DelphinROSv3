"""
Library of useful plots of the sonar data
that can be run live based on data from sonar publisher
"""

import rospy
import numpy
import pylab
import sys
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import signal

import sys
from PyQt4.QtCore import QCoreApplication

def time_plot(bin_array, ax, memory):
    """ 
    plot all sonar measurements against the time axis
    bins vs time
    intensity per bin represented by colour
    """
    plotValues = numpy.swapaxes(bin_array[len(bin_array)-memory:], 0, 1)
    return ax.imshow(plotValues, origin='lower', cmap=pylab.get_cmap('gist_heat'), vmin = 0, vmax = 200, interpolation = 'none')


def fixed_angle_time(dataset, angle, memory):
    """
    plot the bins for only one angle
    bins vs time
    intensity per bin represnted by colour
    memory gives how many most recent data sets are displayed
    """
    binData = []
    while not rospy.is_shutdown():
        for idx, a in enumerate(dataset.angle):
            if a == angle:
                binData.append(dataset.bins[idx])
        plotData = numpy.swapaxes(binData[len(binData)-memory:], 0, 1)
        pylab.imshow(plotData, origin='lower', cmap=pylab.get_cmap('gist_heat'), vmin = 0, vmax = 200, interpolation = 'none')
        pylab.ion()
        pylab.draw()
        pylab.show()
        pylab.clf()

def round_bins(dataset, ax):
    """
    plot bins as in typical sonar view
    overwrite old data with new measurements
    """
    #bla = dataset.allData
    r_resolution = len(dataset.allData[0])
    swath = range(0, 90)

    theta_resolution = len(swath)
    #testbins = numpy.array(bla)
    values = numpy.zeros((theta_resolution, r_resolution))


    #circular = pylab.subplot(111, polar=True)
    #while not rospy.is_shutdown():
    for i in range(1, 8):
        try:
            new_theta = swath.index(dataset.angle[-i]/64)
            values[new_theta] = dataset.allData[-i]
        except:
            pass

    theta, r = numpy.mgrid[0:2*numpy.pi:numpy.complex(theta_resolution), 0:1:numpy.complex(r_resolution)]
    return ax.pcolormesh(theta, r, values, cmap=pylab.get_cmap('gist_heat'), vmin = 0, vmax = 200)




##########################################################################################
## EXPERIMENTAL STARTS HERE
##########################################################################################

def image_plot_test(dataset):
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    app = QtGui.QApplication(sys.argv)
    win =  QtGui.QMainWindow()

    # Enable antialiasing for prettier plots
    pg.setConfigOptions(antialias=True)

    imv = pg.ImageView()
    win.setCentralWidget(imv)
    imv.setImage(numpy.array(dataset.bins))
    win.show()
    #QtGui.QApplication.instance().exec_()

def multiple_images_plot_test(dataset):
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    app = QtGui.QApplication(sys.argv)
    #win =  QtGui.QMainWindow()
    win = QtGui.QWidget()
    #win = pg.GraphicsWindow()

    # Enable antialiasing for prettier plots
    pg.setConfigOptions(antialias=True)
    
    grid = QtGui.QGridLayout()
    win.setLayout(grid)
    imv = pg.ImageView()
    imv.setImage(numpy.array(dataset.bins))
    imv2 = pg.ImageView()
    imv2.setImage(numpy.array(dataset.bins))
    grid.addWidget(imv,0,0)
    b1 = QtGui.QPushButton("test")
    button = QtGui.QPushButton("test2")
    grid.addWidget(button, 0,3)
    grid.addWidget(imv2, 0,1)

    win.move(30,15)
    win.setWindowTitle("test")
    win.show()
    QtGui.QApplication.instance().exec_()

def scrolling_plots(dataset):
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    app = QtGui.QApplication(sys.argv)
    #win = pg.GraphicsWindow(title="Basic plotting examples")
    #win.setWindowTitle('pyqtgraph example: Plotting')
    win =  QtGui.QMainWindow()


    # Enable antialiasing for prettier plots
    pg.setConfigOptions(antialias=True)

    imv = pg.ImageView()
    win.setCentralWidget(imv)
    imv.setImage(numpy.array(dataset.bins))
    def update():
        print("naaaaa %%%%%%%%")
        imv.setImage(numpy.array(dataset.bins))
    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(50)

    win.show()
    QtGui.QApplication.instance().exec_()

def polar_plot_test(dataset):
    app = QtGui.QApplication(sys.argv)
    win =  QtGui.QMainWindow()
    imv = pg.ImageView()
    imv.setImage(dataset.polarImage)
    win.setCentralWidget(imv)
    win.show()
    QtGui.QApplication.instance().exec_()
