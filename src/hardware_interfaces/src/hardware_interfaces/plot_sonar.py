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


        #circular.pcolormesh(a_grid, b_grid, z_grid)
        #pylab.ion()
        #pylab.draw()
        #pylab.show()

    #circular.pcolormesh(dataset.angle, dataset.bins)

    """
    binData = []
    while not rospy.is_shutdown():
        for idx, a in enumerate(dataset.angle):
            binData.append(dataset.bins[idx])
        plotData = numpy.swapaxes(binData[len(binData)-testvariable:], 0, 1)
        circular.imshow(plotData, origin='lower', cmap=pylab.get_cmap('gist_heat'), vmin = 0, vmax = 200, interpolation = 'none')
        # zip(dataset.angle dataset.bins)
        pylab.ion()
        pylab.draw()
        pylab.show()
        pylab.clf()

    """
def scrolling_plots(dataset):
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    app = QtGui.QApplication(sys.argv)
    win = pg.GraphicsWindow(title="Basic plotting examples")
    win.setWindowTitle('pyqtgraph example: Plotting')


    # Enable antialiasing for prettier plots
    pg.setConfigOptions(antialias=True)

    p1 = win.addPlot(title="Basic array plotting", y=numpy.random.normal(size=100))

    p2 = win.addPlot(title="lets go sonar")
    y = dataset.angle
    x = range(0,len(dataset.angle))
    p2.plot(y)

    win.nextRow()
    p3 = win.addPlot(title = "bins")
    la = pg.image(dataset.bins)
    p3.plot(la)

    global curve, data, ptr, p6
    p6 = win.addPlot(title="Updating plot")
    curve = p6.plot(pen='y')
    data = numpy.random.normal(size=(10,1000))
    ptr = 0
    def update():
        global curve, data, ptr, p6
        curve.setData(data[ptr%10])
        if ptr == 0:
            p6.enableAutoRange('xy', False)  ## stop auto-scaling after the first data set is plotted
        ptr += 1
    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(50)

    win.show()
    print("%%%%%%%")
    QtGui.QApplication.instance().exec_()
    print("%%%%")
    #QtGui.closeAllWindows()



"""

                                    ## Start Qt event loop unless running in interactive mode or using pyside.
                                    if __name__ == '__main__':
                                        import sys
                                            if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
"""
