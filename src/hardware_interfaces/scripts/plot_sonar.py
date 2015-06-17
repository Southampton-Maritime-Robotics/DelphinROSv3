#!/usr/bin/python
###########
#  VERSION AS OF 18/05/2015
# simple plotter for visualising sonar data live/from rosbag

###########

import roslib; roslib.load_manifest('hardware_interfaces') 
import rospy
import numpy
import time
import sys
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
import signal

from std_msgs.msg import String
from std_msgs.msg import Float32
from hardware_interfaces.msg import sonar_data
from hardware_interfaces.msg import sonar	

from hardware_interfaces import plot_sonar
from hardware_interfaces import analyse_sonar

################################################################

def get_sonar(msgData):
    # initialise figure
    sonar.add_message(msgData)

def draw_figures_pyplot():
    #plot_sonar.fixed_angle_time(sonar, 88, 50)
    #plot_sonar.round(sonar, 1)
 
    pylab.ion()
    fig1 = pylab.subplot(211)
    fig2 = pylab.subplot(212, polar = True)
    fig1.plot([1, 2, 3, 4])


    while not rospy.is_shutdown():
        #print("updating")
        plot_sonar.time_plot(sonar.bins[-60:], fig1, 60)
        #plot_sonar.round_bins(sonar, fig2)

        plot_sonar.round_bins(sonar, fig2)
        fig1.plot([1, 2, 2, 2])
        #print(fig2)
        #fig1 = fig2
        pylab.draw()
        pylab.show()

def draw_figures():
    """
    setup the Qt Window so it updates regularly and all settings are as wished
    """

    # make sure that ctrl+c quits:
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QtGui.QApplication(sys.argv)
    win = QtGui.QWidget()
    grid = QtGui.QGridLayout()    
    win.setLayout(grid)

    ## define all plots in the window
    w = 20
    # bins vs time
    bins_vs_time = pg.ImageView()
    grid.addWidget(bins_vs_time, 1, 1, w, w)

    bins = QtGui.QLabel('Bins')
    grid.addWidget(bins, 1, 0)

    time = QtGui.QLabel('Time')
    grid.addWidget(time, w + 1, w/2)

    title_bins_vs_time = QtGui.QLabel('Time vs Bin Intensity')
    grid.addWidget(title_bins_vs_time, 0, 1, 1, w)


    # polar bins
    polar_bins = pg.ImageView()
    grid.addWidget(polar_bins, 1, w + 1, w, w)
    grid.addWidget(QtGui.QLabel('Polar plot of recent bin intensities'), 0, w + 1, 1, w)



    # minimum altitude
    # TODO
    # distance of minimum altitude
    # TODO


    def update_figures():
        #TODO make the figure zoom/drag etc. work again
        bins_vs_time.setImage(numpy.array(sonar.bins[-150:]))
        polar_bins.setImage(sonar.polarImage)

    timer = QtCore.QTimer()
    timer.timeout.connect(update_figures)
    timer.start(50)
    win.show()
    QtGui.QApplication.instance().exec_()

################################################################
################################################################
if __name__ == '__main__':

    # if script is main, assume its run manually for debugging
    # TODO if this is only used when run manually
    # then where does the 'sonar' variable come from normally?

    rospy.init_node('plot_sonar', log_level=rospy.DEBUG)
    rospy.Subscriber('sonar_output', String, get_sonar)
    sonar = analyse_sonar.sonar()
    time.sleep(1)
    draw_figures()
    
    
    #plot_sonar.image_plot_test(sonar)
    #plot_sonar.scrolling_plots(sonar)
    #plot_sonar.multiple_images_plot_test(sonar)
    #plot_sonar.polar_plot_test(sonar)


  
