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
    # replacement for draw_figures_pyplot()
    # make sure that ctrl+c quits:
    signal.signal(signal.SIGINT, signal.SIG_DFL)


    def updateA():
        imv2.setImage(numpy.array(sonar.bins))

    timer = QtCore.QTimer()
    timer.timeout.connect(updateA)
    timer.start(50)
    win.show()
    QtGui.QApplication.instance().exec_()

################################################################
################################################################
if __name__ == '__main__':

    # if script is main, assume its run manually for debugging
    rospy.init_node('plot_sonar', log_level=rospy.DEBUG)
    rospy.Subscriber('sonar_output', String, get_sonar)
    sonar = analyse_sonar.sonar()
    time.sleep(1)
    #draw_figures()
    
    
    #plot_sonar.image_plot_test(sonar)
    #plot_sonar.scrolling_plots(sonar)
    #plot_sonar.multiple_images_plot_test(sonar)
    plot_sonar.polar_plot_test(sonar)


  
