#!/usr/bin/python
###########
#  VERSION AS OF 18/06/2015
#  simple plotter for visualising sonar data live/from rosbag
#  current plots:
#  * live stream of all bins recorded (bins vs time)
#  * polar plot using the most recent bins available for each angle (polar bins)
#  

###########

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

from hardware_interfaces import sonar_analyse

################################################################

def get_sonar(msgData):
    """ add sonar data to sonar object as it comes in from ROS messages as *msgData*
    """
    sonar.add_message(msgData)


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

    # detected distance
    obstacle_detection = pg.PlotWidget()
    obstacle_detection.plot(sonar.targetRange[-150:])
    grid.addWidget(obstacle_detection, w+3, 1, w, w)
    grid.addWidget(QtGui.QLabel('Obstacle range for all angles'), w+2, 1, 1, w)



    # detect distance at fixed angle
    obstacle_angle = pg.PlotWidget()
    grid.addWidget(obstacle_angle, w+3, w+1, w, w)
    grid.addWidget(QtGui.QLabel('Obstacle range for one fixed angle'), w+2, w + 1, 1, w)



    # minimum altitude
    # TODO
    # distance of minimum altitude
    # TODO


    def update_figures():
        #TODO make the figure zoom/drag etc. work again
        bins_vs_time.setImage(numpy.array(sonar.bins[-150:]))
        polar_bins.setImage(sonar.polarImage)
        # update obstacle detection
        sonar.detect_obstacle(0)
        obstacle_detection.clear()
        obstacle_detection.plot(sonar.targetRange[-150:])
        obstacle_angle.clear()
        obstacle_angle.setYRange(0,15)
        obstacle_angle.plot(sonar.fixedAngleTarget[-150:])
 


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
    sonar = sonar_analyse.sonar()
    time.sleep(1)
    draw_figures()
    
    
  
