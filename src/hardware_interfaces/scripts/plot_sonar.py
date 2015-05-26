#!/usr/bin/python
###########
#  VERSION AS OF 18/05/2015
# simple plotter for visualising sonar data live/from rosbag

###########

import roslib; roslib.load_manifest('hardware_interfaces') 
import rospy
import numpy
import time
import matplotlib.pyplot as plt
from pylab import *
from std_msgs.msg import String
from std_msgs.msg import Float32
from hardware_interfaces.msg import sonar_data
from hardware_interfaces.msg import sonar	

from hardware_interfaces import plot_sonar

################################################################
class plotter:
    def __init__(self):
        self.data = []
        self.num_bins = 50
        #fig, az = okst.subplots()


        #self.ani = animation.FuncAnimation(self.data
    def update(self):
        n, bins, patches = plt.hist(figure.data, figure.num_bins)
        plt.xlabel('Smarts')
        plt.ylabel('Probability')
        plt.title(r'Histogram of IQ: $\mu=100$, $\sigma=15$')
        plt.plot(bins)
 
        plt.draw()
        print("%%%%%%%%%")

def time_plot(bin_array):
    while not rospy.is_shutdown():
        plotValues = np.swapaxes(bin_array,0,1)
        imshow(plotValues, origin='lower', cmap=get_cmap('gist_heat'), vmin = 0, vmax = 200, interpolation = 'none')

        #plot(testvariable)
        ion()    # for some reason all of these three are necessary
        draw()
        show()



def get_sonar(msgData):
    # initialise figure

    rawData= msgData.data
    msgData = numpy.fromstring(msgData.data, dtype=numpy.uint8)
    print(msgData)
    sonar.data.append(msgData)

def draw_test():
    if not rospy.is_shutdown():

        figure.update()
        publishPID.publish(2)
        rospy.spin()

        

################################################################
################################################################
if __name__ == '__main__':

    # if script is main, assume its run manually for debugging
    rospy.init_node('plot_sonar', log_level=rospy.DEBUG)
    rospy.Subscriber('sonar_output', String, get_sonar)
    sonar = plotter()
    time.sleep(2)  # This is needed for stable plotting
    plot_sonar.time_plot(sonar.data)
   
