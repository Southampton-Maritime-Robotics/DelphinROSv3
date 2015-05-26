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

################################################################
class plotter:
    def __init__(self):
        self.data = [0]
        self.num_bins = 50
        #fig, az = okst.subplots()
        self.out_queue


        #self.ani = animation.FuncAnimation(self.data
    def update(self):
        n, bins, patches = plt.hist(figure.data, figure.num_bins)
        plt.xlabel('Smarts')
        plt.ylabel('Probability')
        plt.title(r'Histogram of IQ: $\mu=100$, $\sigma=15$')
        plt.plot(bins)
 
        plt.draw()
        print("%%%%%%%%%")
 

def get_sonar(msgData):
    # initialise figure

    global testvariable
    rawData= msgData.data
    msgData = numpy.fromstring(msgData.data, dtype=numpy.uint8)
    print(msgData)
    figure.data = msgData
    testvariable.append(msgData)

def draw_test():
    if not rospy.is_shutdown():

        figure.update()
        publishPID.publish(2)
        rospy.spin()

def some_function():
    global testvariable
    while not rospy.is_shutdown():
        plotValues = np.swapaxes(testvariable,0,1)
        imshow(plotValues, origin='lower', cmap=get_cmap('gist_heat'), vmin = 0, vmax = 200, interpolation = 'none')

        #plot(testvariable)
        ion()    # for some reason all of these three are necessary
        draw()
        show()

        

################################################################
################################################################
if __name__ == '__main__':

    # if script is main, assume its run manually for debugging
    rospy.init_node('plot_sonar', log_level=rospy.DEBUG)
    global testvariable
    testvariable = []
    rospy.Subscriber('sonar_output', String, get_sonar)
    #draw_test = rospy.Publisher('nanana', Float32, queue_size=10)
    time.sleep(2)
    some_function()
    #plotter.update(figure)
   
