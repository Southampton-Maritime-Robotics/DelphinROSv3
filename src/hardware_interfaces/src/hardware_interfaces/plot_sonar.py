"""
Library of useful plots of the sonar data
that can be run live based on data from sonar publisher
"""

import rospy
import numpy
import pylab

def time_plot(bin_array):
    """ 
    plot all sonar measurements against the time axis
    bins vs time
    intensity per bin represented by colour
    """
    while not rospy.is_shutdown():   # this loop is necessary for a regularly updating plot
        plotValues = numpy.swapaxes(bin_array, 0, 1)
        pylab.imshow(plotValues, origin='lower', cmap=pylab.get_cmap('gist_heat'), vmin = 0, vmax = 200, interpolation = 'none')
        pylab.ion()
        pylab.draw()
        pylab.show()
