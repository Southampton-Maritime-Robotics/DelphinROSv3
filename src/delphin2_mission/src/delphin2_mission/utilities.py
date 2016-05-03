#=========================
# Utility
#-------------------------

from pylab import *
from math import *

class uti(object):
    def interPointLine(self,wp1,wp2,p):
        # determine intersection point that minimise a cross-track error
        # source http://www.fundza.com/vectors/point2line/index.html
        # source http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
        
        # bear in mind that the intersection point can appear outside the line segment.
        # In that case, it is recommended to track the begining or the ending point of the line segment instead.
        
        vec_wp = array([ float(wp2[0]-wp1[0]), float(wp2[1]-wp1[1]) ])
        vec_point = array([ float(p[0]-wp1[0]), float(p[1]-wp1[1]) ])
        
        vec_wp_len = sqrt(vec_wp[0]**2+vec_wp[1]**2)
        
        t =  dot(vec_wp,vec_point) / (vec_wp_len**2)
        p_inter = wp1+t*vec_wp

        return t, p_inter
        
    def rangeBearing(self, p1, p2):
        rang = sqrt( (p2[0]-p1[0])**2 + (p2[1]-p1[1])**2 )
        bear = atan2( (p2[0]-p1[0]), (p2[1]-p1[1]) )*180/pi
        bear = mod(bear,360)
            
        return rang, bear
    
    def computeHeadingError(self,demand,actual):
        demand = mod(demand,360)
        actual = mod(actual,360)
        errHeading = demand-actual
        if errHeading<-180:
            errHeading = errHeading%360
        elif errHeading>180:
            errHeading = -(-errHeading%360)
        return errHeading
        
    def limits(self,value, min, max):       #Function to contrain within defined limits
        if value < min:				   
           value = min
        elif value > max:
           value = max
        return value
        
    def surgeVelFromHeadingError(self,uMax,errHeading):
####        u = uMax*exp(-uGain*abs(errHeading))
        errTolerance = 10
        errThreshold = 50
        
        errHeading = abs(errHeading)
        if errHeading <= errTolerance:
            u = uMax
        elif errHeading <= errThreshold:
            u = uMax - float(uMax)*(errHeading-errTolerance)/(errThreshold-errTolerance)
        else:
            u = 0
                
        return u
