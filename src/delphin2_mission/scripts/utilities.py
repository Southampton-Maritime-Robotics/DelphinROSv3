#=========================
# Utility
#-------------------------

from pylab import *

class uti:

    def waypointSwitching(self,target,eta,R_wp): ### FIXME: not functioning
        # identify if it is has to move onto the next waypoint
#        self.testPrint(path)
        flagSwitchWP = False
        dist = sqrt( (target[0]-eta[0])**2 + (target[1]-eta[1])**2 )
        if dist<=R_wp:
            flagSwitchWP = True
        return flagSwitchWP
        
    def findFirstSegment(self,path,eta):
        # identify the first line segment to follow
        _,pathLen = path.shape
        distMin = Inf # initialise the minimum distance
        wpTarget = 0 # 0 is the first wp in the list
        
        # find a nearest wp
        for i in range(1,pathLen): # loop from 1 to pathLen-1
            dist = sqrt( (path[0,i]-eta[0])**2 + (path[1,i]-eta[1])**2 )
            if dist<distMin:
                distMin = dist
                wpTarget = i
#                print 'condition is hold'
        
        # identify the segment to follow
        if wpTarget == pathLen:
            # if the last wp is the closest one, set it as a target
            pass
        else:
            t1,p_inter1 = self.interPointLine(path[:,wpTarget-1],path[:,wpTarget],eta)
            t2,p_inter2 = self.interPointLine(path[:,wpTarget],path[:,wpTarget+1],eta)
            if t1>=1:
                wpTarget += 1
            elif (t1 < 1 and t1 >= 0) and (t2 < 1 and t2 >= 0):
                d1 = sqrt( (p_inter1[0]-eta[0])**2 + (p_inter1[1]-eta[1])**2 )
                d2 = sqrt( (p_inter2[0]-eta[0])**2 + (p_inter2[1]-eta[1])**2 )
                if d2<=d1:
                    wpTarget += 1
        return wpTarget
        
    def interPointLine(self,wp1,wp2,p):
        # determine intersection point that minimise a cross-track error
        # source http://www.fundza.com/vectors/point2line/index.html
        # source http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
        vec_wp = array([ float(wp2[0]-wp1[0]), float(wp2[1]-wp1[1]) ])
        vec_point = array([ float(p[0]-wp1[0]), float(p[1]-wp1[1]) ])
        
        vec_wp_len = sqrt(vec_wp[0]**2+vec_wp[1]**2)
        
        t =  dot(vec_wp,vec_point) / (vec_wp_len**2)
        p_inter = wp1+t*vec_wp
        
        return t, p_inter
