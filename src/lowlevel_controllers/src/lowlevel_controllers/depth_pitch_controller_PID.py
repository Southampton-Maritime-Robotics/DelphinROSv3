"""
Vertical plane functions for Depth Pitch PID
"""

import rospy
import math

class DepthPitchPID():
    def __init__(self):
        # read parameters, set initial values
        self.weightTH = rospy.get_param("vertical/PID/weight/thruster")
        self.weightCS = rospy.get_param("vertical/PID/weight/controlsurface")




    def determineActuatorWeight(self, _speed, _depth):
        """
        Calculate Thruster and control surface weight based on forwards speed.
        both values lie between 0 and 1
        thruster weight decreases with speed, control surface weight increases with speed
        """

        w_th = 1-0.5*(math.tanh((_speed-self.weightTH["U_star"])/self.weightTH["delta"])+1);

        w_cs =   0.5*(math.tanh((_speed-self.weightCS["U_star"])/self.weightCS["delta"])+1);
        
        return [w_th, w_cs]


