#!/usr/bin/python

"""
Publishes current altitude based on current depth
Assumption: Constant depth, as in the towing tanks
Constant depth is set via parameter

TODO: Support more complex 2D terrain lines
"""

import rospy
import time
from std_msgs.msg import String
from hardware_interfaces.msg import depth
from hardware_interfaces.msg import status
from hardware_interfaces.msg import altitude


class AltitudeEstimate:
    def __init__(self):
        self.poolDepth = rospy.get_param("/dummy/PoolDepth")
        self.altitude = self.poolDepth

    def altitude_update(self, depth):
        self.altitude = self.poolDepth - depth.depth


def altitude_loop():
    control_rate = rospy.get_param("/maxRate")
    control_period = 1./control_rate
    r = rospy.Rate(control_rate)
    
    # to control a timing for status publishing
    t0_status = time.time()
    dt_status = rospy.get_param('status_timing')

    while not rospy.is_shutdown():    
        time_ref = time.time()

        time_elapsed = time.time() - t0_status
        if time_elapsed > dt_status:
            t0_status = time.time()
            # Publish altimeter node status (ID is 3, see status.msg)
            pub_status.publish(nodeID=3, status=True)
            
        # Publish data
        pub_altitude.publish(altitude=estimate.altitude, altitude_filt=estimate.altitude, altitude_derr=0)
        
        if time.time() - time_ref < control_period:
            r.sleep()
        else:
            rate_warning = "altitude_transducer rate does not meet the desired value of %.2fHz: " \
                           "actual control rate is %.2fHz" % (control_rate, 1/time_elapsed)
            rospy.logwarn(rate_warning)
            pub_missionlog.publish(str)


################################################################
def shutdown():
    pub_status.publish(nodeID=5, status=False)

################################################################        
#     INITIALISE     ###########################################
################################################################

if __name__ == '__main__':
    time.sleep(1)  # Allow System to come Online
    rospy.init_node('depth_transducer')
    rospy.logwarn(">>>>>>>>>>> CAUTION DUMMY ALTITUDE - using altitude guess instead of actual altitude sensor")
    estimate = AltitudeEstimate()
    rospy.Subscriber('depth_out', depth, estimate.altitude_update)

    pub_altitude = rospy.Publisher('altitude_out', altitude, queue_size=10)
    pub_missionlog = rospy.Publisher('MissionStrings', String, queue_size=10)
    pub_status = rospy.Publisher('status', status, queue_size=10)
    
    rospy.on_shutdown(shutdown)
    
    time.sleep(0.3)
    
    altitude_loop()   # Main loop for publishing mission log and status
