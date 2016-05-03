#!/usr/bin/env python

"""
A mission to get the AUV back to point M.

#Notes
-X is defined as +VE east, Y is defined as +VE north
"""

import rospy
import smach
import smach_ros
import time
import numpy

from delphin2_mission.library_highlevel     import library_highlevel
from delphin2_mission.utilities             import uti
from delphin2_mission.wp_EastleightLake     import wp
from std_msgs.msg                           import String

## import simple states
from delphin2_mission.basic_states          import *
## import a comples state cconstructor
from delphin2_mission.construct_stateContainer import construct_stateContainer

################################################################################
### create functionality objects
_lib = library_highlevel()
_myUti = uti()
_smCon = construct_stateContainer(_lib, _myUti)
_wp = wp()

################################################################################
# state container generating section
def construct_smach_top():
    # Create the top level state machine
    sm_top = smach.StateMachine(outcomes=['finish'])
    sm_top.userdata.wp = []
    
    # Open the container, add state and define state transition
    with sm_top:
        smach.StateMachine.add('INITIALISE', Initialise(_lib,15),
            transitions={'succeeded':'GPS_FIX', 'aborted':'STOP','preempted':'STOP'})
        smach.StateMachine.add('GPS_FIX', waitForGPS(_lib, timeout=30),
            transitions={'succeeded':'GO_POINT_M', 'aborted':'STOP', 'preempted':'STOP'})
        smach.StateMachine.add('GO_POINT_M', _smCon.LOS_path_following(path=_wp.M, timeout=600),
            transitions={'succeeded':'STOP', 'aborted':'STOP', 'preempted':'STOP'})
        smach.StateMachine.add('STOP', Stop(_lib), 
            transitions={'succeeded':'finish'})

    return sm_top

################################################################################
# main code section
def main():
    # create state machine
    sm_top = construct_smach_top()

    # Create and start the introspection server - for visualisation
    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    sis.start()
    
    # Execute the state machine
    outcome = sm_top.execute()
    
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    rospy.init_node('smach_state_machine')
    _lib.wakeUp(5)
    main()
