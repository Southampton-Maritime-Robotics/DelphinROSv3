#!/usr/bin/env python

"""
A mission to get the AUV navigating along a predefined path at surface in Eastleigh lake.

A state waitForGPS is necessary to ensure that the AUV gets it position right before starting a path following state.

#Notes
-X is defined as +VE east, Y is defined as +VE north

"""

from __future__ import division
import rospy
import smach
import smach_ros
import time
import numpy

from delphin2_mission.library_highlevel     import library_highlevel
from delphin2_mission.utilities             import uti
from delphin2_mission.wp_TestwoodLake       import wp
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
def test_follow_path():
    # Create the top level state machine
    sm_se = smach.Sequence(outcomes=['succeeded','aborted','preempted'],
                        connector_outcome = 'succeeded')
                        
    # define a sequence of tasks
    with sm_se:
        smach.Sequence.add('GPS_FIX', 
            waitForGPS(_lib, timeout=50))
        
        # path_lawn_mowing, path_S_shaped, pathTest
        smach.Sequence.add('FOLLOW_PATH', 
            _smCon.LOS_path_following(
                    path=_wp.home,   # CHANGE PATH HERE
                    demandProp = 22,
                    timeout=3000))
            
####        smach.Sequence.add('GoHome',
####            _smCon.LOS_path_following(
####                    path=_wp.O, 
####                    demandProp = 22,
####                    timeout=600))
                        
    return sm_se
    
def construct_smach_top():
    # Create the top level state machine
    sm_top = smach.StateMachine(outcomes=['finish'])
    sm_top.userdata.wp = []
    
    # Open the container, add state and define state transition
    with sm_top:
        smach.StateMachine.add('INITIALISE', 
            Initialise(_lib,20),
            transitions={'succeeded':'SEQUENCE', 'aborted':'STOP','preempted':'STOP'})
            
        smach.StateMachine.add('SEQUENCE', 
            test_follow_path(),
            transitions={'succeeded':'STOP', 'aborted':'STOP','preempted':'STOP'})
            
        smach.StateMachine.add('STOP', 
            Stop(_lib), 
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
