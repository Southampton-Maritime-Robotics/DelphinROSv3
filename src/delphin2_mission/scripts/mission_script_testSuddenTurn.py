#!/usr/bin/env python

"""
A mission to test the heading control performance in Eastleigh lake.
The AUV is programed to move forward at a constant propeller demand while maintaining a constant heading.
It is then suddently turn 90deg left (while executing a constant propeller demand) and hold the heading demand for a while to see how lond does the controller required to get the AUV back to the equilibrium again.

#Notes
-X is defined as east, Y is defined as north
"""

from __future__ import division
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
def suddenTurn(heading_init, demandProp):
    # creating a sequence state machine
    sm_se = smach.Sequence(outcomes=['succeeded','aborted','preempted'],
                        connector_outcome = 'succeeded')
    
    # define a sequence of tasks
    with sm_se:
        smach.Sequence.add('GoToHeading',
            GoToHeading(_lib, _myUti, 
                    demandHeading=heading_init, 
                    stable_time=20,
                    timeout=60)) # adjust stable_time not timeout
                        
        smach.Sequence.add('Accelerate',
            _smCon.track_heading_while_going_forward(
                    demandProp, 
                    demandHeading = heading_init, 
                    time_steady = -1, 
                    timeout = 20))
                        
        smach.Sequence.add('SuddenTurn',
            _smCon.track_heading_while_going_forward(
                    demandProp, 
                    demandHeading = heading_init-90, 
                    time_steady = -1, 
                    timeout = 70))
                        
####        smach.Sequence.add('GoHome',
####            _smCon.LOS_path_following(
####                    path=_wp.pathMtoO,
####                    demandProp=22,
####                    timeout=600))
    
    return sm_se
    
def construct_smach_top():
    # Create the top level state machine
    sm_top = smach.StateMachine(outcomes=['finish'])
    
    # Open the container, add state and define state transition
    with sm_top:
        smach.StateMachine.add('INITIALISE', 
            Initialise(_lib,15),
            transitions={'succeeded':'SEQUENCE', 'aborted':'STOP','preempted':'STOP'})
            
        smach.StateMachine.add('SEQUENCE', 
            suddenTurn(heading_init=300, demandProp=0),
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
