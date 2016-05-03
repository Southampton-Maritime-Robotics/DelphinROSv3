#!/usr/bin/env python

"""
A mission script to test AUV heading --- mainly used for zero speed operation.

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

################################################################################
# state container generating section
def construct_smach_sequence():
    # creating a sequence state machine
    sm_se = smach.Sequence(outcomes=['succeeded','aborted','preempted'],
                        connector_outcome = 'succeeded')
    
    # define a sequence of tasks
    with sm_se:
#        smach.Sequence.add('GoForwards', GoForwards(_lib, demandProp = 22, timeout = 20))
#        smach.Sequence.add('GoToDepth_atSpeed_atHeading', _smCon.track_depth_while_keeping_heading_and_going_forward(demandProp = 22, demandHeading = 180, demandDepth = 2.5, time_steady = -1, timeout = 60))
        smach.Sequence.add('GoToHeading_atSpeed_1', _smCon.track_heading_while_going_forward(demandProp = 0, demandHeading = 45, time_steady = -1, timeout = 30))
        smach.Sequence.add('GoToHeading_atSpeed_2', _smCon.track_heading_while_going_forward(demandProp = 0, demandHeading = 180, time_steady = -1, timeout = 30))
        smach.Sequence.add('GoToHeading_atSpeed_3', _smCon.track_heading_while_going_forward(demandProp = 0, demandHeading = 45, time_steady = -1, timeout = 30))
    
    return sm_se
    
def construct_smach_top():
    # Create the top level state machine
    sm_top = smach.StateMachine(outcomes=['finish'])
    
    # Open the container, add state and define state transition
    with sm_top:
        smach.StateMachine.add('INITIALISE', Initialise(_lib,15),
            transitions={'succeeded':'SEQUENCE', 'aborted':'STOP','preempted':'STOP'})
        smach.StateMachine.add('SEQUENCE', construct_smach_sequence(),
            transitions={'succeeded':'STOP', 'aborted':'STOP','preempted':'STOP'})
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
