#!/usr/bin/env python

'''
A mission script to test the PID-based depth-pitch controller(see depthPitchPID.py) with an altitude demand
'''

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
def construct_smach_sequence(demandHeading, demandAltitude, demandProp):
    # creating a sequence state machine
    sm_se = smach.Sequence(outcomes=['succeeded','aborted','preempted'],
                        connector_outcome = 'succeeded')
    
    # define a sequence of tasks
    with sm_se:

        smach.Sequence.add('GoToAltitude_start', _smCon.track_altitude_while_keeping_heading_and_going_forward(
                                        demandProp=0,
                                        demandHeading=demandHeading,
                                        demandAltitude=demandAltitude,
                                        time_steady=1,
                                        timeout=20))

        smach.Sequence.add('GoToAltitude_atSpeed', _smCon.track_altitude_while_keeping_heading_and_going_forward(
                                        demandProp=demandProp,
                                        demandHeading=demandHeading,
                                        demandAltitude=demandAltitude,
                                        time_steady=-1,
                                        timeout=50))
                                        
        smach.Sequence.add('GoToAltitude_end', _smCon.track_altitude_while_keeping_heading_and_going_forward(
                                        demandProp=0,
                                        demandHeading=demandHeading,
                                        demandAltitude=demandAltitude,
                                        time_steady=-1,
                                        timeout=5))
    return sm_se
    
def construct_smach_top():
    # Create the top level state machine
    sm_top = smach.StateMachine(outcomes=['finish'])
    
    # Open the container, add state and define state transition
    with sm_top:
        smach.StateMachine.add('INITIALISE', Initialise(_lib,15),
            transitions={'succeeded':'SEQUENCE', 'aborted':'STOP','preempted':'STOP'})
        smach.StateMachine.add('SEQUENCE', construct_smach_sequence(demandHeading=270, demandAltitude=3.3, demandProp = 0),
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
