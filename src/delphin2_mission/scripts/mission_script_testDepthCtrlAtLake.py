#!/usr/bin/env python

'''
A mission script to test the PID-based depth-pitch controller(see depthPitchPID.py).

'''

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
def construct_smach_sequence(demandHeading, demandProp, demandDepth):
    # creating a sequence state machine
    sm_se = smach.Sequence(outcomes=['succeeded','aborted','preempted'],
                        connector_outcome = 'succeeded')
    
    # define a sequence of tasks
    with sm_se:
        smach.Sequence.add('GoToHeading',
            GoToHeading(_lib, _myUti, 
                    demandHeading=demandHeading, 
                    stable_time=10,
                    timeout=60)) # adjust stable_time not timeout
                    
        smach.Sequence.add('GoToDepth', 
            _smCon.track_depth_while_keeping_heading_and_going_forward(
                    demandProp=0,
                    demandHeading=demandHeading,
                    demandDepth=demandDepth,
                    time_steady=-1,
                    timeout=40))
                    
        smach.Sequence.add('GoForwardAtDepth', 
            _smCon.track_depth_while_keeping_heading_and_going_forward(
                    demandProp,
                    demandHeading=demandHeading,
                    demandDepth=demandDepth,
                    time_steady=-1,
                    timeout=40))
                    
        smach.Sequence.add('MaintainAtDepth',
            _smCon.track_depth_while_keeping_heading_and_going_forward(
                    demandProp=0,
                    demandHeading=demandHeading,
                    demandDepth=demandDepth,
                    time_steady=-1,
                    timeout=40))
                    
        smach.Sequence.add('GPS_FIX', 
            waitForGPS(_lib, 
                    timeout=90))
                    
        smach.Sequence.add('GoHome',
            _smCon.LOS_path_following(
                    path=_wp.pathMtoO,
                    demandProp = 22,
                    timeout=600))
    
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
            construct_smach_sequence(demandHeading=220, demandProp=10, demandDepth=0.4),
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
