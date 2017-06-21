#!/usr/bin/env python

'''
Move along towing tank and return, whilst keeping a constant altitude

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
def construct_smach_sequence(demand_heading, demand_altitude, demand_prop, time_steady, start_timeout, end_timeout):
    """
    create sequence for alligning to correct heading at altitude,
    then moving along heading, and getting to a halt at altitude and heading
    TODO: why is the last sequence needed?

    demand_heading: goal heading for aligning and then tracking while moving forward
    demand_altitude: goal altitude for aligning and then tracking while moving forward
    demand_prop: propeller demand for moving forwards
    time_steady: the alignment at start and end should be stable for this long
    start_timeout: timeout for the initial positioning
    end_timeout: timeout for the final positioning
    """
    sm_se = smach.Sequence(outcomes=['succeeded','aborted','preempted'],
                        connector_outcome = 'succeeded')
    
    # define a sequence of tasks
    with sm_se:

        smach.Sequence.add('GoToAltitude_start', _smCon.track_altitude_while_keeping_heading_and_going_forward(
                                        demandProp=0,
                                        demandHeading=demand_heading,
                                        demandAltitude=demand_altitude,
                                        time_steady=time_steady,
                                        timeout=start_timeout))

        smach.Sequence.add('GoToAltitude_atSpeed', _smCon.track_altitude_while_keeping_heading_and_going_forward(
                                        demandProp=demand_prop,
                                        demandHeading=demand_heading,
                                        demandAltitude=demand_altitude,
                                        time_steady=-1,
                                        timeout=0))
                                        
        smach.Sequence.add('GoToAltitude_end', _smCon.track_altitude_while_keeping_heading_and_going_forward(
                                        demandProp=0,
                                        demandHeading=demand_heading,
                                        demandAltitude=demand_altitude,
                                        time_steady=time_steady,
                                        timeout=end_timeout))
    return sm_se
    
def construct_smach_top():
    # Create the top level state machine
    sm_top = smach.StateMachine(outcomes=['finish'])
    heading1 = rospy.get_param("/mission/Heading1")
    heading2 = rospy.get_param("/mission/Heading2")
    demand_altitude = rospy.get_param("/mission/Altitude")
    demand_prop = rospy.get_param("/mission/Prop")
    # Open the container, add state and define state transition
    with sm_top:
        smach.StateMachine.add('INITIALISE', Initialise(_lib,20),
            transitions={'succeeded':'DIVE_ALONG', 'aborted':'STOP','preempted':'STOP'})
        smach.StateMachine.add('DIVE_ALONG', construct_smach_sequence(heading1, demand_altitude, demand_prop, 
                                                                      time_steady=3, start_timeout=60, end_timeout=10),
            transitions={'succeeded':'DIVE_RETURN', 'aborted':'STOP','preempted':'STOP'})
        smach.StateMachine.add('DIVE_RETURN', construct_smach_sequence(heading2, demand_altitude, demand_prop,
                                                                      time_steady=3, start_timeout=10, end_timeout=10),
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
