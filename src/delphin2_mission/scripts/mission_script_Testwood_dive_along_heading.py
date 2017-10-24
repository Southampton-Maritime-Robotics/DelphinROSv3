#!/usr/bin/env python

"""
2017 Testwood lake experiment
* get GPS position
* Move to start position of experiment 
* dive at constant altitude with given heading for given period
* resurface and obtain GPS position

#Notes
-X is defined as +VE east, Y is defined as +VE north

"""

from __future__ import division
import rospy
import smach
import smach_ros
import time
import numpy

from delphin2_mission.library_highlevel import library_highlevel
from delphin2_mission.utilities import uti
from delphin2_mission.wp_TestwoodLake import wp
from std_msgs.msg import String

## import simple states
import delphin2_mission.basic_states as bstate
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


def go_to_start():
    path_name = rospy.get_param("mission/PathName")
    path_to_altitude_tracking_start = getattr(wp, path_name)

    # state machine for reaching the start point at the surface
    sm_se = smach.Sequence(outcomes=['succeeded', 'aborted', 'preempted'],
                           connector_outcome='succeeded')
                        
    # define a sequence of tasks
    with sm_se:
        smach.Sequence.add('GPS_FIX', 
                           bstate.waitForGPS(_lib, timeout=50))
        
        smach.Sequence.add('FOLLOW_PATH',
                           _smCon.LOS_path_following(path=path_to_altitude_tracking_start,
                                                     demandProp=10.,
                                                     timeout=1380))
                        
    return sm_se


def move_at_altitude_sequence(demand_heading, demand_altitude, demand_prop, time_steady, start_timeout, track_timeout, end_timeout):
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
                                        timeout=track_timeout))
                                        
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
    sm_top.userdata.wp = []

    # get mission variables from parameter server
    dive_heading = rospy.get_param("/mission/DiveHeading")
    demand_altitude = rospy.get_param("/mission/Altitude")
    demand_prop = rospy.get_param("/mission/Prop")
    forwards_time = rospy.get_param("/mission/ForwardsTime")


 
    
    # Open the container, add state and define state transition
    with sm_top:
        smach.StateMachine.add('INITIALISE', 
                               bstate.Initialise(_lib, 15),
                               transitions={'succeeded': 'TO_START',
                                            'aborted': 'STOP',
                                            'preempted': 'STOP'})
            
        smach.StateMachine.add('TO_START',
                               go_to_start(),
                               transitions={'succeeded': 'AT_ALTITUDE',
                                            'aborted': 'STOP',
                                            'preempted': 'STOP'})

        smach.StateMachine.add('AT_ALTITUDE', 
                               move_at_altitude_sequence(dive_heading, demand_altitude, demand_prop,
                                                         time_steady=3, start_timeout=60,
                                                         track_timeout=forwards_time, end_timeout=10),
                               transitions={'succeeded': 'GET_GPS',
                                            'aborted': 'STOP',
                                            'preempted': 'STOP'})

        smach.StateMachine.add('GET_GPS',
                               bstate.waitForGps(_lib,
                                                 timeout=50),
                               transitions={'succeeded': 'STOP',
                                            'aborted': 'STOP',
                                            'preempted': 'STOP'
                                            })

        smach.StateMachine.add('STOP', 
                               bstate.Stop(_lib),
                               transitions={'succeeded': 'finish'})
            
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
