#!/usr/bin/env python

"""
A mission to get the AUV to perform a standard manoeuvring trial.
At a time being, only ZigZag manoeuvre is used.

#Notes
-X is defined as east, Y is defined as north
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
def ToStart_and_ZigZag(startLocation, demandProp, headingMean, headingAmp, demand_th_hor, demand_cs_ver, cycleMax, timeout):
    # Create the top level state machine
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    # Open the container, add state and define state transition
    with sm:
        smach.StateMachine.add('ToStart', _smCon.LOS_path_following(path=startLocation, timeout=300),
            transitions={'succeeded':'AdjustHeading', 'aborted':'aborted', 'preempted':'preempted'})
        smach.StateMachine.add('AdjustHeading', GoToHeading(_lib, _myUti, headingMean, stable_time=10, timeout=60),
            transitions={'succeeded':'Accelerate', 'aborted':'aborted', 'preempted':'preempted'})
        smach.StateMachine.add('Accelerate', _smCon.track_heading_while_going_forward(demandProp, demandHeading=headingMean, time_steady=-1, timeout=10),
            transitions={'succeeded':'ZigZag', 'aborted':'aborted', 'preempted':'preempted'})
        smach.StateMachine.add('ZigZag', _smCon.manoeuvrint_ZigZag(demandProp, headingMean, headingAmp, demand_th_hor, demand_cs_ver, cycleMax, timeout), 
            transitions={'succeeded':'succeeded', 'aborted':'aborted', 'preempted':'preempted'})
    return sm
    
    track_heading_while_going_forward(self, demandProp, demandHeading, time_steady, timeout)
    
def construct_smach_top():
    # Create the top level state machine
    sm_top = smach.StateMachine(outcomes=['finish'])
    sm_top.userdata.wp = []
    
    # Open the container, add state and define state transition
    with sm_top:
        smach.StateMachine.add('INITIALISE', Initialise(_lib,15),
            transitions={'succeeded':'GPS_FIX', 'aborted':'STOP','preempted':'STOP'})
        smach.StateMachine.add('GPS_FIX', waitForGPS(_lib, timeout=30),
            transitions={'succeeded':'Test_ZigZagManoeuvre', 'aborted':'STOP', 'preempted':'STOP'})
        smach.StateMachine.add('Test_ZigZagManoeuvre', ToStart_and_ZigZag(startLocation=_wp.B, demandProp=22, headingMean=190, headingAmp=30, demand_th_hor=0, demand_cs_ver=30, cycleMax=10, timeout=600),
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
