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
def ZigZag(demandProp, headingMean, headingAmp, demand_th_hor, demand_cs_ver, cycleMax, timeout):
    # Create the top level state machine
    sm_se = smach.Sequence(outcomes=['succeeded','aborted','preempted'],
                        connector_outcome = 'succeeded')
                        
    # define a sequence of tasks
    with sm_se:

####        smach.Sequence.add('GPS_FIX', 
####            waitForGPS(_lib, timeout=30))
####            
####        smach.Sequence.add('ToStart', 
####            _smCon.LOS_path_following(
####                    path=startLocation,
####                    demandProp=22,
####                    timeout=300))
####            
        smach.Sequence.add('AdjustHeading', 
            GoToHeading(_lib, _myUti, 
                    headingMean, 
                    stable_time=-1, 
                    timeout=20))
            
        smach.Sequence.add('Accelerate', 
            _smCon.track_heading_while_going_forward(
                    demandProp, 
                    demandHeading=headingMean, 
                    time_steady=-1, 
                    timeout=10))
            
        smach.Sequence.add('ZigZag', 
            _smCon.manoeuvrint_ZigZag(
                    demandProp, 
                    headingMean, 
                    headingAmp, 
                    demand_th_hor, 
                    demand_cs_ver, 
                    cycleMax, 
                    timeout))
            
####        smach.Sequence.add('GoHome',
####            _smCon.LOS_path_following(
####                    path=_wp.pathMtoO,
####                    demandProp=22,
####                    timeout=600))
                        
    return sm_se
    
def construct_smach_top():
    # Create the top level state machine
    sm_top = smach.StateMachine(outcomes=['finish'])
    sm_top.userdata.wp = []
    
    # Open the container, add state and define state transition
    with sm_top:
        smach.StateMachine.add('INITIALISE', 
            Initialise(_lib,15),
            transitions={'succeeded':'Test_ZigZagManoeuvre', 'aborted':'STOP','preempted':'STOP'})
        
        smach.StateMachine.add('Test_ZigZagManoeuvre', 
            ZigZag( demandProp=22, 
                    headingMean=225, 
                    headingAmp=30,
                    demand_th_hor=1800, 
                    demand_cs_ver=0, 
                    cycleMax=8, 
                    timeout=600),
            transitions={'succeeded':'STOP', 'aborted':'STOP', 'preempted':'STOP'})

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
