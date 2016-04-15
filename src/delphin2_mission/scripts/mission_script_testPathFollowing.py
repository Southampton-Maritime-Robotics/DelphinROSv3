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

### define a key waypoints and paths in according to the mission requirement
# waypoints
O = numpy.array([4.,0.]) # home: shifted from the origin a little to make sure it will not collide with the pier
A = numpy.array([-28.,-20.]) # reference point A
B = numpy.array([-1.,50.]) # reference point B
M = numpy.array([(A[0]+B[0])/2., (A[1]+B[1])/2.]) # mid-point between A and B
# reference paths
pathAtoB = numpy.vstack((A,B)).T
pathBtoA = numpy.vstack((B,A)).T
pathMtoO = numpy.vstack((M,O)).T
pathMtoA = numpy.vstack((M,A)).T
pathOtoM = numpy.vstack((O,M)).T
pathTest = numpy.vstack((A,M,O,B,M,O)).T

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
            transitions={'succeeded':'FOLLOW_PATH', 'aborted':'STOP', 'preempted':'STOP'})
        smach.StateMachine.add('FOLLOW_PATH', _smCon.LOS_path_following(path=pathTest, timeout=600),
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
