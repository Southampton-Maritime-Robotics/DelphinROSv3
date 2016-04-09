#!/usr/bin/env python

'''
A mission script to test if all actuators are working correctly. This must be done before a mission.

TODO:
- camera test may be added

'''

import rospy
import smach
import smach_ros
import time

from delphin2_mission.library_highlevel import library_highlevel
from state_initialise                   import Initialise
from state_stop                         import Stop
from state_verify_lights                import verify_lights
from state_verify_thrusters             import verify_thrusters
from state_verify_fins                  import verify_fins
from state_verify_prop                  import verify_prop

from std_msgs.msg import String
################################################################################
### create functionality objects
_lib = library_highlevel()

################################################################################
# state container generating section
def construct_smach_sequence():
    # creating a sequence state machine
    sm_se = smach.Sequence(outcomes=['succeeded','aborted','preempted'],
                        connector_outcome = 'succeeded')
    
    # define a sequence of tasks
    with sm_se:
        smach.Sequence.add('VERIFY_LIGHTS', verify_lights(_lib))
        smach.Sequence.add('VERIFY_THRUSTERS', verify_thrusters(_lib))
        smach.Sequence.add('VERIFY_FINS', verify_fins(_lib))
        smach.Sequence.add('VERIFY_PROP', verify_prop(_lib))
        
    return sm_se

def construct_smach_top():
    # Create the top level state machine
    sm_top = smach.StateMachine(outcomes=['finish'])
    
    # Open the container, add state and define state transition
    with sm_top:
        smach.StateMachine.add('INITIALISE', Initialise(_lib,15),
            transitions={'succeeded':'DRY_LAND_TEST', 'aborted':'STOP','preempted':'STOP'})
        smach.StateMachine.add('DRY_LAND_TEST', construct_smach_sequence(),
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
