#!/usr/bin/env python

"""
A general purpose mission script that lets the user quickly manages a sequence of tasks for the Delphin2 AUV in a section "MAIN CODE".

#Notes
-X is defined as east, Y is defined as north
"""


import rospy
import smach
import smach_ros
import time
import numpy
from pylab import *

from delphin2_mission.library_highlevel     import library_highlevel
from delphin2_mission.utilities             import uti
from state_initialise                       import Initialise
from state_stop                             import Stop
from state_goToDepth                        import GoToDepth
from state_goToHeading                      import GoToHeading
from state_goForwards                       import GoForwards
from state_goTurning                        import GoTurning
from state_pathFollowingLOS                 import pathFollowingLOS
from std_msgs.msg import String

from state_actions                          import actions

################################################################################

def construct_stateMachine():
    # Define an instance of highlevelcontrollibrary to pass to all action servers
    lib = library_highlevel()
    myUti = uti()
    lib.wakeUp(5)
    
    # Create the sub SMACH state machine
    sm_con = smach.Concurrence(outcomes=['succeeded','aborted','preempted'],
                               default_outcome='aborted',
                               outcome_map={'succeeded':
                                               {'GoToHeading':'succeeded'},
                                            'aborted':
                                               {'GoToHeading':'aborted'}})
    # Open the container
    with sm_con:
        # Add states to the container
        smach.Concurrence.add('GoTurning', GoTurning(lib, 0, 20, 200))
        smach.Concurrence.add('GoForwards', GoForwards(lib, 22, 200))
        smach.Concurrence.add('GoToHeading', GoToHeading(lib, myUti, 20.02, -1, 200))

    # creating the sub SMACH state machine
    se = smach.Sequence(outcomes=['succeeded','aborted','preempted'],
                        connector_outcome = 'succeeded')
    with se:
#        smach.Sequence.add('INITIALISE',Initialise(lib, 15)) #15 = timeout for initialisation state
#        smach.Sequence.add('GoTurning', GoTurning(lib, -1000, -12, 100))
        smach.Sequence.add('GoToHeading', GoToHeading(lib, myUti, None, -1, 10))
#        smach.Sequence.add('GoForwards', GoForwards(lib, 10, 20))
        smach.Sequence.add('GoToDepth', GoToDepth(lib, 0.5, 10, 15))


    # Create a SMACH state machine - with outcome 'finish'
    sm_top = smach.StateMachine(outcomes=['finish'])


    # Open the main container
    with sm_top:
            
        smach.StateMachine.add('CON', sm_con, transitions={'succeeded':'SEQUENCE',
                                                                'aborted':'CON',
                                                                'preempted':'STOP'})
        smach.StateMachine.add('SEQUENCE', se, transitions={'succeeded':'STOP',
                                                            'aborted':'STOP',
                                                            'preempted':'STOP'})

################################################################################

        # [3/3] Generic States (Come to this state just before terminating the mission)
        smach.StateMachine.add('STOP', Stop(lib), 
            transitions={'succeeded':'finish'})
    
    return sm_top

def main():

    rospy.init_node('smach_example_state_machine')

    # Allow the topic to come online
#    time.sleep(0)

    O = array([4.,0.]) # home: shifted from the origin a little to make sure it will not collide with the pier
    A = array([-28.,-20.]) # reference point A
    B = array([-1.,50.]) # reference point B
    
    M = array([(A[0]+B[0])/2., (A[1]+B[1])/2.]) # mid-point between A and B

    # Reference paths created from the reference points
    pathAtoB = numpy.vstack((A,B)).T
    pathBtoA = numpy.vstack((B,A)).T
    pathMtoO = numpy.vstack((M,O)).T
    pathMtoA = numpy.vstack((M,A)).T
    pathOtoM = numpy.vstack((O,M)).T
    pathTest = numpy.vstack((M,A,B,M)).T

    # create state machine
    sm = construct_stateMachine()

    # Create and start the introspection server - for visualisation
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    
    # Execute the state machine
    outcome = sm.execute()
    
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
