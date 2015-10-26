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
from state_goSideway                        import GoSideway
from state_goYaw                            import GoYaw
from state_pathFollowingLOS                 import pathFollowingLOS
from std_msgs.msg import String

from state_actions                          import actions

################################################################################
            
def main():

    rospy.init_node('smach_example_state_machine')
    
    # Define an instance of highlevelcontrollibrary to pass to all action servers
    lib = library_highlevel()
    myUti = uti()
    
    #Set Up Publisher for Mission Control Log
    pub = rospy.Publisher('MissionStrings', String)

    # Allow the topic to come online
    time.sleep(10)

    O = array([0.,0.]) # home: shifted from the origin a little to make sure it will not collide with the pier
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

################################################################################
########### STATE MACHINE ######################################################
################################################################################

    # Create a SMACH state machine - with outcome 'finish'
    sm = smach.StateMachine(outcomes=['finish'])

    # Open the container
    with sm:
        # Add states to the container
        # generic state

        # creating the sequence state machine
        se = smach.Sequence(outcomes=['succeeded','aborted','preempted'],
                            connector_outcome = 'succeeded')

        with se:
        # [1/3] Initialise State (Must Be Run First!)
            smach.Sequence.add('INITIALISE',Initialise(lib, 15)) #15 = timeout for initialisation state

        # [2/3] Added States
            # prop, th_ver[front, rear], th_hor[front, rear], cs_ver, cs_hor, actionHold
            smach.Sequence.add('ACTIONS_1',actions(lib, 0, [0, 0], [0, 0], 0, 0, 10, ))
#            smach.Sequence.add('GoHOME', pathFollowingLOS(lib, myUti, O))


        smach.StateMachine.add('SEQUENCE', se, transitions={'succeeded':'STOP',
                                                            'aborted':'STOP',
                                                            'preempted':'STOP'})

################################################################################

        # [3/3] Generic States (Come to this state just before terminating the mission)
        smach.StateMachine.add('STOP', Stop(lib), 
            transitions={'succeeded':'finish'})

################################################################################
########### EXECUTE STATE MACHINE AND STOP #####################################
################################################################################

    # Create and start the introspection server - for visualisation
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    
    # Execute the state machine
    outcome = sm.execute()
    print 'finished executing state machine'

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
