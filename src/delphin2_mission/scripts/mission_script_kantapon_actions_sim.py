#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time

from delphin2_mission.library_highlevel             import library_highlevel
from state_initialise             import Initialise
from state_stop                   import Stop
from state_goToDepth              import GoToDepth
from state_goToHeading            import GoToHeading
from state_trackFollow            import TrackFollow
from state_trackAltitude          import trackAltitude
from state_surface                import Surface
from state_camera                 import camera
from hardware_interfaces.msg      import compass
from std_msgs.msg import String
import matplotlib.pyplot as plt;

#### from kantapon's folder
import sys
import os.path
basepath = os.path.dirname(__file__)
filepath = os.path.abspath(os.path.join(basepath, 'kantapon'))
sys.path.append(filepath)
#from state_thruster_testing       import thruster_testing
from state_actions_sim                import actions

################################################################################
#Notes
#
#X is defined as east, Y is defined as north

################################################################################
#Modifications
#
#9/2/12 Modified state GoToXYZ

def main():

    rospy.init_node('smach_example_state_machine')
    # Define an instance of highlevelcontrollibrary to pass to all action servers
    lib = library_highlevel()

    # Create a SMACH state machine - with outcome 'finish'
    sm = smach.StateMachine(outcomes=['finish'])

    # Open the container
    with sm:
        # Add states to the container
        # generic state

################################################################################
########### MAIN CODE ##########################################################
################################################################################

#################################################################################
#        # [1/3] Initialise State (Must Be Run First!)
#        smach.StateMachine.add('INITIALISE', Initialise(lib,15), #15 = timeout for initialisation state
#            transitions={'succeeded':'GoToXYZ', 'aborted':'STOP','preempted':'STOP'})  
            
################################################################################
        # [2/3] Added States
        smach.StateMachine.add('ACTIONS', actions(lib), 
            transitions={'succeeded':'finish', 'aborted':'finish','preempted':'finish'})

################################################################################


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

