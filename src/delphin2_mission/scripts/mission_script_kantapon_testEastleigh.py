#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from pylab import *
import numpy

from library_highlevel            import library_highlevel
from state_initialise             import Initialise
from state_importWaypoints        import ImportWaypoints
from state_stop                   import Stop
from state_goToDepth              import GoToDepth
from state_goToHeading            import GoToHeading
from state_goToXYZ2               import GoToXYZ
from state_trackFollow            import TrackFollow
from state_trackAltitude          import trackAltitude
from state_surface                import Surface
from state_camera                 import camera
from state_terminalZ              import terminalZ
from state_N                      import N
from state_setTail                import setTail
from state_actionserver_goToDepth import GoToDepthServer
from hardware_interfaces.msg      import compass
from actionlib                    import *
from actionlib.msg                import *
from std_msgs.msg import String
import matplotlib.pyplot as plt;

#### from kantapon's folder
import sys
sys.path.append('/home/delphin2/DelphinROSv3/src/delphin2_mission/scripts/kantapon')
from state_testSurge                import testSurge
from utilities                      import uti
from state_pathFollowingLOS         import pathFollowingLOS

################################################################################
#Notes
#
# X is defined as east, Y is defined as north

def main():

################################################################################
########### DECLARE PARAMETERS #################################################
################################################################################
    
    rospy.init_node('smach_example_state_machine')
    # Define an instance of highlevelcontrollibrary to pass to all action servers
    lib = library_highlevel()
    myUti = uti()

    # points defined relative to the origin, i.e. north pier of Eastleigh lake TODO: correct these point
    A = array([-32.,4.]) # reference point A
    B = array([-2.,64.]) # reference point B
    M = array([(A[0]+B[0])/2., (A[1]+B[1])/2.]) # mid-point between A and B
    O = array([-2,2]) # home: shifted from the origin a little to make sure it will not collide with the pier
    pathAtoB = numpy.vstack((A,B)).T 
    pathBtoA = numpy.vstack((B,A)).T
    pathMtoO = numpy.vstack((M,O)).T
    
    # guidance
    L_los = 5 # [m] line-of-sight distance
    wp_R = 5 # [m] radius of acceptance
    
    # speed control
    uGain = -0.07 # gain to control speed variation: high abs(value) -> less overshoot -> more settling time
    uMax = 1 # [m/s] maximum surge speed

    # tolerant
    headingHold = 1 # TODO [sec] Heading must be hold for this many second to ensure the auv achieve equilibrium
    errHeadingTol = 5 # TODO [sec] tolarance in heading error
    
    time.sleep(10) # TODO: to be removed: tempolary used to allow the system to come online

################################################################################
########### STATE MACHINE ######################################################
################################################################################

    # Create a SMACH state machine - with outcome 'finish'
    sm = smach.StateMachine(outcomes=['finish'])

    # Open the container
    with sm:
        # Add states to the container
        # generic state

#################################################################################
#        # [1/3] Initialise State (Must Be Run First!)
#        smach.StateMachine.add('INITIALISE', Initialise(lib,15), #15 = timeout for initialisation state
#            transitions={'succeeded':'SURGE', 'aborted':'STOP','preempted':'STOP'})  
            
################################################################################
        # [2/3] Added States
        
#        smach.StateMachine.add('TRANSIT', pathFollowingLOS(lib,myUti,pathBtoA), 
#            transitions={'succeeded':'STOP', 'aborted':'STOP','preempted':'STOP'})
        
        # This state will guide the AUV back and forth between point A and B with different rudder RPM
        smach.StateMachine.add('SURGE', testSurge(lib,myUti,pathAtoB, uGain, uMax, headingHold, errHeadingTol,wp_R), 
            transitions={'succeeded':'STOP', 'aborted':'STOP','preempted':'STOP'})

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

