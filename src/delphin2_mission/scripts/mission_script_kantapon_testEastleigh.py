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
import os.path
basepath = os.path.dirname(__file__)
filepath = os.path.abspath(os.path.join(basepath, 'kantapon'))
sys.path.append(filepath)
from utilities                      import uti
from state_pathFollowingLOS         import pathFollowingLOS
from state_testSurge                import testSurge
from state_testYaw                  import testYaw
from state_testSway                 import testSway
from state_testTurningCircle        import testTurningCircle
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
#    pathBtoA = numpy.vstack((B,A)).T
    pathMtoO = numpy.vstack((M,O)).T
    pathMtoA = numpy.vstack((M,A)).T
    pathOtoM = numpy.vstack((O,M)).T
    pathTest = array([[-15,100,20,120],
                      [-60,40,80,120]])
    
    # guidance
    L_los = 5 # [m] line-of-sight distance
    wp_R = 5 # [m] radius of acceptance
    
    # speed control
    uGain = 0.07 # gain to control speed variation: high value -> less overshoot -> more settling time
    uMax = 1 # [m/s] maximum surge speed

    # tolerant
    errHeadingTol = 2 # TODO [sec] tolarance in heading error
    
    # general
    timeDemandHold = 2 # 40 sec TODO actuator demand will be hold for this many second
    timeDelay = 1 # 5-20 sec TODO the vehicle will stop for this many second as to let its motion decay to near zero
    
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

        #=================================================
        ## PATH FOLLOWING TEST 
        # This state will get the AUV moving along the path
####        smach.StateMachine.add('PATH_FOLLOWING', pathFollowingLOS(lib,myUti, pathTest, L_los, uGain, uMax, wp_R), 
####            transitions={'succeeded':'HOME', 'aborted':'HOME','preempted':'HOME'})
        #-------------------------------------------------
        
        #=================================================
        ## SURGE TEST
        # This state will get the AUV transit to point A
####        smach.StateMachine.add('toWork', pathFollowingLOS(lib,myUti, pathMtoA, L_los, uGain, uMax, wp_R), 
####            transitions={'succeeded':'SURGE', 'aborted':'HOME','preempted':'HOME'})
####        # This state will guide the AUV back and forth between point A and B with different rudder RPM
####        smach.StateMachine.add('SURGE', testSurge(lib,myUti,pathAtoB, uGain, uMax, errHeadingTol, wp_R, timeDelay), 
####            transitions={'succeeded':'HOME', 'aborted':'HOME','preempted':'HOME'})
        #-------------------------------------------------
        
        #=================================================
        ## YAW TEST 
        # This state will get the AUV transit to point M
####        smach.StateMachine.add('toWork', pathFollowingLOS(lib,myUti, pathOtoM, L_los, uGain, uMax, wp_R), 
####            transitions={'succeeded':'YAW', 'aborted':'HOME','preempted':'HOME'})
####        # This state will keep the AUV at point M and perform yaw motion response test: TODO checi timeDemandHold
####        smach.StateMachine.add('YAW', testYaw(lib, myUti, M, uGain, uMax, wp_R, timeDemandHold, timeDelay), 
####            transitions={'succeeded':'HOME', 'aborted':'HOME','preempted':'HOME'})
        #-------------------------------------------------
        
        #=================================================
        ## SWAT TEST 
        # This state will get the AUV transit to point M
####        smach.StateMachine.add('toWork', pathFollowingLOS(lib,myUti, pathOtoM, L_los, uGain, uMax, wp_R), 
####            transitions={'succeeded':'SWAY', 'aborted':'HOME','preempted':'HOME'})
####        # This state will keep the AUV at point M and perform sway motion response test: TODO checi timeDemandHold
####        smach.StateMachine.add('SWAY', testSway(lib, myUti, M, uGain, uMax, errHeadingTol, wp_R, timeDemandHold, timeDelay), 
####            transitions={'succeeded':'HOME', 'aborted':'HOME','preempted':'HOME'})
        #-------------------------------------------------

        #=================================================
        ## TURNING CIRCLE TEST 
        # This state will get the AUV transit to point A
        smach.StateMachine.add('toWork', pathFollowingLOS(lib,myUti, pathMtoA, L_los, uGain, uMax, wp_R), 
            transitions={'succeeded':'TURNING', 'aborted':'HOME','preempted':'HOME'})
        # This state will keep the AUV at point M and perform sway motion response test: TODO checi timeDemandHold
        smach.StateMachine.add('TURNING', testTurningCircle(lib, myUti, pathAtoB, uGain, uMax, errHeadingTol, wp_R, timeDemandHold, timeDelay), 
            transitions={'succeeded':'HOME', 'aborted':'HOME','preempted':'HOME'})
        #-------------------------------------------------
        
        ## GENERIC STATE FOR EASTLEIGH LAKE
        # This state will get the AUV HOME
        smach.StateMachine.add('HOME', pathFollowingLOS(lib,myUti, pathMtoO, L_los, uGain, uMax, wp_R), 
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

