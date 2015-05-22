#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from pylab import *
import numpy

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

from delphin2_mission.utilities     import uti
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

    # points defined relative to the origin O
    # North pier of the Eastleigh lake (50.957024,-1.366769)
    O = array([-2,2]) # home: shifted from the origin a little to make sure it will not collide with the pier
    A = array([-28.,12.]) # reference point A
    B = array([-1.,64.]) # reference point B
#    # South pier of the Eastleigh lake (50.956473,-1.366835)
#    O = array([-2,2]) # home: shifted from the origin a little to make sure it will not collide with the pier
#    A = array([-20.,14.]) # reference point A
#    B = array([-30.,64.]) # reference point B
    
    M = array([(A[0]+B[0])/2., (A[1]+B[1])/2.]) # mid-point between A and B

    pathAtoB = numpy.vstack((A,B)).T
    pathBtoA = numpy.vstack((B,A)).T
    pathMtoO = numpy.vstack((M,O)).T
    pathMtoA = numpy.vstack((M,A)).T
    pathOtoM = numpy.vstack((O,M)).T
    pathTest = numpy.vstack((O,M,A,B,M)).T
#    pathTest = array([[-15,100,20,120],
#                      [-60,40,80,120]])

    # determine range and heading to the first waypoint in the list # TODO: comment this block
    # help to ensure that the AUV got hte location right
    while not rospy.is_shutdown():
        X = lib.getX()
        Y = lib.getY()
        Px = pathAtoB[0][0]
        Py = pathAtoB[1][0]
        rang, bear = myUti.rangeBearing([X,Y], [Px, Py])
        print "range to the point is ", rang, "deg"
        print "bearing to the point is ", bear, "metre"
        
    # guidance
    L_los = 5 # [m] line-of-sight distance
    wp_R = 5 # [m] radius of acceptance
    
    # speed control
    uGain = 0.07 # gain to control speed variation: high value -> less overshoot -> more settling time
    uMax = 1 # [m/s] maximum surge speed

    # tolerant
    errHeadingTol = 2 # TODO [deg] tolarance in heading error
    
    # general
    timeDemandHold = 2 # 60 sec TODO actuator demand will be hold for this many second
    turningAngle = 720 # [deg] TODO the vehicle has to turn this many degree before move to the next step
    timeDelay = 1 # 20 sec TODO the vehicle will stop for this many second as to let its motion decay to near zero
    
    time.sleep(10) # TODO: to be removed: tempolary used to allow the system to come online
    
    # for testing at depth # TODO:
    depthDemand = 0. # [m].
    depthTol = 0.2 # [m]. It is account as the AUV get to the depth if the depth error is less than this.
    depthDemandMin = 0.5 # [m] if the depthDemand is less than this, it is accounted as no depth demand specified.

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
        # This state will get the AUV moving along the pathTest [O->M->A->B->M->O]. note that [M->O] is in the state 'HOME'
        smach.StateMachine.add('PATH_FOLLOWING', pathFollowingLOS(lib,myUti, pathTest, L_los, uGain, uMax, wp_R), 
            transitions={'succeeded':'HOME', 'aborted':'HOME','preempted':'HOME'})
        #-------------------------------------------------
        
        #=================================================
        ## SURGE TEST
        # This state will get the AUV transit to point A
####        smach.StateMachine.add('toWork', pathFollowingLOS(lib,myUti, pathMtoA, L_los, uGain, uMax, wp_R), 
####            transitions={'succeeded':'SURGE', 'aborted':'HOME','preempted':'HOME'})
####        # This state will guide the AUV back and forth between point A and B with different rudder RPM
####        smach.StateMachine.add('SURGE', testSurge(lib,myUti,pathAtoB, uGain, uMax, errHeadingTol, wp_R, timeDelay, depthDemand, depthTol, depthDemandMin), 
####            transitions={'succeeded':'HOME', 'aborted':'HOME','preempted':'HOME'})
        #-------------------------------------------------
        
        #=================================================
        ## YAW TEST 
        # This state will get the AUV transit to point M
####        smach.StateMachine.add('toWork', pathFollowingLOS(lib,myUti, pathOtoM, L_los, uGain, uMax, wp_R), 
####            transitions={'succeeded':'YAW', 'aborted':'HOME','preempted':'HOME'})
####        # This state will keep the AUV at point M and perform yaw motion response test: TODO check timeDemandHold
####        smach.StateMachine.add('YAW', testYaw(lib, myUti, M, uGain, uMax, wp_R, timeDemandHold, timeDelay, depthDemand, depthTol, depthDemandMin), 
####            transitions={'succeeded':'HOME', 'aborted':'HOME','preempted':'HOME'})
        #-------------------------------------------------
        
        #=================================================
        ## SWAT TEST 
        # This state will get the AUV transit to point M
####        smach.StateMachine.add('toWork', pathFollowingLOS(lib,myUti, pathOtoM, L_los, uGain, uMax, wp_R), 
####            transitions={'succeeded':'SWAY', 'aborted':'HOME','preempted':'HOME'})
####        # This state will keep the AUV at point M and perform sway motion response test: TODO check timeDemandHold
####        smach.StateMachine.add('SWAY', testSway(lib, myUti, M, uGain, uMax, errHeadingTol, wp_R, timeDemandHold, timeDelay, depthDemand, depthTol, depthDemandMin), 
####            transitions={'succeeded':'HOME', 'aborted':'HOME','preempted':'HOME'})
        #-------------------------------------------------

        #=================================================
        ## SPIRAL TEST 
####        # This state will get the AUV transit to point A
####        smach.StateMachine.add('toWork', pathFollowingLOS(lib,myUti, pathMtoA, L_los, uGain, uMax, wp_R), 
####            transitions={'succeeded':'SPIRAL', 'aborted':'HOME','preempted':'HOME'})
####        # This state will get the AUV perform a spiral manoeuvre
####        smach.StateMachine.add('SPIRAL', testSpiral(lib, myUti, pathAtoB, uGain, uMax, errHeadingTol, wp_R, timeDemandHold, timeDelay, depthDemand, depthTol, depthDemandMin, turningAngle), 
####            transitions={'succeeded':'HOME', 'aborted':'HOME','preempted':'HOME'})
####        #-------------------------------------------------

        #=================================================
        ## TURNING CIRCLE TEST 
####        # This state will get the AUV transit to point A
####        smach.StateMachine.add('toWork', pathFollowingLOS(lib,myUti, pathMtoA, L_los, uGain, uMax, wp_R), 
####            transitions={'succeeded':'TURNING', 'aborted':'HOME','preempted':'HOME'})
####        # This state will get the AUV perform a turning curcle manoeuvre: TODO check timeDemandHold
####        smach.StateMachine.add('TURNING', testTurningCircle(lib, myUti, pathAtoB, uGain, uMax, errHeadingTol, wp_R, timeDemandHold, timeDelay), 
####            transitions={'succeeded':'HOME', 'aborted':'HOME','preempted':'HOME'})
####        #-------------------------------------------------
        
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

