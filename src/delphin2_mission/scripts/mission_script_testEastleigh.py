#!/usr/bin/env python

'''
A mission script for testing at Eastleigh Lake.

The tasks consist of various manoeuvrint test, e.g.
--surge manoeuvre
--spiral manoeuvre
--zigzag manoeuvre

specify:
-reference points A,B,O and M

'''

import rospy
import smach
import smach_ros
import time
from pylab import *
import numpy

from std_msgs.msg import String

from delphin2_mission.library_highlevel     import library_highlevel
from delphin2_mission.utilities             import uti
from state_initialise                       import Initialise
from state_stop                             import Stop
from state_pathFollowingLOS                 import pathFollowingLOS
from state_manoeuvreSurge                   import manoeuvreSurge
from state_manoeuvreSpiral                  import manoeuvreSpiral
from state_manoeuvreZigZag                  import manoeuvreZigZag
from state_verboseLocation                  import verboseLocation

################################################################################

def main():

################################################################################
########### DECLARE PARAMETERS #################################################
################################################################################
    
    rospy.init_node('smach_example_state_machine')
    # Define an instance of highlevelcontrollibrary to pass to all action servers
    lib = library_highlevel()
    myUti = uti()
    
    # Allow the topic to come online
    time.sleep(10)

    ### points defined relative to the origin O #
    # North pier of the Eastleigh lake (50.957024,-1.366769)
    O = array([-0.,0.]) # home: shifted from the origin a little to make sure it will not collide with the pier
    A = array([-28.,-20.]) # reference point A
    B = array([-1.,50.]) # reference point B
#    # South pier of the Eastleigh lake (50.956473,-1.366835)
#    O = array([-2.,2.]) # home: shifted from the origin a little to make sure it will not collide with the pier
#    A = array([-20.,14.]) # reference point A
#    B = array([-30.,64.]) # reference point B
    
    M = array([(A[0]+B[0])/2., (A[1]+B[1])/2.]) # mid-point between A and B

    # Reference paths created from the reference points
    pathAtoB = numpy.vstack((A,B)).T
    pathBtoA = numpy.vstack((B,A)).T
    pathMtoO = numpy.vstack((M,O)).T
    pathMtoA = numpy.vstack((M,A)).T
    pathOtoM = numpy.vstack((O,M)).T
    pathTest = numpy.vstack((M,A,B,M)).T
        
    # general
    timeDemandHold = 60.    # 60 sec TODO actuator demand will be hold for this many second. used in sway and yaw test
    turningAngle = 720.     # [deg] TODO the vehicle has to turn this many degree before move to the next step
    timeDelay = 30.         # 30 sec TODO the vehicle will stop for this many second as to let its motion decay to near zero
    errHeadingTol = 3.      # the heading error band that count as the AUV reachs a desired heading
    
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

        # creating the sequence state machine
        se = smach.Sequence(outcomes=['succeeded','aborted','preempted'],
                            connector_outcome = 'succeeded')

        with se:
            # [1/3] Initialise State (Must Be Run First!)
####            smach.Sequence.add('INITIALISE',Initialise(lib, 15)) #15 = timeout for initialisation state

            # [2/3] Added States
            #=================================================
####            ## VERBOSE LOCATION 
####            # This state will keep publishing the range and bearing to the specified location
####            smach.Sequence.add('VERBOSE_LOCATION', verboseLocation(lib,myUti,B,controlRate))
####            #-------------------------------------------------

            #=================================================
####            ## PATH FOLLOWING 
####            # This state will get the AUV moving along the pathTest [O->M->A->B->M]
####            smach.Sequence.add('toWork', pathFollowingLOS(lib, myUti, pathTest))
            #-------------------------------------------------
            
            #=================================================
####            ## SURGE MANOEUVRE
####            # This state will get the AUV transit to point A
####            smach.Sequence.add('toWork', pathFollowingLOS(lib, myUti, pathMtoA))
####            # This state will guide the AUV back and forth between point A and B with different rudder RPM
####            smach.Sequence.add('SURGE', manoeuvreSurge(lib,myUti,pathAtoB, uMax, errHeadingTol, wp_R, timeDelay, depthDemand, depthTol, depthDemandMin, controlRate))
####            #-------------------------------------------------

            #=================================================
            ## SPIRAL MANOEUVRE 
            # This state will get the AUV transit to point A
            smach.Sequence.add('toWork', pathFollowingLOS(lib, myUti, pathMtoA))
            # This state will get the AUV perform a spiral manoeuvre
            smach.Sequence.add('SPIRAL', manoeuvreSpiral(lib, myUti, pathAtoB, errHeadingTol, timeDelay, depthDemand, depthTol, depthDemandMin, turningAngle))
            #-------------------------------------------------

            #=================================================
            ## ZIG-ZAG MANOEUVRE 
####            # This state will get the AUV transit to point A
####            smach.Sequence.add('toWork', pathFollowingLOS(lib, myUti, pathMtoA))
####            # This state will get the AUV perform a spiral manoeuvre
####            smach.Sequence.add('ZIGZAG', manoeuvreZigZag(lib, myUti, pathAtoB, uMax, errHeadingTol, wp_R, timeDemandHold, timeDelay, depthDemand, depthTol, depthDemandMin, controlRate))
####            #-------------------------------------------------

            #=================================================
            ## GET THE AUV HOME
            smach.StateMachine.add('HOME', pathFollowingLOS(lib, myUti, pathMtoO))

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
