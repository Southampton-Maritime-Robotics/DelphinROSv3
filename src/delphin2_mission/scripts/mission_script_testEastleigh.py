#!/usr/bin/env python

'''
A mission script for testing at Eastleigh Lake.

The tasks consist of various manoeuvrint test, e.g.
--surge manoeuvre
--sway manoeuvre
--yaw manoeuvre
--spiral manoeuvre

specify:
-reference points (A,B,O,M)

'''

import rospy
import smach
import smach_ros
import time
from pylab import *
import numpy

from delphin2_mission.library_highlevel             import library_highlevel
from state_initialise             import Initialise
from state_stop                   import Stop
from std_msgs.msg import String

from delphin2_mission.utilities     import uti
from state_pathFollowingLOS         import pathFollowingLOS
from state_manoeuvreSurge           import manoeuvreSurge
from state_manoeuvreYaw             import manoeuvreYaw
from state_manoeuvreSway            import manoeuvreSway
from state_manoeuvreSpiral          import manoeuvreSpiral
from state_manoeuvreZigZag          import manoeuvreZigZag
from state_verboseLocation          import verboseLocation

################################################################################

def main():

################################################################################
########### DECLARE PARAMETERS #################################################
################################################################################
    
    rospy.init_node('smach_example_state_machine')
    # Define an instance of highlevelcontrollibrary to pass to all action servers
    lib = library_highlevel()
    myUti = uti()
    
    #Set Up Publisher for Mission Control Log
    pub = rospy.Publisher('MissionStrings', String)
    
    # Allow the topic to come online
    time.sleep(10)
    
    #Read back seat driver Settings    
    overDepth = rospy.get_param('over-depth')
    str = 'Backseat Driver Parameter: over depth set to %s m' %(overDepth)
    pub.publish(str)
    rospy.loginfo(str)
    time.sleep(0.1)
    overPitch = rospy.get_param('over-pitch')    
    str = 'Backseat Driver Parameter: over pitch set to %s deg' %(overPitch)
    pub.publish(str) 
    rospy.loginfo(str)
    time.sleep(0.1)    
    overRoll = rospy.get_param('over-roll')
    str = 'Backseat Driver Parameter: over roll set to %s deg' %(overRoll)
    pub.publish(str) 
    rospy.loginfo(str)
    time.sleep(0.1)         
    maxInternalTemp = rospy.get_param('max-internal-temp')  
    str = 'Backseat Driver Parameter: max internal temperature %s deg' %(maxInternalTemp)
    pub.publish(str)
    rospy.loginfo(str)
    time.sleep(0.1)
    minMotorVoltage = rospy.get_param('min-motor-voltage')
    str = 'Backseat Driver Parameter: min motor voltage %s V' %(minMotorVoltage)
    pub.publish(str) 
    rospy.loginfo(str)
    time.sleep(0.1) 
    missionTimeout = rospy.get_param('mission-timeout') 
    str = 'Backseat Driver Parameter: mission-timeout %s min' %(missionTimeout)
    pub.publish(str) 
    rospy.loginfo(str)

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
    
    # guidance
    L_los = 10. # [m] line-of-sight distance
    wp_R = 3. # [m] radius of acceptance
    
    # speed control
    uMax = 1. # [m/s] maximum surge speed

    # tolerant
    errHeadingTol = 2. # TODO [deg] tolarance in heading error
    
    # general
    controlRate = 20. # 10. # [Hz] control rate in each state, put in -1 to not control the rate
    timeDemandHold = 60. # 60 sec TODO actuator demand will be hold for this many second. used in sway and yaw test
    turningAngle = 360. # [deg] TODO the vehicle has to turn this many degree before move to the next step
    timeDelay = 30. # 30 sec TODO the vehicle will stop for this many second as to let its motion decay to near zero
    
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
        # [1/3] Initialise State (Must Be Run First!)
####        smach.StateMachine.add('INITIALISE', Initialise(lib,15), #15 = timeout for initialisation state
####            transitions={'succeeded':'HOME', 'aborted':'STOP','preempted':'STOP'})
            
################################################################################
        # [2/3] Added States

        #=================================================
        ## VERBOSE LOCATION 
        # This state will keep publishing the range and bearing to the specified location
####        smach.StateMachine.add('toWork', verboseLocation(lib,myUti,B,controlRate), 
####            transitions={'succeeded':'HOME', 'aborted':'HOME','preempted':'HOME'})
        #-------------------------------------------------

        #=================================================
        ## PATH FOLLOWING 
        # This state will get the AUV moving along the pathTest [O->M->A->B->M]
####        smach.StateMachine.add('toWork', pathFollowingLOS(lib,myUti, pathTest, L_los, uMax, wp_R, controlRate, 30), # (..., locationWaitTimeout [sec])
####            transitions={'succeeded':'HOME', 'aborted':'HOME','preempted':'HOME'})
        #-------------------------------------------------
        
        #=================================================
        ## SURGE MANOEUVRE
        # This state will get the AUV transit to point A
####        smach.StateMachine.add('toWork', pathFollowingLOS(lib,myUti, pathMtoA, L_los, uMax, wp_R, controlRate, 30), # (..., locationWaitTimeout [sec])
####            transitions={'succeeded':'SURGE', 'aborted':'HOME','preempted':'HOME'})
####        # This state will guide the AUV back and forth between point A and B with different rudder RPM
####        smach.StateMachine.add('SURGE', manoeuvreSurge(lib,myUti,pathAtoB, uMax, errHeadingTol, wp_R, timeDelay, depthDemand, depthTol, depthDemandMin, controlRate), 
####            transitions={'succeeded':'HOME', 'aborted':'HOME','preempted':'HOME'})
        #-------------------------------------------------
        
        #=================================================
        ## YAW MANOEUVRE 
        # This state will get the AUV transit to point M
####        smach.StateMachine.add('toWork', pathFollowingLOS(lib,myUti, M, L_los, uMax, wp_R, controlRate, 30), # (..., locationWaitTimeout [sec])
####            transitions={'succeeded':'YAW', 'aborted':'HOME','preempted':'HOME'})
####        # This state will keep the AUV at point M and perform yaw motion response test
####        smach.StateMachine.add('YAW', manoeuvreYaw(lib, myUti, M, uMax, wp_R, timeDemandHold, timeDelay, depthDemand, depthTol, depthDemandMin, controlRate), 
####            transitions={'succeeded':'HOME', 'aborted':'HOME','preempted':'HOME'})
        #-------------------------------------------------
        
        #=================================================
        ## SWAY MANOEUVRE 
        # This state will get the AUV transit to point M
####        smach.StateMachine.add('toWork', pathFollowingLOS(lib,myUti, M, L_los, uMax, wp_R, controlRate, 30), # (..., locationWaitTimeout [sec])
####            transitions={'succeeded':'SWAY', 'aborted':'HOME','preempted':'HOME'})
####        # This state will keep the AUV at point M and perform sway motion response test:
####        smach.StateMachine.add('SWAY', manoeuvreSway(lib, myUti, M, uMax, errHeadingTol, wp_R, timeDemandHold, timeDelay, depthDemand, depthTol, depthDemandMin, controlRate), 
####            transitions={'succeeded':'HOME', 'aborted':'HOME','preempted':'HOME'})
        #-------------------------------------------------

        #=================================================
        ## SPIRAL MANOEUVRE 
        # This state will get the AUV transit to point A
        smach.StateMachine.add('toWork', pathFollowingLOS(lib,myUti, pathMtoA, L_los, uMax, wp_R, controlRate, 30), # (..., locationWaitTimeout [sec])
            transitions={'succeeded':'SPIRAL', 'aborted':'HOME','preempted':'HOME'})
        # This state will get the AUV perform a spiral manoeuvre
        smach.StateMachine.add('SPIRAL', manoeuvreSpiral(lib, myUti, pathAtoB, uMax, errHeadingTol, wp_R, timeDemandHold, timeDelay, depthDemand, depthTol, depthDemandMin, turningAngle, controlRate), 
            transitions={'succeeded':'HOME', 'aborted':'HOME','preempted':'HOME'})
        #-------------------------------------------------

        #=================================================
        ## ZIG-ZAG MANOEUVRE 
        # This state will get the AUV transit to point A
#        smach.StateMachine.add('toWork', pathFollowingLOS(lib,myUti, pathMtoA, L_los, uMax, wp_R, controlRate, 30), # (..., locationWaitTimeout [sec])
#            transitions={'succeeded':'ZIGZAG', 'aborted':'HOME','preempted':'HOME'})
#        # This state will get the AUV perform a spiral manoeuvre
#        smach.StateMachine.add('ZIGZAG', manoeuvreZigZag(lib, myUti, pathAtoB, uMax, errHeadingTol, wp_R, timeDemandHold, timeDelay, depthDemand, depthTol, depthDemandMin, controlRate), 
#            transitions={'succeeded':'HOME', 'aborted':'HOME','preempted':'HOME'})
        #-------------------------------------------------
        
        ## GENERIC STATE FOR EASTLEIGH LAKE
        # This state will get the AUV HOME
        smach.StateMachine.add('HOME', pathFollowingLOS(lib,myUti, pathMtoO, L_los, uMax, wp_R, controlRate, 30), # (..., locationWaitTimeout [sec])
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
