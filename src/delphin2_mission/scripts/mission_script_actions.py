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
    
    # sleep for this many second to allow the system to come online
    time.sleep(5)
    controlRate = 10. # Hz

    O = array([-2.,2.]) # home: shifted from the origin a little to make sure it will not collide with the pier
    A = array([-28.,-20.]) # reference point A
    B = array([-1.,50.]) # reference point B
    
    M = array([(A[0]+B[0])/2., (A[1]+B[1])/2.]) # mid-point between A and B

    # Reference paths created from the reference points
    pathAtoB = numpy.vstack((A,B)).T
    pathBtoA = numpy.vstack((B,A)).T
    pathMtoO = numpy.vstack((M,O)).T
    pathMtoA = numpy.vstack((M,A)).T
    pathOtoM = numpy.vstack((O,M)).T
    pathTest = numpy.vstack((O,M,A,B,M)).T

    homeLocation = array([-2,2])
    
    # guidance
    L_los = 5. # [m] line-of-sight distance
    wp_R = 3. # [m] radius of acceptance
    
    # speed control
    uGain = 0.07 # gain to control speed variation: high value -> less overshoot -> more settling time
    uMax = 1. # [m/s] maximum surge speed

    # Create a SMACH state machine - with outcome 'finish'
    sm = smach.StateMachine(outcomes=['finish'])
    
    #Allow system to come online
    time.sleep(10)

    # Open the container
    with sm:
        # Add states to the container
        # generic state

################################################################################
########### MAIN CODE ##########################################################
################################################################################

################################################################################
        # [1/3] Initialise State (Must Be Run First!) # TODO uncomment me
####        smach.StateMachine.add('INITIALISE', Initialise(lib,15), #15 = timeout for initialisation state
####            transitions={'succeeded':'ACTIONS', 'aborted':'STOP','preempted':'STOP'})

################################################################################
        # [2/3] Added States
####        smach.StateMachine.add('ACTIONS', actions(lib), 
####            transitions={'succeeded':'STOP', 'aborted':'STOP','preempted':'STOP'})
        
####        smach.StateMachine.add('GoFORWARD', GoForwards(lib, myUti, 10, 10, controlRate), # (lib, myUti, timeout [sec], propDemand, controlRate [Hz])
####            transitions={'succeeded':'STOP', 'aborted':'STOP','preempted':'STOP'})

####        smach.StateMachine.add('GoSIDEWAY', GoSideway(lib, myUti, 30, -1000, controlRate), # (lib, myUti, timeout [sec], thrusterDemand, controlRate [Hz])
####            transitions={'succeeded':'STOP', 'aborted':'STOP','preempted':'STOP'})
            
####        smach.StateMachine.add('GoToHEADING', GoToHeading(lib, myUti, 90, 5, 10, 30, controlRate), # (lib, myUti, demandHeading [deg], tolerance [deg], stable_time [sec], timeout [sec], controlRate [Hz])
####            transitions={'succeeded':'STOP', 'aborted':'STOP','preempted':'STOP'})

        smach.StateMachine.add('GoHOME', pathFollowingLOS(lib,myUti, pathTest, L_los, uGain, uMax, wp_R, controlRate, 30), # (..., locationWaitTimeout [sec])
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
