#!/usr/bin/env python

'''
A mission script to test the PID-based depth-pitch controller(see depthPitchPID.py).
There are two state:
- depthPitchControl is used when testing at zero speed
- depthPitchControlAtSpeed is used when testing at forward speeds

'''

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
from hardware_interfaces.msg      import compass
from std_msgs.msg import String
import matplotlib.pyplot as plt;

from state_testDepthPitchControl             import testDepthPitchControl
from state_testDepthPitchControlAtSpeed      import testDepthPitchControlAtSpeed
from state_manoeuvreYaw_in_tank              import manoeuvreYaw

################################################################################
           
def main():

    global WPlongitude 
    global WPlatitude
    global Blongitude 
    global Blatitude
    global NosWayPoints		
    global longOrigin
    global latOrigin

    rospy.init_node('smach_example_state_machine')
    
    # Define an instance of highlevelcontrollibrary to pass to all action servers
    lib = library_highlevel()
    
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
    
    # Create a SMACH state machine - with outcome 'finish'
    sm = smach.StateMachine(outcomes=['finish'])

    # Open the container
    with sm:
        # Add states to the container
        # generic state

################################################################################
########### MAIN CODE ##########################################################
################################################################################

################################################################################
        # [1/3] Initialise State (Must Be Run First!)
        smach.StateMachine.add('INITIALISE', Initialise(lib,15), #15 = timeout for initialisation state
            transitions={'succeeded':'SEQUENCE', 'aborted':'STOP','preempted':'STOP'})

################################################################################
        # [2/3] Added States
        # creating the sequence state machine
        se = smach.Sequence(outcomes=['succeeded','aborted','preempted'],
                            connector_outcome = 'succeeded')
        with se:
#            smach.Sequence.add('depthPitchControl', testDepthPitchControl(lib))
            smach.Sequence.add('depthPitchControl', testDepthPitchControlAtSpeed(lib))
        
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
