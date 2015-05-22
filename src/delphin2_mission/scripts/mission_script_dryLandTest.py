#!/usr/bin/env python

'''
# Dry Land Test:
test of all sensors and actuators before the vehicle is run in water
- run thrusters and propeller at lower RPM
'''

import rospy
import smach
import smach_ros
import time

from delphin2_mission.library_highlevel             import library_highlevel
from state_initialise             import Initialise
from state_stop                   import Stop
from state_goToHeading            import GoToHeading
from state_trackFollow            import TrackFollow
from state_trackAltitude          import trackAltitude
from state_surface                import Surface
from state_camera                 import camera
from state_dryLandTest            import dryLandTest
from std_msgs.msg import String
import matplotlib.pyplot as plt;

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
    
    #Allow system to come online
    time.sleep(5)
    
    # Create a SMACH state machine - with outcome 'finish'
    sm = smach.StateMachine(outcomes=['finish'])

    # Open the container
    with sm:
        # Add states to the container
        # generic state

################################################################################
########### MAIN CODE ##########################################################
################################################################################
	#Generate States

################################################################################
#        Initialise State Should Be Run First!
        smach.StateMachine.add('INITIALISE', Initialise(lib,15), #15 = timeout for initialisation state
            transitions={'succeeded':'DRYLANDTEST', 'aborted':'STOP','preempted':'STOP'})  
            
################################################################################
        #Dry Land Test
        smach.StateMachine.add('DRYLANDTEST', dryLandTest(lib), 
            transitions={'succeeded':'STOP', 'aborted':'STOP','preempted':'STOP'})

################################################################################
  
################################################################################
        # generic states
        smach.StateMachine.add('STOP', Stop(lib), 
            transitions={'succeeded':'finish'})    

################################################################################
################################################################################
#       AUTO GENERATED STATES 
        
        if ImportWaypoints==True:
        #Auto Generate States to take sub to each waypoint
            for i in xrange (0,NosWaypoints):
                    depthDemand=0.0
                    X,Y,dummy,dummy=lib.distanceandbearingTwoLatLong(latOrigin,WPlatitude[i], longOrigin,WPlongitude[i])
		    useCurrentLocAsInitialXY=False		#If True will use current location when enters state as initial location for calculating path if 				false will use initial X and inital Y
		    initialX=0
		    initialY=0
                    XYtolerance=7.5
                    Ztolerance=2
                    XYstable_time=1
                    Zstable_time=1
                    timeout=420
                    StateName='GOTOWAYPOINT%s' %i
                    if i< NosWaypoints-1:		
			NextState='GOTOWAYPOINT%s' %(i+1)
                    else:
			NextState='HOME'

                    smach.StateMachine.add(StateName, TrackFollow(lib, X, Y, depthDemand, useCurrentLocAsInitialXY, initialX, initialY, XYtolerance, Ztolerance, XYstable_time, Zstable_time,timeout),  
                        transitions={'succeeded':NextState, 'aborted':'HOME','preempted':'STOP'})

        

            

################################################################################
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

