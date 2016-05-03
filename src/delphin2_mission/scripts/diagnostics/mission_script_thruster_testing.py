#!/usr/bin/env python

# Goal of test: relation between power consumption and truster demand
#
# Operates both vertical thrusters at the same demand
# demand varies during the test
# current consumed by trusters is measured

import rospy
import smach
import smach_ros
import time

from delphin2_mission.library_highlevel            import library_highlevel
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

from state_thruster_testing       import thruster_testing

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
    
    #Display and Record Starting Location
    latOrigin = rospy.get_param('lat_orig')
    longOrigin = rospy.get_param('long_orig')
    str = 'Origin Loaded, Longitude=%s Latitude=%s' %(longOrigin,latOrigin)
    pub.publish(str)
    rospy.loginfo(str)
    X=lib.getX()
    Y=lib.getY()
    str = 'Initial Position X=%s and Y=%s' %(X,Y)
    rospy.loginfo(str)
    pub.publish(str)

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
          

        # operate thruster
        smach.StateMachine.add('THRUSTER_TESTING', thruster_testing(lib), 
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

