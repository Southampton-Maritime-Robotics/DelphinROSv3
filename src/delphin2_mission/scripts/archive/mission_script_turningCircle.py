#!/usr/bin/env python

import roslib; roslib.load_manifest('delphin2_mission')
import rospy
import smach
import smach_ros
import time

from library_highlevel            import library_highlevel
from state_initialise             import Initialise
from state_TestHeading            import TestHeading
from state_importWaypoints        import ImportWaypoints
from state_stop                   import Stop
from state_goToDepth              import GoToDepth
from state_TurningCircle          import TurningCircle
from state_goHOME                 import GoToHome
from state_goToHeading            import GoToHeading
from state_goToXYZ2               import GoToXYZ
from state_trackFollow            import TrackFollow
from state_trackAltitude          import trackAltitude
from state_trackValley            import trackValley
from state_goForwards             import GoForwards
from state_surface                import Surface
from state_camera                 import camera
from state_terminalZ              import terminalZ
from state_N                      import N
from state_dryLandTest            import dryLandTest
from state_setTail                import setTail
from state_actionserver_goToDepth import GoToDepthServer
from delphin2_mission.msg             import compass
from delphin2_mission.msg           import GoToDepthAction, GoToDepthGoal
from actionlib                    import *
from actionlib.msg                import *
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
    #Load Waypoints


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
#        Initialise State Should Be Run First!
        
                
        smach.StateMachine.add('INITIALISE', Initialise(lib,15), #15 = timeout for initialisation state
            transitions={'succeeded':'START', 'aborted':'STOP','preempted':'STOP'})  
            
        smach.StateMachine.add('START', GoToXYZ(lib, 0, 6, 0, 5, 5, 5, 5, 1200), #WpXnew, WpYnew, depthDemand, XYtolerance, Ztolerance, XYstable_time, Zstable_time, timeout
            transitions={'succeeded':'POINT', 'aborted':'POINT','preempted':'POINT'})
            
        smach.StateMachine.add('POINT', GoToHeading(lib, 0, 20, 5, 60),  #demand, tolerance, stable_time, timeout
            transitions={'succeeded':'CIRCLE', 'aborted':'STOP','preempted':'STOP'})             

        smach.StateMachine.add('CIRCLE', TurningCircle(lib, 10, 20, 20, 60),  #propdemand, timeIstPhase, rudderAngle, time2ndPahse, timeout
            transitions={'succeeded':'FINISH', 'aborted':'STOP','preempted':'STOP'})     

        smach.StateMachine.add('FINISH', GoToXYZ(lib, 0, 10.0, 0, 5, 0.5, 5, 5, 60), #WpXnew, WpYnew, depthDemand, XYtolerance, Ztolerance, XYstable_time, Zstable_time, timeout
            transitions={'succeeded':'STOP', 'aborted':'STOP','preempted':'STOP'})

            
        # generic states
        smach.StateMachine.add('STOP', Stop(lib), 
            transitions={'succeeded':'finish'})    
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

