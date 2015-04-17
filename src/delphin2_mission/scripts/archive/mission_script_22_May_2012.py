#!/usr/bin/env python

import roslib; roslib.load_manifest('DelphinROSv2')
import rospy
import smach
import smach_ros
import time

from delphin2_mission.library_highlevel            import library_highlevel
from state_initialise             import Initialise
from state_importWaypoints        import ImportWaypoints
from state_stop                   import Stop
from state_goToDepth              import GoToDepth
from state_goToAltitude           import GoToAltitude
from state_goToHeading            import GoToHeading
from state_goToXYZ2               import GoToXYZ
from state_trackFollow            import TrackFollow
from state_trackAltitude          import trackAltitude
from state_surface                import Surface
from state_camera                 import camera
from state_terminalZ              import terminalZ
from state_N                      import N
from state_dryLandTest            import dryLandTest
from state_setTail                import setTail
from state_actionserver_goToDepth import GoToDepthServer
from DelphinROSv2.msg             import compass
from DelphinROSv2.msg             import GoToDepthAction, GoToDepthGoal
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

    ImportWaypoints = rospy.get_param('ImportWaypoints')

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

    ImportWaypoints = rospy.get_param('ImportWaypoints')
    str='Import Waypoints is set to %r in the launch file' %ImportWaypoints
    rospy.loginfo(str)
    pub.publish(str)
    if ImportWaypoints==True:
        pathAndFile1='/home/delphin2/DelphinROSv2/Paths/Testwood9.kml'		#Desired Path
        pub.publish(pathAndFile1)
        pathAndFile2='/home/delphin2/DelphinROSv2/Paths/TestwoodBoundary.kml'	#Operating Area Outline
        plot=False#Plot Waypints File
				
        (WPlongitude, WPlatitude, Load1)=lib.loadWaypoints(pathAndFile1)
        (Blongitude, Blatitude, Load2)=lib.loadWaypoints(pathAndFile2)
        if Load1 ==1 and Load2==1:
            NosWaypoints=len(WPlongitude)
            str= 'Waypoints loaded, Nos of Waypoints = %s' %NosWaypoints
            rospy.loginfo(str)
            pub.publish(str)
            for i in xrange(0,NosWaypoints):
                str='Waypoint %i, Latitude=%s, Longitude=%s' %(NosWaypoints, WPlatitude[i], WPlongitude[i])
                pub.publish(str)
            if plot==True:
                pl=plt.plot(WPlongitude,WPlatitude)
                pl=plt.plot(Blongitude,Blatitude,'r')
                plt.show()	


    # ...and an action server for the GoToDepth action
    #server = GoToDepthServer('test_depth_action', lib, False)

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
            transitions={'succeeded':'START_LOCATION', 'aborted':'STOP','preempted':'STOP'})  
            
        smach.StateMachine.add('START_LOCATION', GoToXYZ(lib, 0, 0.0, 0, 5, 0.5, 5, 5, 1200),  #100.0, 180.0,
            transitions={'succeeded':'STOP', 'aborted':'STOP','preempted':'STOP'})            
        
        smach.StateMachine.add('Alt_test', GoToAltitude(lib, 1.5, 0.4, 60, 0, 120, 45, 0.0), #altitude_demand, altitude_tolerance, stable_time, pitch_demand, timeout, heading_demand, speed_demand
            transitions={'succeeded':'SURFACE', 'aborted':'STOP','preempted':'STOP'})
            
        smach.StateMachine.add('SURFACE', Surface(lib, 60.0),  #timeout
            transitions={'succeeded':'STOP', 'aborted':'STOP','preempted':'STOP'})            
            
################################################################################
        #Dry Land Test
#        smach.StateMachine.add('DRYLANDTEST', dryLandTest(lib), 
#            transitions={'succeeded':'STOP', 'aborted':'STOP','preempted':'STOP'})

################################################################################
            
#        smach.StateMachine.add('CAMERA1_ON', camera(lib, 1, True, 'cam1.avi',True), 
#            transitions={'succeeded':'CAMERA1_OFF', 'aborted':'STOP','preempted':'STOP'})
        
#        smach.StateMachine.add('CAMERA1_OFF', camera(lib, 1, False, 'cam1.avi',False), 
#            transitions={'succeeded':'STOP', 'aborted':'STOP','preempted':'STOP'})
            
#        smach.StateMachine.add('CAMERA0_ON', camera(lib, 0, True, 'cam1.avi',True), 
#            transitions={'succeeded':'ALTITUDE0', 'aborted':'STOP','preempted':'STOP'})
        
#        smach.StateMachine.add('CAMERA0_OFF', camera(lib, 0, False, 'cam1.avi',False), 
#            transitions={'succeeded':'SURFACE', 'aborted':'STOP','preempted':'STOP'})
            
#        smach.StateMachine.add('CAMERA_TEST', camera(lib, 0, True, 'cam0.avi',True), 
#            transitions={'succeeded':'PAUSE', 'aborted':'STOP','preempted':'STOP'})             
            
################################################################################

#        smach.StateMachine.add('PAUSE', GoToDepth(lib, 0.0, 0.4, 20, 0, 30, 300, 0.0),  #depth_demand, tolerance, stable_time, pitch_demand, timeout, heading, speed
#            transitions={'succeeded':'DIVE1', 'aborted':'STOP','preempted':'STOP'})
            
#        smach.StateMachine.add('DIVE1', GoToDepth(lib, 2.0, 0.5, 60, 0, 120, 300, 0.0),  #depth_demand, tolerance, stable_time, pitch_demand, timeout, heading, speed
#            transitions={'succeeded':'DIVE2', 'aborted':'STOP','preempted':'STOP'})
            
#        smach.StateMachine.add('DIVE2', GoToDepth(lib, 2.0, 0.5, 60, 0, 120, 300, 0.5),  #depth_demand, tolerance, stable_time, pitch_demand, timeout, heading, speed
#            transitions={'succeeded':'DIVE3', 'aborted':'STOP','preempted':'STOP'})
            
#        smach.StateMachine.add('DIVE3', GoToDepth(lib, 2.0, 0.5, 60, 0, 120, 300, 1.0),  #depth_demand, tolerance, stable_time, pitch_demand, timeout, heading, speed
#            transitions={'succeeded':'SURFACE', 'aborted':'STOP','preempted':'STOP'})
            
################################################################################

#        smach.StateMachine.add('PAUSE_NORTH', GoToDepth(lib, 0.0, 0.4, 10, 0, 30, 0, 0.0),  #depth_demand, tolerance, stable_time, pitch_demand, timeout, heading, speed
#            transitions={'succeeded':'NORTH', 'aborted':'STOP','preempted':'STOP'})
            
#        smach.StateMachine.add('NORTH', GoToDepth(lib, 0.0, 0.4, 30, 0, 120, 0, 0.8),  #depth_demand, tolerance, stable_time, pitch_demand, timeout, heading, speed
#            transitions={'succeeded':'PAUSE_EAST', 'aborted':'STOP','preempted':'STOP'})
            
#        smach.StateMachine.add('PAUSE_EAST', GoToDepth(lib, 0.0, 0.4, 10, 0, 30, 90, 0.0),  #depth_demand, tolerance, stable_time, pitch_demand, timeout, heading, speed
#            transitions={'succeeded':'EAST', 'aborted':'SURFACE','preempted':'SURFACE'})
            
#        smach.StateMachine.add('EAST', GoToDepth(lib, 0.0, 0.4, 30, 0, 120, 90, 0.8),  #depth_demand, tolerance, stable_time, pitch_demand, timeout, heading, speed
#            transitions={'succeeded':'PAUSE_SOUTH', 'aborted':'SURFACE','preempted':'SURFACE'})
            
#        smach.StateMachine.add('PAUSE_SOUTH', GoToDepth(lib, 0.0, 0.4, 10, 0, 30, 180, 0.0),  #depth_demand, tolerance, stable_time, pitch_demand, timeout, heading, speed
#            transitions={'succeeded':'SOUTH', 'aborted':'SURFACE','preempted':'SURFACE'})
            
#        smach.StateMachine.add('SOUTH', GoToDepth(lib, 0.0, 0.4, 30, 0, 120, 180, 0.8),  #depth_demand, tolerance, stable_time, pitch_demand, timeout, heading, speed
#            transitions={'succeeded':'PAUSE_WEST', 'aborted':'SURFACE','preempted':'SURFACE'})
            
#        smach.StateMachine.add('PAUSE_WEST', GoToDepth(lib, 0.0, 0.4, 10, 0, 30, 270, 0.0),  #depth_demand, tolerance, stable_time, pitch_demand, timeout, heading, speed
#            transitions={'succeeded':'WEST', 'aborted':'SURFACE','preempted':'SURFACE'})
            
#        smach.StateMachine.add('WEST', GoToDepth(lib, 0.0, 0.4, 30, 0, 120, 270, 0.8),  #depth_demand, tolerance, stable_time, pitch_demand, timeout, heading, speed
#            transitions={'succeeded':'END_LOCATION', 'aborted':'SURFACE','preempted':'SURFACE'})
            
            

            
#        smach.StateMachine.add('TANK_DIVE', GoToDepth(lib, 3.0, 0.4, 60, 0, 120, 45, 0.0),
#            transitions={'succeeded':'STOP', 'aborted':'STOP','preempted':'STOP'})

#################################################################################
            
    
                   
#################################################################################

#        smach.StateMachine.add('ALTITUDE0', GoToDepth(lib, 3.0, 0.4, 60, 0, 180, 45, 0.0),  #depth_demand, tolerance, stable_time, pitch_demand, timeout, heading, speed
#            transitions={'succeeded':'CAMERA1_ON', 'aborted':'SURFACE','preempted':'SURFACE'})

 #       smach.StateMachine.add('PAUSE_ALT', GoToDepth(lib, 0.0, 0.4, 20, 0, 30, 270, 0.0),  #depth_demand, tolerance, stable_time, pitch_demand, timeout, heading, speed
#            transitions={'succeeded':'CAMERA1_ON', 'aborted':'STOP','preempted':'STOP'})
        
#        smach.StateMachine.add('ALTITUDE1', trackAltitude(lib, 1.5, 270, 0.0, 40), #altitude_demand, heading_demand, speed_demand, time):
#            transitions={'succeeded':'ALTITUDE2', 'aborted':'SURFACE','preempted':'SURFACE'})
            
#        #Track altitude
#        smach.StateMachine.add('ALTITUDE2', trackAltitude(lib, 1.0, 270, 0.0, 40), #altitude_demand, heading_demand, speed_demand, time):
#            transitions={'succeeded':'ALTITUDE3', 'aborted':'SURFACE','preempted':'SURFACE'})
            
#        smach.StateMachine.add('ALTITUDE3', trackAltitude(lib, 1.0, 270, 0.4, 100), #altitude_demand, heading_demand, speed_demand, time):
#            transitions={'succeeded':'ALTITUDE4', 'aborted':'SURFACE','preempted':'SURFACE'})
            
#        smach.StateMachine.add('ALTITUDE4', trackAltitude(lib, 0.75, 270, 0.4, 100), #altitude_demand, heading_demand, speed_demand, time):#
#            transitions={'succeeded':'SURFACE', 'aborted':'SURFACE','preempted':'SURFACE'})

################################################################################

        #Home State Return to Safe Location
#       	smach.StateMachine.add('HOME', GoToXYZ(lib, 0.0, 0.0, 0.0, 5, 0.5, 5, 5, 1200),  #, WpYnew, depthDemand, XYtolerance, Ztolerance, XYstable_time, Zstable_time, timeout
#            transitions={'succeeded':'STOP', 'aborted':'END_LOCATION','preempted':'STOP'})
            
#        smach.StateMachine.add('START_LOCATION', GoToXYZ(lib, -10.0, 20.0, 0, 5, 0.5, 5, 5, 1200),  #100.0, 180.0,
#            transitions={'succeeded':'PAUSE_ALT', 'aborted':'STOP','preempted':'STOP'})
            
#        smach.StateMachine.add('END_LOCATION', GoToXYZ(lib, -80.0, 0.0, 0.0, 5, 0.5, 5, 5, 1200),  
#            transitions={'succeeded':'STOP', 'aborted':'STOP','preempted':'STOP'})
            
#        smach.StateMachine.add('POINTNORTH', GoToHeading(lib, 0, 10, 15, 60),  #demand, tolerance, stable_time, timeout
#            transitions={'succeeded':'HOME', 'aborted':'STOP','preempted':'STOP'}) 
            
        
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

