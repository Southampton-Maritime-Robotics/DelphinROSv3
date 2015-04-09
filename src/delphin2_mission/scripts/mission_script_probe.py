#!/usr/bin/env python

import roslib; roslib.load_manifest('DelphinROSv2')
import rospy
import smach
import smach_ros
import time
import math
import numpy

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
            transitions={'succeeded':'LOCATION0', 'aborted':'STOP','preempted':'STOP'})  
            
            
################################################################################

#       Read parameters from launch file 
        StartX = rospy.get_param('StartX')  
        StartY = rospy.get_param('StartY') 
        FinishX = rospy.get_param('FinishX') 
        FinishY = rospy.get_param('FinishY') 
        HomeX = rospy.get_param('HomeX') 
        HomeY = rospy.get_param('HomeY') 
        MinAltitude = rospy.get_param('MinAltitude') 
        Speed = rospy.get_param('Speed') 
        n = rospy.get_param('n')   
        
        dx=FinishX-StartY
        dy=FinishY-StartY
        Heading=math.atan2(dx,dy)/numpy.pi*180

        if Heading <0:
            Heading=180-Heading
            
        for i in range(0,n):            

        #Go To Location
        
            locName='LOCATION%s' %i
            pauseName='PAUSE%s' %i
            altitudeName='ALTITUDE%s' %i
            surfaceName='SURFACE%s' %i
            nextlocName='LOCATION%s'%(i+1)
            X=StartX+dx/float(n)*i
            Y=StartY+dy/float(n)*i
        
            smach.StateMachine.add(locName, GoToXYZ(lib, X, Y, 0.0, 4, 0.5, 5, 5, 350),  #WpXnew, WpYnew, depthDemand, XYtolerance, Ztolerance, XYstable_time, Zstable_time, timeout
                transitions={'succeeded':pauseName, 'aborted':'SURFACE_ABORT','preempted':'STOP'})

            #Pause Facing Correct Heading
        
            smach.StateMachine.add(pauseName, GoToDepth(lib, 0.0, 0.4, 20, 0, 30, Heading, 0.0),  #depth_demand, tolerance, stable_time, pitch_demand, timeout, heading, speed
                transitions={'succeeded':altitudeName, 'aborted':'STOP','preempted':'STOP'})
            
            #Go to Altitude
              
              
            smach.StateMachine.add(altitudeName, GoToAltitude(lib, MinAltitude, 0.3, 5, 0, 120, Heading, 0.0), #altitude_demand, altitude_tolerance, stable_time, pitch_demand, timeout, heading_demand, speed_demand
                transitions={'succeeded':surfaceName, 'aborted':surfaceName,'preempted':'SURFACE_ABORT'})
        
            #Surface
            smach.StateMachine.add(surfaceName, Surface(lib, 120.0),  #timeout
                transitions={'succeeded':nextlocName, 'aborted':'SURFACE_ABORT','preempted':'SURFACE_ABORT'})

        #Final Probe

        smach.StateMachine.add(nextlocName, GoToXYZ(lib, FinishX, FinishY, 0.0, 4, 0.5, 5, 5, 350),  #WpXnew, WpYnew, depthDemand, XYtolerance, Ztolerance, XYstable_time, Zstable_time, timeout
            transitions={'succeeded':'FinalPause', 'aborted':'SURFACE_ABORT','preempted':'STOP'})

            #Pause Facing Correct Heading
        
        smach.StateMachine.add('FinalPause', GoToDepth(lib, 0.0, 0.4, 20, 0, 30, Heading, 0.0),  #depth_demand, tolerance, stable_time, pitch_demand, timeout, heading, speed
            transitions={'succeeded':'FinalAltitude', 'aborted':'STOP','preempted':'STOP'})
            
            #Go to Altitude
            
        smach.StateMachine.add('FinalAltitude', GoToAltitude(lib, MinAltitude, 0.3, 5, 0, 120, Heading, 0.0), #altitude_demand, altitude_tolerance, stable_time, pitch_demand, timeout, heading_demand, speed_demand
            transitions={'succeeded':'FinalSurface', 'aborted':'FinalSurface','preempted':'SURFACE_ABORT'})
        
            #Surface
        smach.StateMachine.add('FinalSurface', Surface(lib, 120.0),  #timeout
            transitions={'succeeded':'HOME', 'aborted':'SURFACE_ABORT','preempted':'SURFACE_ABORT'})


        smach.StateMachine.add('SURFACE_ABORT', Surface(lib, 720.0),  #timeout
            transitions={'succeeded':'HOME', 'aborted':'HOME','preempted':'HOME'})

            
        smach.StateMachine.add('HOME', GoToXYZ(lib, HomeX, HomeY, 0.0, 5, 0.5, 5, 5, 1200),  
            transitions={'succeeded':'STOP', 'aborted':'STOP','preempted':'STOP'})
            
        
################################################################################
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

