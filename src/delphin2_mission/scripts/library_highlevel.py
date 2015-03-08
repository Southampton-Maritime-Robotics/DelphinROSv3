#!/usr/bin/python
#import roslib; roslib.load_manifest('delphin2_mission')
import rospy
import numpy
import math
import time
import re

#import message types for publishing:
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from std_msgs.msg import String

#import message types for subscribing:
from hardware_interfaces.msg    import compass
from hardware_interfaces.msg    import position
from hardware_interfaces.msg    import tail_setpoints
from hardware_interfaces.msg    import tsl_setpoints
from hardware_interfaces.msg    import tail_feedback
from hardware_interfaces.msg    import tsl_feedback
from hardware_interfaces.msg    import altitude
from hardware_interfaces.msg    import status
from hardware_interfaces.msg    import camera
from lowlevel_controllers.msg   import heading_control
from hardware_interfaces.msg    import sonar_data
from lowlevel_controllers.msg   import depthandpitch_MPC

class library_highlevel:
# library class for high-level controller.  Defines the inputs, output and basic functionality that
# all high-level controllers should inherit.

    def __init__(self):
        #constructor method - creates objects of type 'HighLevelController'
        
        #set up publishers (stored as object variables)
        self.pub_sway_demand               = rospy.Publisher('sway_demand', Float32)
        self.pub_prop_demand               = rospy.Publisher('prop_demand',Int8)                #tail_section.py currently expecting uint8
        self.pub_heading_demand            = rospy.Publisher('heading_demand', Float32) 
        self.pub_depth_demand              = rospy.Publisher('depth_demand', Float32) 
        self.pub_speed                     = rospy.Publisher('speed_demand', Float32)
        self.pub_pitch_demand              = rospy.Publisher('pitch_demand', Float32) 
        self.pub_tsl_onOff_horizontal      = rospy.Publisher('TSL_onOff_horizontal', Bool)      #turns thrusters on and off 
        self.pub_tsl_onOff_vertical        = rospy.Publisher('TSL_onOff_vertical', Bool)        #turns thrusters on and off
        self.pub_tail_setpoints_vertical   = rospy.Publisher('tail_setpoints_vertical', tail_setpoints)
        self.pub_tail_setpoints_horizontal = rospy.Publisher('tail_setpoints_horizontal', tail_setpoints)
        self.pub_tsl_heading               = rospy.Publisher('TSL_setpoints_horizontal', tsl_setpoints)
        self.pub_tsl_depth                 = rospy.Publisher('TSL_setpoints_vertical', tsl_setpoints)
        self.pub_heading_control_onOff     = rospy.Publisher('Heading_onOFF', Bool)             #turns heading controller on and off
        self.pub_depth_control_onOff       = rospy.Publisher('Depth_onOFF', Bool)               #turns depth controller on and off
        self.pub_camera                    = rospy.Publisher('Camera', camera)
        self.pub_pitch_control_onOff       = rospy.Publisher('Pitch_onOFF', Bool)
        self.Mission_pub                   = rospy.Publisher('MissionStrings', String)
        self.pub_SMS                       = rospy.Publisher('SMS_message', String)
        self.pub_light                     = rospy.Publisher('light_onOff', Bool)
        
        #set up subscribers
        rospy.Subscriber('compass_out', compass, self.callback_compass)                         #compass data
        rospy.Subscriber('position_dead', position, self.callback_position)                     #position (dead reckoning)
        rospy.Subscriber('altimeter_out', altitude, self.callback_altitude)
        rospy.Subscriber('back_seat_flag', Int8, self.callback_back_seat_flag)
        rospy.Subscriber('TSL_feedback', tsl_feedback, self.callback_tsl_feedback)
        rospy.Subscriber('status', status, self.callback_status)
        rospy.Subscriber('tail_output', tail_feedback, self.callback_tail_feedback)
        rospy.Subscriber('Heading_controller_values', heading_control, self.callback_heading_control)      
        rospy.Subscriber('sonar_processed', sonar_data, self.callback_sonar_range)       
        rospy.Subscriber('DepthandPitch_MPC_values', depthandpitch_MPC, self.callback_depthandpitchMPC)
        
        #gets parameters from server ####Don't Need to be Here ????????
        self.__maxDepthDemand = rospy.get_param('max-depth-demand')
        self.overdepth = rospy.get_param('over-depth')
        self.overpitch = rospy.get_param('over-pitch')
        
        #initilise empty object variables to hold latest data from subscribers
        ######### might need to set default values in message files
        self.__compass = compass()
        self.__position = position()
        self.__altitude = altitude()
        self.__sonar_range = 0
        self.__back_seat_flag = 0 #back_seat_flag is 0 when no errors are present
        self.__motor_voltage = 0
        
        self.__TSL_status = 0 #initisliased to indicate presence of errors - this will be set to 1 once node has been tested during vehicle initialisation
        self.__tail_status = 0
        self.__altimeter_status = 0
        self.__gps_status = 0
        self.__compass_status = 0
        
        self.__heading_error=0.0
        self.__altitude=0.0
   
        self.__motor_voltage = 0.0
        self.__T0rpm = 0.0
        self.__T1rpm = 0.0
        self.__T2rpm = 0.0
        self.__T3rpm = 0.0
        self.__CS_b = 0.0
        self.__CS_c = 0.0
        self.__CS_d = 0.0
        self.__CS_e = 0.0
        self.__PropRPM = 0.0
   
    # stops vehicle and shuts down ROS
    # may be better as an action....
    def stop(self):
        str="Stop method invoked - ROS will shut down in 5 seconds"
        rospy.logfatal(str)
        self.Mission_pub.publish(str)
        self.setArduinoThrusterHorizontal(0, 0)
        self.switchHorizontalThrusters(0)
        self.setArduinoThrusterVertical(0, 0)
        self.switchVerticalThrusters(0)
        self.setRearProp(0)
        time.sleep(5)
        rospy.logfatal("Shutting down")
        self.Mission_pub.publish('Shutting Down')
        rospy.signal_shutdown('mission finished')
        
    def sendSMS(self, message):
        self.pub_SMS.publish(message)
        
    def lightOnOff(self, onOff):
        self.pub_light.publish(onOff)
    
    # sets sway 'demand' (tsl_setpoints ~500-1000)
    def sway(self, demand):
        str = "Swaying %s (positive to starboard)" %demand
        rospy.loginfo(str)
        # keep current heading (not current heading demand!)
        self.changeHeadingBy(0)
        self.pub_sway_demand.publish(demand)
        self.switchHorizontalThrusters(1)
        self.switchHeadingOnOff(1)
        
    
    # sets a 'demand' for the rear prop (between 0 and 22ish)
    def setRearProp(self, demand):
        #publish rear prop_demand
        print 'Rear prop to', demand
        str = "Setting rear prop demand %s" %demand
        rospy.loginfo(str)
        self.pub_prop_demand.publish(demand)
        
    # set a 'demand' (in degrees) for the rudder angle
    def setRudderAngle(self, demand):
        vertical=tail_setpoints()
        vertical.cs0 = demand
        vertical.cs1 = demand
        str = "Ruddder demand %.3f deg" %demand
        rospy.loginfo(str)
        #publish rear rudder_demand
        self.pub_tail_setpoints_vertical.publish(vertical)

    # set a 'demand' (in degrees) for the rudder angle
    def setControlSurfaceAngle(self, b,c,d,e): # (VerUp,HorRight,VerDown,HorLeft)
        vertical   = tail_setpoints()
        horizontal = tail_setpoints()
        vertical.cs0 = b
        vertical.cs1 = d
        horizontal.cs0 = c
        horizontal.cs1 = e
        str = "Control surface demands - top: %s, sb: %s, bottom: %s, p: %s deg" %(b,c,d,e)
        rospy.loginfo(str)
        self.pub_tail_setpoints_vertical.publish(vertical)
        self.pub_tail_setpoints_horizontal.publish(horizontal)

    # manually send tsl setpoint values for horizontal thrusters
    def setArduinoThrusterHorizontal(self, thruster2, thruster3):
        output=tsl_setpoints()
        output.thruster0=thruster2
        output.thruster1=thruster3
        str = "Horizontal Thruster Setpoints %s %s" %(thruster2,thruster3)
        rospy.loginfo(str)
        self.switchHorizontalThrusters(1)
        self.switchHeadingOnOff(0)
        time.sleep(0.5)
        self.pub_tsl_heading.publish(output)

    # manually send tsl setpoint values for vertical thrusters
    def setArduinoThrusterVertical(self, thruster0, thruster1):
        output=tsl_setpoints()
        output.thruster0=thruster0
        output.thruster1=thruster1
        
        #print statement might not work! hasn't been tested!
#        str = "Vertical Thruster Setpoints %s %s" %(thruster0,thruster1)
#        rospy.loginfo(str)
        self.switchVerticalThrusters(1)
        self.switchDepthOnOff(0)
        time.sleep(0.5)
        self.pub_tsl_depth.publish(output)

    # move to depth 'demand' (metres)
    def setDepth(self, demand):
        #publish depthDemand
        #print 'Setting depth demand: ', demand, 'm'
        self.switchDepthOnOff(1)
        self.switchVerticalThrusters(1)
#        print demand
        if (demand < self.__maxDepthDemand and demand > 0):
            self.pub_depth_demand.publish(demand)
#            str = "Setting depth demand %sm" %demand
#            rospy.loginfo(str)
######        elif demand <= 0.2: # TODO this case will never be in use
######            str = "Requested depth demand %sm <=0m, turning vertical thrusters Off. " %	demand
######            self.pub_depth_demand.publish(demand)
######            rospy.logwarn(str)   
######            #self.switchDepthOnOff(0)
######            self.switchVerticalThrusters(0)           
        else:
            self.pub_depth_demand.publish(self.__maxDepthDemand)
            str = "Requested depth %sm > maxDepthDemand (%sm)" %(demand, self.__maxDepthDemand)
            rospy.logwarn(str)
            str = "Setting depth demand %sm" %self.__maxDepthDemand
            rospy.logwarn(str)
            
    def setSpeed(self, demand):
        self.pub_speed.publish(demand)
        str = "Setting speed demand %s m/s" %demand
        rospy.loginfo(str)
    
    def setPitch(self, demand):
        self.switchPitchOnOff(1)
        self.switchVerticalThrusters(1)
        
        if (abs(demand) < self.overpitch):
            self.pub_pitch_demand.publish(demand)
#            str = "Setting pitch demand %sdeg" %demand
#            rospy.loginfo(str)
        else:
            demand_mod = numpy.sign(demand)*self.overpitch
            self.pub_pitch_demand.publish(demand_mod)
#            str = "Magnitude of requested pitch %sdeg > maxPitchDemand (%sdeg)" %(demand, self.overpitch)
#            rospy.logwarn(str)
#            str = "Setting pitch demand %sdeg" %demand_mod
#            rospy.logwarn(str)

    # move to heading 'demand' (degrees)
    def setHeading(self, demand):
        #publish headingDemand
        cur_heading=self.getHeading()
#        str = "Setting heading demand %.3f deg, current heading %.3f deg" %(demand, cur_heading)
#        rospy.loginfo(str)
        self.switchHorizontalThrusters(1)
        self.switchHeadingOnOff(1)
        self.pub_heading_demand.publish(demand)

    # change heading by 'headingChange' (degrees)
    def changeHeadingBy(self, headingChange):
        #change heading by an amount (headingChange)
        self.setHeading(self.__compass.heading + headingChange)

    # change depth by 'depthChange' (metres)
    def changeDepthBy(self, depthChange):
        #change depth by an amount (depthChange)
        self.setDepth(self.__compass.depth + depthChange)
    
    # switch horizontal thrusters on or off {1,0}
    def switchHorizontalThrusters(self, onOff):
        if onOff == 1:
            self.pub_tsl_onOff_horizontal.publish(1)
            str = "Switch Horizontal Thruster ON"
            rospy.logdebug(str)	
        else: 
            self.pub_tsl_onOff_horizontal.publish(0) 
            str = "Switch Horizontal Thruster OFF"
            rospy.logdebug(str)	
            
    # switch vertical thrusters on or off {1,0}
    def switchVerticalThrusters(self, onOff):
        if onOff == 1:
            self.pub_tsl_onOff_vertical.publish(1) 
            str = "Switch Vertical Thruster ON"
            rospy.logdebug(str)	        
        else:
            self.pub_tsl_onOff_vertical.publish(0) 
            str = "Switch Vertical Thruster OFF"
            rospy.logdebug(str)	 

    # switch heading controller on or off {1,0}
    def switchHeadingOnOff(self,onOff):
        if onOff ==1:
            self.pub_heading_control_onOff.publish(1)
            str = "Switch Heading Control ON"
            rospy.logdebug(str)	 
        else:
            self.pub_heading_control_onOff.publish(0)
            str = "Switch Heading Control OFF"
            rospy.logdebug(str)	 

    # switch depth controller on or off {1,0}
    def switchDepthOnOff(self,onOff):
        if onOff ==1:
            self.pub_depth_control_onOff.publish(1)
            str = "Switch Depth Control ON"
            rospy.logdebug(str)
        else:
            self.pub_depth_control_onOff.publish(0)
            str = "Switch Depth Control OFF"
            rospy.logdebug(str)
            
    def switchPitchOnOff(self,onOff):
        if onOff ==1:
            self.pub_pitch_control_onOff.publish(1)
            str = "Switch Pitch Control ON"
            rospy.logdebug(str)
        else:
            self.pub_pitch_control_onOff.publish(0)
            str = "Switch Pitch Control OFF"
            rospy.logdebug(str)
    #################################################
    # Getter methods    
    ############# these might change depending on which compass is being used...
    ############# could I create PNI compass class? etc that could be instantiated in constructor of this class?
    ############# or is that getting too complicated...?    
    def getDepthandpitchMPC(self):
        return self.__depthandpitchMPC
    
    def getHeading(self):
        return self.__compass.heading
    
    def getRoll(self):
        return self.__compass.roll
    
    def getPitch(self):
        return self.__compass.pitch_filt
    
    def getTemperature(self):
        return self.__compass.temperature
    
    def getDepth(self):
        return self.__compass.depth_filt
    
    # magnetometer
    def getM(self):
        return self.__compass.m
    
    def getMx(self):
        return self.__compass.mx
    
    def getMy(self):
        return self.__compass.my
    
    def getMz(self):
        return self.__compass.mz
    
    # accelerometer
    def getA(self):
        return self.__compass.a
    
    def getAx(self):
        return self.__compass.ax
    
    def getAy(self):
        return self.__compass.ay
    
    def getAz(self):
        return self.__compass.az

    # get position values
    def getX(self):
        return self.__position.X

    def getY(self):
        return self.__position.Y
    
    def getGPSValidFix(self):
        return self.__position.ValidGPSfix    
               
    def getAltitude(self):
        return self.__altitude
    
    def getSonarRange(self):
        return self.__sonar_range
    
    def getBackSeatErrorFlag(self):
        return self.__back_seat_flag
    
    def getVoltage(self):
        return self.__motor_voltage
    
    def getT0rpm(self):
        return self.__T0rpm
    
    def getT1rpm(self):
        return self.__T1rpm
    
    def getT2rpm(self):
        return self.__T2rpm
    
    def getT3rpm(self):
        return self.__T3rpm
    
    def getThrusterStatus(self):
        return self.__TSL_status
    
    def getTailStatus(self):
        return self.__tail_status
        
    def getAltimeterStatus(self):
        return self.__altimeter_status
    
    def getGPSStatus(self):
        return self.__gps_status
    
    def getCompassStatus(self):
        return self.__compass_status
    
    def getCS_b(self):
        return self.__CS_b
    
    def getCS_c(self):
        return self.__CS_c
    
    def getCS_d(self):
        return self.__CS_d
    
    def getCS_e(self):
        return self.__CS_e
    
    def getPropRPM(self):
        return self.__PropRPM
    
    def getHeadingError(self):
        return self.__heading_error     

    #################################################
    #Navigation commands
    #################################################

    def loadWaypoints(self,pathAndFile):
        global longitude
        global latitude

        #Load Waypoints from File
        try:
            f=open(pathAndFile,'r')
            lines=f.readlines()	
            NosLines=len(lines)
            StartLine=-99999
            EndLine=-99999
            #Search for line detailing start of coordinates
            for i in xrange (0,NosLines):
                l=re.search('<coordinates>',lines[i])		
                if l!=None:
                    StartLine=i+1


            for i in xrange (0,NosLines):
                l=re.search('</coordinates>',lines[i])		
                if l!=None:
                    EndLine=i-1



            waypoints=lines[StartLine]

            NosWaypoints=len(waypoints)

            p = re.compile('[-]*\d+.\d+')    
            data=p.findall(waypoints)
            NosWaypoints=len(data)/2

            longitude=numpy.zeros(NosWaypoints)
            latitude=numpy.zeros(NosWaypoints)



            for i in xrange (0,NosWaypoints):
                longitude[i]=float(data[2*i]) # east west
                latitude[i]=float(data[2*i+1]) #north south

            Load=1
        except IOError as e:
            Load=0
            longitude=0
            latitude=0      
            rospy.logerr("Waypoint File Not Found") 
            
        return longitude, latitude, Load




    def distanceandbearingTwoLatLong(self,lat1,lat2,lon1,lon2): #returns bearing between two locations in lat/long in degrees.
        # Code from http://www.movable-type.co.uk/scripts/latlong.html
        R = 6371000 #Radius of the earth in m
        dLat = math.radians(lat2-lat1)
        dLon = math.radians(lon2-lon1)
        lat1 = math.radians(lat1)
        lat2 = math.radians(lat2)
        y = math.sin(dLon) * math.cos(lat2)
        x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dLon);
        brng = math.atan2(y, x)

        a = math.sin(dLat/2)*math.sin(dLat/2)+math.sin(dLon/2)*math.sin(dLon/2)*math.cos(lat1)*math.cos(lat2)
        c = 2*math.atan2(math.sqrt(a),math.sqrt(1-a)) 
        d = R*c
        X=d*math.sin(brng)
        Y=d*math.cos(brng)
        brng= math.degrees(brng)
        return X,Y,R,brng

    def latLongFromRangeAndBearing(self,lat1,lon1,d,brng): #returns new lat/long from an initial point with a range (m) and bearing (deg)
    # Code from http://www.movable-type.co.uk/scripts/latlong.html
        R = 6371000 #Radius of the earth in m
        brng=math.radians(brng)
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.asin(math.sin(lat1)*math.cos(d/R) +math.cos(lat1)*math.sin(d/R)*math.cos(brng))
        lon2 = lon1 + math.atan2(math.sin(brng)*math.sin(d/R)*math.cos(lat1), math.cos(d/R)-math.sin(lat1)*math.sin(lat2))
        lat2=math.degrees(lat2)
        lon2=math.degrees(lon2)
        return lat2,lon2
    
    def camera(self, cam, rec, filename):
        self.pub_camera.publish(camera = cam, record = rec, filename = filename)

    #################################################
    # Callbacks
    #################################################
    def callback_depthandpitchMPC(self, data):
        self.__depthandpitchMPC = data

    def callback_compass(self, compass_data):
#           if compass_data.depth > 15:
#                str = "Depth exceeded 15m - aborting mission"
#                rospy.logfatal(str)
#                self.stop()
        self.__compass = compass_data
    
    def callback_position(self, position):
        self.__position = position   

    def callback_altitude(self, altitude):
        self.__altitude = altitude.altitude_filt
           
    def callback_sonar_range(self, sonar):
        self.__sonar_range = sonar.TargetRange
           
    def callback_back_seat_flag(self, back_seat_flag):
        self.__back_seat_flag = back_seat_flag.data
           
    def callback_tsl_feedback(self, tsl):
        self.__motor_voltage = tsl.voltage
        self.__T0rpm = tsl.speed0
        self.__T1rpm = tsl.speed1
        self.__T2rpm = tsl.speed2
        self.__T3rpm = tsl.speed3
           
    def callback_tail_feedback(self, tail_feedback):
        self.__CS_b = tail_feedback.b
        self.__CS_c = tail_feedback.c
        self.__CS_d = tail_feedback.d
        self.__CS_e = tail_feedback.e
        self.__PropRPM = tail_feedback.rpm
           
    def callback_heading_control(self, heading_control):
        self.__heading_error = heading_control.error
              
           
    def callback_status(self, status):
        if status.nodeID == 1:
            #TSL_board:
            self.__TSL_status = status.status
            return
        elif status.nodeID == 2:
            self.__tail_status = status.status
            return
        elif status.nodeID == 3:
            self.__altimeter_status = status.status
            return
        elif status.nodeID == 4:
            self.__gps_status = status.status
            return
        elif status.nodeID == 5:
            self.__compass_status = status.status
            return
