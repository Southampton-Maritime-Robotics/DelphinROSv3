#!/usr/bin/python

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
from hardware_interfaces.msg    import depth
from hardware_interfaces.msg    import position
from hardware_interfaces.msg    import tail_setpoints
from hardware_interfaces.msg    import tsl_setpoints
from hardware_interfaces.msg    import tail_feedback
from hardware_interfaces.msg    import tsl_feedback
from hardware_interfaces.msg    import altitude
from hardware_interfaces.msg    import status
from hardware_interfaces.msg    import camera
from hardware_interfaces.msg    import sonar_data
from hardware_interfaces.msg    import energy_consumed

from lowlevel_controllers.msg   import heading_control

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
        rospy.Subscriber('depth_out', depth, self.callback_depth)                         #compass data
        rospy.Subscriber('position_dead', position, self.callback_position)                     #position (dead reckoning)
        rospy.Subscriber('altimeter_out', altitude, self.callback_altitude)
        rospy.Subscriber('back_seat_flag', Int8, self.callback_back_seat_flag)
        rospy.Subscriber('TSL_feedback', tsl_feedback, self.callback_tsl_feedback)
        rospy.Subscriber('status', status, self.callback_status)
        rospy.Subscriber('tail_output', tail_feedback, self.callback_tail_feedback)
        rospy.Subscriber('Heading_controller_values', heading_control, self.callback_heading_control)
        rospy.Subscriber('sonar_processed', sonar_data, self.callback_sonar_range)
        rospy.Subscriber('EnergyConsumed', energy_consumed, self.EnergyConsumed_callback)
        
        
        #gets parameters from server ####Don't Need to be Here ????????
        self.__maxDepthDemand = rospy.get_param('max-depth-demand')
        self.overdepth = rospy.get_param('over-depth')
        self.overpitch = rospy.get_param('over-pitch')
        
        #initilise empty object variables to hold latest data from subscribers
        ######### might need to set default values in message files
        self.__compass = compass()
        self.__depth = depth()
        self.__position = position()
        self.__altitude = altitude()
        self.__sonar_range = 0
        self.__back_seat_flag = 0 #back_seat_flag is 0 when no errors are present
        
        self.__TSL_status = 0 #initisliased to indicate presence of errors - this will be set to 1 once node has been tested during vehicle initialisation
        self.__tail_status = 0
        self.__altimeter_status = 0
        self.__gps_status = 0
        self.__depth_transducer_status = 0
        self.__xsens_status = 0
        self.__heading_ctrl_status = 0
        self.__depth_ctrl_status = 0
        self.__deadreckoner_status = 0
        self.__logger_status = 0
        self.__backSeatDriver_status = 0
        self.__energyMonitor_status = 0
        
        self.__heading_error=0.0
        self.__altitude=0.0
   
        self.__motor_voltage = 10.0
        self.__T0rpm = 0.0
        self.__T1rpm = 0.0
        self.__T2rpm = 0.0
        self.__T3rpm = 0.0
        self.__CS_b = 0.0
        self.__CS_c = 0.0
        self.__CS_d = 0.0
        self.__CS_e = 0.0
        self.__PropRPS = 0.0
   
    def wakeUp(self,_timeDelay):
        # Called in the mission script, just after creating the libraly object, as to let the object starts working
        # Without this the callback mechanism will not yet start.
        time.sleep(_timeDelay)
        
    # stops vehicle and shuts down ROS
    # may be better as an action....
    def stop(self):
        str="Stop method invoked - ROS will shut down in 5 seconds"
        rospy.logfatal(str)
        self.Mission_pub.publish(str)
        self.setArduinoThrusterHorizontal(0, 0)
        self.setArduinoThrusterVertical(0, 0)
        self.setRearProp(0)
        self.setControlSurfaceAngle(0, 0, 0, 0)
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
        # keep current heading (not current heading demand!)
        self.changeHeadingBy(0)
        self.pub_sway_demand.publish(demand)
    
    # sets a 'demand' for the rear prop (between 0 and 22ish)
    def setRearProp(self, demand):
        self.pub_prop_demand.publish(round(demand))
        
    # set a 'demand' (in degrees) for the rudder angle
    def setRudderAngle(self, demand):
        vertical=tail_setpoints()
        demand = round(demand)
        vertical.cs0 = demand
        vertical.cs1 = demand
        self.pub_tail_setpoints_vertical.publish(vertical)

    # set a 'demand' (in degrees) for the stenplane angle
    def setSternPlaneAngle(self, demand):
        horizontal=tail_setpoints()
        demand = round(demand)
        horizontal.cs0 = demand
        horizontal.cs1 = demand
        self.pub_tail_setpoints_horizontal.publish(horizontal)

    # set a 'demand' (in degrees) for the rudder angle
    def setControlSurfaceAngle(self, b,c,d,e): # (VerUp,HorRight,VerDown,HorLeft)
        vertical   = tail_setpoints()
        horizontal = tail_setpoints()
        vertical.cs0 = round(b)
        vertical.cs1 = round(d)
        horizontal.cs0 = round(c)
        horizontal.cs1 = round(e)
        self.pub_tail_setpoints_vertical.publish(vertical)
        self.pub_tail_setpoints_horizontal.publish(horizontal)

    # manually send tsl setpoint values for horizontal thrusters
    def setArduinoThrusterHorizontal(self, thruster2, thruster3):
        output=tsl_setpoints()
        output.thruster0 = round(thruster2)
        output.thruster1 = round(thruster3)
        self.pub_tsl_heading.publish(output)

    # manually send tsl setpoint values for vertical thrusters
    def setArduinoThrusterVertical(self, thruster0, thruster1):
        output=tsl_setpoints()
        output.thruster0 = round(thruster0)
        output.thruster1 = round(thruster1)
        #publish demand
        self.pub_tsl_depth.publish(output)

    # move to depth 'demand' (metres)
    def setDepth(self, demand):
        if (demand < 0):
            # if demand is zero or less than zero
            str = "Improper depth demand (%sm), turn off depth controller" %(demand)
            rospy.logwarn(str)
        else:
            if demand > self.__maxDepthDemand: # apply a saturation to the demand
                str = "Requested depth %sm > maxDepthDemand (%sm)" %(demand, self.__maxDepthDemand)
                rospy.logwarn(str)
                str = "Setting depth demand %sm" %self.__maxDepthDemand
                rospy.logwarn(str)
                self.pub_depth_demand.publish(self.__maxDepthDemand)
            else:
                self.pub_depth_demand.publish(demand)
            
    def setSpeed(self, demand):
        self.pub_speed.publish(demand)
    
    def setPitch(self, demand):
        if (abs(demand) < self.overpitch):
            self.pub_pitch_demand.publish(demand)
        else: # apply a saturation to the demand
            demand_mod = numpy.sign(demand)*self.overpitch
            self.pub_pitch_demand.publish(demand_mod)

    # move to heading 'demand' (degrees)
    def setHeading(self, demand):
        #publish headingDemand
        cur_heading=self.getHeading()
        self.pub_heading_demand.publish(demand)

    # change heading by 'headingChange' (degrees)
    def changeHeadingBy(self, headingChange):
        #change heading by an amount (headingChange)
        self.setHeading(self.__compass.heading + headingChange)

    # change depth by 'depthChange' (metres)
    def changeDepthBy(self, depthChange):
        #change depth by an amount (depthChange)
        self.setDepth(self.__compass.depth + depthChange)
    
    #################################################
    # Getter methods    
    ############# these might change depending on which compass is being used...
    ############# could I create PNI compass class? etc that could be instantiated in constructor of this class?
    ############# or is that getting too complicated...?
    
    def getHeading(self):
        return self.__compass.heading
    
    def getRoll(self):
        return self.__compass.roll
    
    def getPitch(self):
        return self.__compass.pitch
    
    def getTemperature(self):
        return self.__compass.temperature
    
    def getDepth(self):
        return self.__depth.depth_filt
    
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
    
    def getDepthTransducerStatus(self):
        return self.__depth_transducer_status
    
    def getXsensStatus(self):
        return self.__xsens_status
    
    def getHeadingCtrlStatus(self):
        return self.__heading_ctrl_status
    
    def getDepthCtrlStatus(self):
        return self.__depth_ctrl_status
    
    def getDeadreckonerStatus(self):
        return self.__deadreckoner_status
    
    def getLoggerStatus(self):
        return self.__logger_status
    
    def getBackSeatDriverStatus(self):
        return self.__backSeatDriver_status
        
    def getEnergyMonitorStatus(self):
        return self.__energyMonitor_status
    
    def getCS_b(self):
        return self.__CS_b
    
    def getCS_c(self):
        return self.__CS_c
    
    def getCS_d(self):
        return self.__CS_d
    
    def getCS_e(self):
        return self.__CS_e
    
    def getPropRPS(self):
        return self.__PropRPS
    
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

    def callback_compass(self, compass_data):
        self.__compass = compass_data

    def callback_depth(self, depth_data):
        self.__depth = depth_data
    
    def callback_position(self, position):
        self.__position = position

    def callback_altitude(self, altitude):
        self.__altitude = altitude.altitude_filt
           
    def callback_sonar_range(self, sonar):
        self.__sonar_range = sonar.TargetRange
        
    def EnergyConsumed_callback(self, energy):
        self.__motor_voltage = energy.batteryVol
           
    def callback_back_seat_flag(self, back_seat_flag):
        self.__back_seat_flag = back_seat_flag.data
           
    def callback_tsl_feedback(self, tsl):
        self.__T0rpm = tsl.speed0
        self.__T1rpm = tsl.speed1
        self.__T2rpm = tsl.speed2
        self.__T3rpm = tsl.speed3
           
    def callback_tail_feedback(self, tail_feedback):
        self.__CS_b = tail_feedback.b_fb
        self.__CS_c = tail_feedback.c_fb
        self.__CS_d = tail_feedback.d_fb
        self.__CS_e = tail_feedback.e_fb
        self.__PropRPS = tail_feedback.prop_rps
           
    def callback_heading_control(self, heading_control):
        self.__heading_error = heading_control.error
           
    def callback_status(self, status):
        if status.nodeID == 1:
            self.__TSL_status = status.status
        elif status.nodeID == 2:
            self.__tail_status = status.status
        elif status.nodeID == 3:
            self.__altimeter_status = status.status
        elif status.nodeID == 4:
            self.__gps_status = status.status
        elif status.nodeID == 5:
            self.__depth_transducer_status = status.stat
        elif status.nodeID == 6:
            self.__xsens_status = status.status
        elif status.nodeID == 7:
            self.__heading_ctrl_status = status.status
        elif status.nodeID == 8:
            self.__depth_ctrl_status = status.status
        elif status.nodeID == 9:
            self.__deadreckoner_status = status.status
        elif status.nodeID == 10:
            self.__logger_status = status.status
        elif status.nodeID == 11:
            self.__backSeatDriver_status = status.status
        elif status.nodeID == 12:
            self.__energyMonitor_status = status.status
        return
