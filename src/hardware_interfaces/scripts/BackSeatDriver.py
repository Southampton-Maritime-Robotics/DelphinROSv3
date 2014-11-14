#!/usr/bin/python
import roslib; roslib.load_manifest('hardware_interfaces')
import rospy
import time
from std_msgs.msg import Int8
from std_msgs.msg import String

import sys
sys.path.append('/home/delphin2/DelphinROSv3/src/delphin2_mission/scripts')
from library_highlevel import library_highlevel



######################################
#Modifications
#30/4/12 Added total water column check and heading error check


def main(controller):
    rospy.init_node('back_seat_driver')
    pub=rospy.Publisher('back_seat_flag',Int8)
    pub2 = rospy.Publisher('MissionStrings', String)
    
    # Store Initial Time
    time_zero = time.time()    
    
    time.sleep(5) #Allow critical systems to come online.
    time_error=time.time() #Error for heading check
    #time_error_water=time.time()
  
    # Import Limit Parameters from laumch file
    overDepth = rospy.get_param('over-depth')
    overPitch = rospy.get_param('over-pitch')    
    overRoll = rospy.get_param('over-roll')     
    maxInternalTemp = rospy.get_param('max-internal-temp')  
    minMotorVoltage = rospy.get_param('min-motor-voltage')
    missionTimeout = rospy.get_param('mission-timeout')
    missionTimeout = missionTimeout*60          #Mission timeout in minutes therfore convert to seconds
    
    
    #Initialise BackSeatFlag to zero
    BackSeatFlag=0
    pub2.publish('Backseat Driver Node Is Active')
    
    while not rospy.is_shutdown():
        time.sleep(0.01)
        time_elapsed=time.time()-time_zero
    
    
        #Poll System For Any Potential Errors or Status Warnings
    
        #Identify OverDepth?
        current_depth=controller.getDepth()

        
        if current_depth > overDepth: 
            BackSeatFlag=1
            str = "Current depth %sm > Depth limit of %sm" %(current_depth, overDepth) 
            rospy.logerr(str)
            pub.publish(BackSeatFlag)
            pub2.publish(str)
            return
        
        #Identify OverPitch?
        current_pitch=controller.getPitch()
        
        if abs(current_pitch) > overPitch: 
            BackSeatFlag=1
            str = "Current pitch %sdeg > Pitch limit of %sdeg" %(current_pitch, overPitch) 
            rospy.logerr(str)
            pub.publish(BackSeatFlag)
            pub2.publish(str)
            return
        
        #Identify OverRoll?
        current_roll=controller.getRoll()
        
        if abs(current_roll) > overRoll: 
            BackSeatFlag=1
            str = "Current roll %sdeg > Roll limit of %sdeg" %(current_roll, overRoll) 
            rospy.logerr(str)
            pub.publish(BackSeatFlag)
            pub2.publish(str)
            return        

        #Check Internal Pressure Vessel Temperature
        current_temperature=controller.getTemperature()
        
        if current_temperature>maxInternalTemp: 
            BackSeatFlag=1
            str = "Current temperature %sdeg > Temperature limit of %sdeg" %(current_temperature, maxInternalTemp) 
            rospy.logerr(str)
            pub.publish(BackSeatFlag)
            pub2.publish(str)
            return   
        
#        #Check Motor Voltage
#        current_voltage=controller.getVoltage()
#        
#        if current_voltage<minMotorVoltage: 
#            BackSeatFlag=1
#            str = "Current voltage %smV < Motor voltage limit of %smV" %(current_voltage, minMotorVoltage) 
#            rospy.logerr(str)
#            pub.publish(BackSeatFlag)
#            pub2.publish(str)
#            return                 
        
        #Check Mission Duration
        current_time=time.time()-time_zero
        
        if current_time>missionTimeout: 
            BackSeatFlag=1
            str = "Current mission time %ss > Mission time limit of %ss" %(current_time, missionTimeout) 
            rospy.logerr(str)
            pub.publish(BackSeatFlag)
            pub2.publish(str)
            return   
        
        #Total water column depth - Only activated 30s after start of mission
        
   
        
        

       # if current_total_water_column > 1:        #if current heading error less than error magnitude
        #    time_error_water=time.time()                       #reset error time

#        if (time.time()-time_error_water)>2 and current_time>30: 
#            BackSeatFlag=1
#            str = "Total water depth of < 1m has been observed for 2s and mission time of %s>30s" %(current_time) 
#            rospy.logerr(str)
#            pub.publish(BackSeatFlag)
#            pub2.publish(str)
#            return       
#        
#        #Monitor long term heading error
#        
#        current_heading_error=controller.getHeadingError()
#        error_duration=60
#        error_magnitude=20
#        
#        if current_heading_error<error_magnitude:        #if current heading error less than error magnitude
#            time_error=time.time()                       #reset error time
#            
#        if time.time()-time_error > error_duration:      #error duration has exceeded pre-set error_duration
#            BackSeatFlag=1
#            str = "Heading error has exceeded %s deg for a duration of greater than %s" %(error_magnitude, error_duration) 
#            rospy.logerr(str)
#            pub.publish(BackSeatFlag)
#            pub2.publish(str)
#            return         
#        
        
        
        
 
if __name__ == '__main__':
    # Define an instance of highlevelcontrollibrary to pass to all action servers
    lib = library_highlevel()
    main(lib)
    rospy.spin()
