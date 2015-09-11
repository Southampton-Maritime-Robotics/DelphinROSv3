#!/usr/bin/python

"""
A node that keeps an eye on the critical nodes and parameters. 

It will raise a terminating flag when at least one of the critical nodes is not working properly or one of the critical parameters goes beyond the threshold.

######################################
#Modifications
30/4/12 Added total water column check and heading error check
26/5/12 Control rate of this node by using rospy.Rate()
27/5/12 Raise a terminating flag when the xsens quiet for longer than 10 seconds

"""

import rospy
import time
from std_msgs.msg import Int8
from std_msgs.msg import String

from delphin2_mission.library_highlevel import library_highlevel

from hardware_interfaces.msg import status

def main(controller):
    rospy.init_node('back_seat_driver')
    pub=rospy.Publisher('back_seat_flag',Int8)
    pubStatus = rospy.Publisher('status', status)
    pubMissionLog = rospy.Publisher('MissionStrings', String)

    controlRate = 10. # [Hz]
    controlPeriod = 1./controlRate
    r = rospy.Rate(controlRate)
    
    # Store Initial Time
    time_zero = time.time()
    
    time.sleep(15) #Allow critical systems to come online.
    
    # Import Limit Parameters from laumch file
    overDepth = rospy.get_param('over-depth')
    overPitch = rospy.get_param('over-pitch')
    overRoll = rospy.get_param('over-roll')
    maxInternalTemp = rospy.get_param('max-internal-temp')
    minMotorVoltage = rospy.get_param('min-motor-voltage')
    missionTimeout = rospy.get_param('mission-timeout')
    missionTimeout = missionTimeout*60 #Mission timeout in minutes therfore convert to seconds
    
    #Initialise BackSeatFlag to zero
    BackSeatFlag=0
    pubMissionLog.publish('Backseat Driver Node Is Active')
    
    headingOld = 0. # [deg]
    timeXsensLastPublish = time.time() # [sec]
    timeXsensDead = 5 # [sec]
    
    pubStatus.publish(nodeID = 11, status = True)
    
    while not rospy.is_shutdown():

        timeRef = time.time()
        
        #Poll System For Any Potential Errors or Status Warnings
    
        #Identify xsens status?
        headingNow = controller.getHeading()
        if headingNow!=headingOld:
            timeXsensLastPublish = time.time()
        if time.time()-timeXsensLastPublish > timeXsensDead:
            BackSeatFlag=1
            str = "xsens did not response for longer than %ss" %(timeXsensDead)
            rospy.logerr(str)
            pub.publish(BackSeatFlag)
            pubMissionLog.publish(str)
            return
        else:
            headingOld = headingNow

        #Identify OverDepth?
        current_depth=controller.getDepth()
        if current_depth > overDepth:
            BackSeatFlag=1
            str = "Current depth %sm > Depth limit of %sm" %(current_depth, overDepth)
            rospy.logerr(str)
            pub.publish(BackSeatFlag)
            pubMissionLog.publish(str)
            return
        
        #Identify OverPitch?
        current_pitch=controller.getPitch()
        if abs(current_pitch) > overPitch:
            BackSeatFlag=1
            str = "Current pitch %sdeg > Pitch limit of %sdeg" %(current_pitch, overPitch)
            rospy.logerr(str)
            pub.publish(BackSeatFlag)
            pubMissionLog.publish(str)
            return
        
        #Identify OverRoll?
        current_roll=controller.getRoll()
        if abs(current_roll) > overRoll:
            BackSeatFlag=1
            str = "Current roll %sdeg > Roll limit of %sdeg" %(current_roll, overRoll)
            rospy.logerr(str)
            pub.publish(BackSeatFlag)
            pubMissionLog.publish(str)
            return

        #Check Internal Pressure Vessel Temperature
        current_temperature=controller.getTemperature()
        if current_temperature>maxInternalTemp:
            BackSeatFlag=1
            str = "Current temperature %sdeg > Temperature limit of %sdeg" %(current_temperature, maxInternalTemp)
            rospy.logerr(str)
            pub.publish(BackSeatFlag)
            pubMissionLog.publish(str)
            return
        
        #Check Motor Voltage
        current_voltage=controller.getVoltage()
        if current_voltage<minMotorVoltage:
            BackSeatFlag=1
            str = "Current voltage %smV < Motor voltage limit of %smV" %(current_voltage, minMotorVoltage)
            rospy.logerr(str)
            pub.publish(BackSeatFlag)
            pubMissionLog.publish(str)
            return
        
        #Check Mission Duration
        current_time=time.time()-time_zero
        if current_time>missionTimeout:
            BackSeatFlag=1
            str = "Current mission time %ss > Mission time limit of %ss" %(current_time, missionTimeout)
            rospy.logerr(str)
            pub.publish(BackSeatFlag)
            pubMissionLog.publish(str)
            return
            
        #Check Motor Control Board Status
        if not controller.getThrusterStatus:
            BackSeatFlag=1
            str = "Thruster control board goes offline"
            rospy.logerr(str)
            pub.publish(BackSeatFlag)
            pubMissionLog.publish(str)
            return

        # Verify and Maintain Control Rate
        timeElapse = time.time()-timeRef
        if timeElapse < controlPeriod:
            r.sleep()
        else:
            str = "BackSeatDriver rate does not meet the desired value of %.2fHz: actual control rate is %.2fHz" %(controlRate,1/timeElapse)
            rospy.logwarn(str)
            
if __name__ == '__main__':
    # Define an instance of highlevelcontrollibrary to pass to all action servers
    lib = library_highlevel()
    main(lib)
