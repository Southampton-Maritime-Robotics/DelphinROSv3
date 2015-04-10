#!/usr/bin/python
# OUTDATED!
# We now use xsens driver package
# with the new xsens sensor
#
# read and publish sensor values from gyroscope
        
from hardware_interfaces.gyro import *

################################################################        
#     INITIALISE     ###########################################
################################################################
if __name__ == '__main__':
   
    rospy.init_node('Rate_gyro')
    rospy.on_shutdown(shutdown)         #Defining shutdown behaviour  

    pub = rospy.Publisher('rate_gyro', gyro)
    
    port_status = setUpSerial()
    serialPort.flushInput()
    
    str = "Rate Gyro online" 
    rospy.loginfo(str)
    
    readData()
        
    
#        global pubStatus
    
