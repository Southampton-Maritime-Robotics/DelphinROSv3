#!/usr/bin/python
        
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
    
