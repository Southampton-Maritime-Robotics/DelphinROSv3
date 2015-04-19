#!/usr/bin/env python
# creates an XSensDriver object
# 
import rospy
from xsens_driver.mtdevice import XSensDriver
		
if __name__=='__main__':

	rospy.init_node('xsens_driver')
	driver = XSensDriver()
	
	rospy.loginfo("xsens_driver online")
	driver.spin()
