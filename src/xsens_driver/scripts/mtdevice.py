#!/usr/bin/env python
import rospy

		
if __name__=='__main__':
	rospy.init_node('xsens_driver')
	driver = XSensDriver()
	
	rospy.loginfo("xsens_driver online")
	driver.spin()
