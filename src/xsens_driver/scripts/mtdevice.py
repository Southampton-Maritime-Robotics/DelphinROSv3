#!/usr/bin/env python
import rospy
from xsens_driver.mtdevice import XSensDriver

def main():
	'''Create a ROS node and instantiate the class.'''

	rospy.init_node('xsens_driver')
	driver = XSensDriver()
	rospy.loginfo("xsens_driver online")
	driver.spin()

if __name__== '__main__':
	main()
	
		
