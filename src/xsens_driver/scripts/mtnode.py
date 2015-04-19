#!/usr/bin/env python
import roslib; roslib.load_manifest('xsens_driver')
import rospy
import select

import mtdevice # XXX this mtdevice.py has been modified by kantapon

from xsens_driver import LatLon # convert lat, lon to X and Y (referenced from original point)

from std_msgs.msg import Header

from xsens_driver.msg import GPS_msg
from xsens_driver.msg import IMU_msg#, GPS_msg

# transform Euler angles or matrix into quaternions
from math import pi, radians
from tf.transformations import quaternion_from_matrix, quaternion_from_euler, identity_matrix

def get_param(name, default):
	try:
		v = rospy.get_param(name)
		rospy.loginfo("Found parameter: %s, value: %s"%(name, str(v)))
	except KeyError:
		v = default
		rospy.logwarn("Cannot find value for parameter: %s, assigning "
				"default: %s"%(name, str(v)))
	return v

class XSensDriver(object):
	
	def __init__(self):
		
		device = get_param('~device', 'auto')
		baudrate = get_param('~baudrate', 0)
		if device=='auto':
			devs = mtdevice.find_devices()
			if devs:
				device, baudrate = devs[0]
				rospy.loginfo("Detected MT device on port %s @ %d bps"%(device,
						baudrate))
			else:
				rospy.logerr("Fatal: could not find proper MT device.")
				rospy.signal_shutdown("Could not find proper MT device.")
				return
		if not baudrate:
			baudrate = mtdevice.find_baudrate(device)
		if not baudrate:
			rospy.logerr("Fatal: could not find proper baudrate.")
			rospy.signal_shutdown("Could not find proper baudrate.")
			return

		rospy.loginfo("MT node interface: %s at %d bd."%(device, baudrate))
		self.mt = mtdevice.MTDevice(device, baudrate)

		self.frame_id = get_param('~frame_id', '/base_imu')
		
		# initialize topics
		self.IMU_pub = rospy.Publisher('IMU_information',IMU_msg)
		self.GPS_pub = rospy.Publisher('GPS_information',GPS_msg)
		
		# reference lat lon in decimal degree 
		# these are needed to be replaced with a home location that specified in delphin2_mission.launch (XXX kantapon XXX)
		self.lat_ori = 50.9346914113
		self.lon_ori = -1.39423859832


	def spin(self):
	
		# XXX this section has been modified by kantapon
		#set an output configuration of the MTi-G-700 (also applicable for other device that support MTData2)
		# construct an OutputData message from particular packetID
		# e.g. "ReqPacket = {'Temp','Ori'}" to output orientation and acceleration

		SampleTimeFine 		= (0x10, 0x60, 0x04, 0x80, 0x04) #XDI_SampleTimeFine		# 4 bytes		
		Temp 				= (0x08, 0x10, 0x04, 0x80, 0x04) #XDI_Temperature		 	# 4 bytes
		Ori 				= (0x20, 0x30, 0x04, 0x80, 0x0c) #XDI_EulerAngles			# 12 bytes
		Vel_ang				= (0x80, 0x20, 0x04, 0x80, 0x0c) #XDI_AngularVelocityGr		# 12 bytes
		Acc_lin				= (0x40, 0x20, 0x04, 0x80, 0x0c) #XDI_Acceleration (linear)	# 12 bytes
		LatLon 				= (0x50, 0x43, 0x04, 0x80, 0x10) #XDI_LatLon					# 16 bytes
		MagField			= (0xC0, 0x20, 0x04, 0x80, 0x0c) #XDI_MagneticField			# 12 bytes
		
		ReqPacket = {'SampleTimeFine','Ori','Acc_lin','LatLon','Vel_ang','MagField'}
		OutputData = ()
		dataLength = 0 # initialize expected_data_length of MTData2
		for n in ReqPacket:
####			print n, type(n)
			dummy = eval(n)
			OutputData += dummy[:-1]
####			print dummy[:-1]
			dataLength += 3+dummy[4] # [packetID]+[packetLEN]+[data]
		
		self.mt.SetOutputConfiguration(OutputData)
		self.mt.ReqOutputConfiguration()
	
		try:
			while not rospy.is_shutdown():
				self.spin_once()
		# Ctrl-C signal interferes with select with the ROS signal handler
		# should be OSError in python 3.?
		except select.error:
			pass

	def spin_once(self):
	
		# create messages and default values
		imu = IMU_msg()		# Time, Temperature, orientation, angular_velocity, linear_acceleration
		pub_temp = False
		gps = GPS_msg() 	# Latitude, Longitude, GPS_status
		pub_GPS = False
		
		has_Time = False
		has_Temp = False
		has_Ori = False
		has_AngVel = False
		has_Acc = False
		has_Posi = False
		has_Mag = False

		# common header
		h = Header()
		h.stamp = rospy.Time.now()
		h.frame_id = self.frame_id

		# get data
		output = self.mt.read_measurement()
		if output.has_key('Timestamp'):
			out_Time = output['Timestamp']
			has_Time = True
		if output.has_key('Temperature'):
			out_Temp = output['Temperature']
			has_Temp = True
		if output.has_key('Orientation Data'):
			out_Ori = output['Orientation Data']
			has_Ori = True
		if output.has_key('Angular Velocity'):
			out_AngVel = output['Angular Velocity']
			has_AngVel = True
		if output.has_key('Acceleration'):
			out_Acc = output['Acceleration']
			has_Acc = True
		if output.has_key('Position'):
			out_Posi = output['Position']
			has_Posi = True
			pub_GPS = True
		if output.has_key('Magnetic'):
			out_Mag = output['Magnetic']
			has_Mag = True
####			print 'here is magx, y and z'
####			print out_Mag

		
		# fill information where it's due # TODO
		if has_Time:
			imu.time = out_Time['SampleTimeFine']
			pub_IMU = True
		if has_Temp:
			imu.temperature = out_Temp['Temp']
			pub_IMU = True
		if has_Ori:
			imu.orientation_roll = out_Ori['Roll']
			imu.orientation_pitch = out_Ori['Pitch']
			imu.orientation_yaw = out_Ori['Yaw']
			pub_IMU = True
		if has_AngVel:
			imu.angular_velocity_x = out_AngVel['gyrX']
			imu.angular_velocity_y = out_AngVel['gyrY']
			imu.angular_velocity_z = out_AngVel['gyrZ']
			pub_IMU = True
		if has_Acc:
			imu.linear_acceleration_x = out_Acc['accX']
			imu.linear_acceleration_y = out_Acc['accY']
			imu.linear_acceleration_z = out_Acc['accZ']
			pub_IMU = True
		if has_Posi:
			gps.latitude = out_Posi['lat']
			gps.longitude = out_Posi['lon']
			[gps.X,gps.Y] = LatLon.main_input([gps.latitude, gps.longitude], [self.lat_ori, self.lon_ori]) # (LatLon_dd, LatLon_ori)
			gps.status = True
		if has_Mag:
			imu.mag_x = out_Mag['magX']
			imu.mag_y = out_Mag['magY']
			imu.mag_z = out_Mag['magZ']
			pub_IMU = True
			
		# publish available information # TODO
		if pub_IMU:
			imu.header = h
			self.IMU_pub.publish(imu)
		if pub_GPS:
			gps.header = h
			self.GPS_pub.publish(gps)
		else: # putlish GPS info always to update the gps status, even in the case that there is no GPS signal found
			gps.header = h
			gps.status = False
			self.GPS_pub.publish(gps)

def main():
	'''Create a ROS node and instantiate the class.'''
	rospy.init_node('xsens_driver')
	driver = XSensDriver()
	driver.spin()


if __name__== '__main__':
	main()
	
		
