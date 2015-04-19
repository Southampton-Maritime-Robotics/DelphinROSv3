"""
## This code is adapted from xsens_driver written by Francis Colas.
Aims to provide a communication with xsens product (mainly MTi-30) using MTData2 message structure.

user adjust 
	-ReqPacket to choose the combination of outputs (this may affect the sampling frequency)
	-the scenario_id to modify the filtering technique used in xsens devices
	
## MODIFICATION ##
25/11/1014 apply configuration to the sensor everytime the driver is run
10/4/2015 control sampling rate with rospy.Rate()
15/4/2015 let users adjust baudrate
		-> open the device with a default baudrate of 115200 
		-> RestoreFactoryDefaults 
		-> close devide
		-> open the device with a prefered baudrate

"""

import serial
import struct
import select
import rospy

import sys, time

from mtdef import MID, MTException, Baudrates, XDIGroup, getName, getMIDName
from hardware_interfaces.msg import compass # compass is a message that is originally used by delphin2.
from custom_def import location, req

################################################################
# set a configuration
################################################################

## filter profile
#available option {general:27, high_mag_dep:28, dynamic:29, low_mag_dep:2a, vru_general:2b}
scenario_id = 0x29

## reference location (for a good heading measurement)
# available options {Boldrewood_Campus, Common_Park, Eastleight_Lake}
LatLonAlt = location.Boldrewood_Campus

## request data packets
# available options {'Acc_lin','FreeAcc_lin','Vel_ang','Ori','Temp'}
ReqPacket = {'req.Acc_lin','req.Vel_ang','req.Ori'}
# need to check the allignment matrix when using FreeAcc_lin

_baudrate = 115200
_controlRate = 40.

################################################################
# MTDevice class
################################################################
## XSens MT device communication object.
class MTDevice(object):
	"""XSens MT device communication object."""
	def __init__(self, port, baudrate=230400, timeout=0.001, autoconf=True,
			config_mode=False):
		"""Open device."""
		
		## serial interface to the device
		self.device = serial.Serial(port, baudrate, timeout=timeout,
				writeTimeout=timeout)
		self.device.flushInput()	# flush to make sure the port is ready 
		self.device.flushOutput()	# flush to make sure the port is ready 
		
		## timeout for communication.
		self.timeout = 10*timeout

	############################################################
	# Low-level communication
	############################################################

	## Low-level message sending function.
	def write_msg(self, mid, data=[]):
		"""Low-level message sending function."""
		length = len(data)
		
		if length>254:
			lendat = [0xFF, 0xFF&length, 0xFF&(length>>8)]
		else:
			lendat = [length]
		packet = [0xFA, 0xFF, mid] + lendat + list(data)
		packet.append(0xFF&(-(sum(packet[1:]))))
		msg = struct.pack('%dB'%len(packet), *packet)

		self.device.write(msg)

	## Low-level MTData2 receiving function.
	# Take advantage of known message length.
	def read_data_msg2(self, dataLength):
		"""Low-level MTData2 receiving function.
		Take advantage of known message length."""

		buf=bytearray() # initialize byte array variable

		HeaderMTData2 = '\xFA\xFF\x36'# [Pre][Bid][MID] of MTData2
		HeaderLength = len(HeaderMTData2) # length of header;
		TotLength = HeaderLength+1+dataLength+1 # [HeaderLength]+[LEN]+[dataLength]+[CS]

		start = time.time()		
		self.device.flushInput()		# ensure the data is of this moment
		self.device.flushOutput()		# ensure the data is of this moment
		
		while (time.time()-start)<self.timeout:
			
			new_start = time.time()

			# Makes sure the buffer(of computer not of this driver) has 'size' bytes.
			def waitfor(size=1):
				while self.device.inWaiting() < size:
					pass
#					if time.time()-new_start >= self.timeout:
#						if not rospy.is_shutdown():
#							raise MTException("timeout waiting for message.")
			
			waitfor(TotLength-len(buf))
			while len(buf)<TotLength: # read output and store in buffer
				buf.extend(self.device.read(TotLength-len(buf)))

			preamble_ind = buf.find(HeaderMTData2)
			if preamble_ind==-1: # header not found
				del buf[:-HeaderLength] 	# delete everything except last three pieces of data
				continue		# go back to "while time.time..." to get a new set of output
			else:				# header found

				# trim unwanted data at the front end
				del buf[:preamble_ind] 		# junk before [Header]
				
				# fill the missing data in the buffer
				waitfor(TotLength-len(buf))
				while len(buf)<TotLength:
					buf.extend(self.device.read(TotLength-len(buf)))
				
				if 0xFF&sum(buf[1:]):
					sys.stderr.write("MT: invalid checksum; discarding data and "\
							"waiting for next message.\n")
					del buf[:-HeaderLength]
					continue

				data = str(buf[HeaderLength+1:-1])
				return data
#		else:
#			raise MTException("could not find MTData message.")
			
	## Low-level message receiving function.
	def read_msg(self):
		"""Low-level message receiving function."""
		__timeout = self.timeout*200 # give extra time for the sensor to response
		start = time.time()
		while (time.time()-start)<__timeout:
			new_start = time.time()

			# Makes sure the buffer has 'size' bytes.
			def waitfor(size=1):
				while self.device.inWaiting() < size:
					if time.time()-new_start >= __timeout:
						raise MTException("timeout waiting for message.")

			c = self.device.read()
			while (not c) and ((time.time()-new_start)<__timeout):
				c = self.device.read()
			if not c:
				raise MTException("timeout waiting for message.")
			if ord(c)<>0xFA: # search for a begining of package
				continue # go back to "while (time.time()-start)<__timeout:"
			# second part of preamble
			waitfor(3) # make sure that there are enought pieces of data to be collected
			if ord(self.device.read())<>0xFF:	# we assume no timeout anymore
				continue # go back to "while (time.time()-start)<__timeout:"
			
			# read message id and length of message
			#msg = self.device.read(2)
			mid, length = struct.unpack('!BB', self.device.read(2))
			if length==255:	# extended length
				waitfor(2)
				length, = struct.unpack('!H', self.device.read(2))
			
			# read contents and checksum
			waitfor(length+1)
			buf = self.device.read(length+1)
			while (len(buf)<length+1) and ((time.time()-start)<__timeout):
				buf+= self.device.read(length+1-len(buf))
			if (len(buf)<length+1):
				continue
			checksum = ord(buf[-1])
			data = struct.unpack('!%dB'%length, buf[:-1])
			if mid == MID.Error:
				sys.stderr.write("MT error 0x%02X: %s."%(data[0],
						MID.ErrorCodes[data[0]]))
			if 0xFF&sum(data, 0xFF+mid+length+checksum):
				sys.stderr.write("invalid checksum; discarding data and "\
						"waiting for next message.\n")
				continue
			return (mid, buf[:-1])
		else:
			raise MTException("could not find message.")

	## Send a message and read confirmation
	def write_ack(self, mid, data=[]):
		"""Send a message a read confirmation."""
		self.write_msg(mid, data)
		
		for tries in range(100):
			mid_ack, data_ack = self.read_msg()
			if mid_ack==(mid+1):
				break
		else:
			raise MTException("Ack (0x%X) expected, MID 0x%X received instead"\
					" (after 100 tries)."%(mid+1, mid_ack))
			pass
		return data_ack	

	############################################################
	# High-level functions	############################################################

	# apply the configuration to the MT device
	def SetOutputConfiguration(self,data):
		self.GoToConfig()
		self.write_ack(MID.OutputConfiguration,data)
		self.GoToMeasurement()
		
	## Place MT device in configuration mode.
	def GoToConfig(self):
		"""Place MT device in configuration mode."""
		self.write_ack(MID.GoToConfig)

	## Place MT device in measurement mode.
	def GoToMeasurement(self):
		"""Place MT device in measurement mode."""
		self.write_ack(MID.GoToMeasurement)
		
	def ReqCurrentScenario(self):
		"""Request the ID of the currently used XKF scenario.
		Assume the device is in Config state."""
		data = self.write_ack(MID.SetCurrentScenario)
		## current XKF id
		self.scenario_id, = struct.unpack('!H', data)
		try:
			scenarios = self.scenarios
		except AttributeError:
			scenarios = self.ReqAvailableScenarios()
		for t, _, label in scenarios:
			if t==self.scenario_id:
				## current XKF label
				self.scenario_label = label
				break
		else:
			self.scenario_label = ""
		return self.scenario_id, self.scenario_label

	def SetCurrentScenario(self, scenario_id):
		"""Sets the XKF scenario to use.
		Assume the device is in Config state."""
		self.GoToConfig()
		self.write_ack(MID.SetCurrentScenario, (0x00, scenario_id&0xFF))
		self.GoToMeasurement()	
		
	def SetLatLonAlt(self, LatLonAlt):
		"""Sets the reference location.
		Assume the device is in Config state."""
		self.GoToConfig()
		self.write_ack(MID.SetLatLonAlt, LatLonAlt)
		self.GoToMeasurement()
		
	def SetBaudrate(self,new_baudrate):
		self.GoToConfig()
		brid = Baudrates.get_BRID(new_baudrate)
		self.write_ack(MID.SetBaudrate,(brid,))
		self.device.baudrate = new_baudrate
		time.sleep(0.01)

	def ReqLatLonAlt(self):
		"""Request the reference location.
		Assume the device is in Config state."""
		data = self.write_ack(MID.SetLatLonAlt)
		return "".join("%02x"%ord(n) for n in data)

	def ReqAvailableScenarios(self):
		"""Request the available XKF scenarios on the device.
		Assume the device is in Config state."""
		scenarios_dat = self.write_ack(MID.ReqAvailableScenarios)
		scenarios = []
		try:
			for i in range(len(scenarios_dat)/22):
				scenario_type, version, label =\
						struct.unpack('!BB20s', scenarios_dat[22*i:22*(i+1)])
				scenarios.append((scenario_type, version, label.strip()))
			## available XKF scenarios
			self.scenarios = scenarios
		except struct.error:
			raise MTException("could not parse the available XKF scenarios.")
		return scenarios
		
	def ReqBaudrate(self):
		"""Request the baudrate.
		Assume the device is in Config state."""
		data = self.write_ack(MID.SetBaudrate)
		return "".join("%02x"%ord(n) for n in data)

	def RestoreFactoryDefaults(self):
		"""Restore MT device configuration to factory defaults (soft version).
		"""
		self.GoToConfig()
		self.write_ack(MID.RestoreFactoryDef)
		self.GoToMeasurement()


	############################################################
	# High-level utility functions	############################################################

	def read_measurement2(self, dataLength):
		# get data
		data = self.read_data_msg2(dataLength)
		return self.parse_MTData2(data)

	## Parse a new MTData2 message
	def parse_MTData2(self, data):
		# Functions to parse each type of packet
		def parse_temperature(data_id, content, ffmt):
			o = {}
			if (data_id&0x00F0) == 0x10:	# Temperature
				o['Temp'], = struct.unpack('!'+ffmt, content)
			else:
				raise MTException("unknown packet: 0x%04X."%data_id)
			return o
		def parse_timestamp(data_id, content, ffmt):
			o = {}
			if (data_id&0x00F0) == 0x10:	# UTC Time
				o['ns'], o['Year'], o['Month'], o['Day'], o['Hour'],\
						o['Minute'], o['Second'], o['Flags'] =\
						struct.unpack('!LHBBBBBB', content)
			elif (data_id&0x00F0) == 0x20:	# Packet Counter
				o['PacketCounter'], = struct.unpack('!H', content)
			elif (data_id&0x00F0) == 0x30:	# Integer Time of Week
				o['TimeOfWeek'], = struct.unpack('!L', content)
			elif (data_id&0x00F0) == 0x40:	# GPS Age
				o['gpsAge'], = struct.unpack('!B', content)
			elif (data_id&0x00F0) == 0x50:	# Pressure Age
				o['pressureAge'], = struct.unpack('!B', content)
			elif (data_id&0x00F0) == 0x60:	# Sample Time Fine
				o['SampleTimeFine'], = struct.unpack('!L', content)
			elif (data_id&0x00F0) == 0x70:	# Sample Time Coarse
				o['SampleTimeCoarse'], = struct.unpack('!L', content)
			elif (data_id&0x00F0) == 0x80:	# Frame Range
				o['startFrame'], o['endFrame'] = struct.unpack('!HH', content)
			else:
				raise MTException("unknown packet: 0x%04X."%data_id)
			return o
		def parse_orientation_data(data_id, content, ffmt):
			o = {}
			if (data_id&0x00F0) == 0x10:	# Quaternion
				o['Q0'], o['Q1'], o['Q2'], o['Q3'] = struct.unpack('!'+4*ffmt,
						content)
			elif (data_id&0x00F0) == 0x20:	# Rotation Matrix
				o['a'], o['b'], o['c'], o['d'], o['e'], o['f'], o['g'], o['h'],\
						o['i'] = struct.unpack('!'+9*ffmt, content)
			elif (data_id&0x00F0) == 0x30:	# Euler Angles
				o['Roll'], o['Pitch'], o['Yaw'] = struct.unpack('!'+3*ffmt,
						content)
			else:
				raise MTException("unknown packet: 0x%04X."%data_id)
			return o
		def parse_pressure(data_id, content, ffmt):
			o = {}
			if (data_id&0x00F0) == 0x10:	# Baro pressure
				# FIXME is it really U4 as in the doc and not a float/double?
				o['Pressure'], = struct.unpack('!L', content)
			else:
				raise MTException("unknown packet: 0x%04X."%data_id)
			return o
		def parse_acceleration(data_id, content, ffmt):
			o = {}
			if (data_id&0x00F0) == 0x10:	# Delta V
				o['Delta v.x'], o['Delta v.y'], o['Delta v.z'] = \
						struct.unpack('!'+3*ffmt, content)
			elif (data_id&0x00F0) == 0x20:	# Acceleration
				o['accX'], o['accY'], o['accZ'] = \
						struct.unpack('!'+3*ffmt, content)
			elif (data_id&0x00F0) == 0x30:	# Free Acceleration
				o['freeAccX'], o['freeAccY'], o['freeAccZ'] = \
						struct.unpack('!'+3*ffmt, content)
			else:
				raise MTException("unknown packet: 0x%04X."%data_id)
			return o
		def parse_position(data_id, content, ffmt):
			o = {}
			if (data_id&0x00F0) == 0x10:	# Altitude MSL
				o['altMsl'], = struct.unpack('!'+ffmt, content)
			elif (data_id&0x00F0) == 0x20:	# Altitude Ellipsoid
				o['altEllipsoid'], = struct.unpack('!'+ffmt, content)
			elif (data_id&0x00F0) == 0x30:	# Position ECEF
				o['ecefX'], o['ecefY'], o['ecefZ'] = \
						struct.unpack('!'+3*ffmt, content)
			elif (data_id&0x00F0) == 0x40:	# LatLon
				o['lat'], o['lon'] = struct.unpack('!'+2*ffmt, content)
			else:
				raise MTException("unknown packet: 0x%04X."%data_id)
			return o
		def parse_angular_velocity(data_id, content, ffmt):
			o = {}
			# FIXME is it really 802y and 803y as in the doc?
			if (data_id&0x00F0) == 0x20:	# Rate of Turn
				o['gyrX'], o['gyrY'], o['gyrZ'] = \
						struct.unpack('!'+3*ffmt, content)
			elif (data_id&0x00F0) == 0x30:	# Delta Q
				o['Delta q0'], o['Delta q1'], o['Delta q2'], o['Delta q3'] = \
						struct.unpack('!'+4*ffmt, content)
			else:
				raise MTException("unknown packet: 0x%04X."%data_id)
			return o
		def parse_GPS(data_id, content, ffmt):
			o = {}
			if (data_id&0x00F0) == 0x30:	# DOP
				o['iTOW'], o['gDOP'], o['pDOP'], o['tDOP'], o['vDOP'], \
						o['hDOP'], o['nDOP'], o['nDOP'] = \
						struct.unpack('!LHHHHHHH', content)
			elif (data_id&0x00F0) == 0x40:	# SOL
				o['iTOW'], o['fTOW'], o['Week'], o['gpsFix'], o['Flags'], \
						o['ecefX'], o['ecefY'], o['ecefZ'], o['pAcc'], \
						o['ecefVX'], o['ecefVY'], o['ecefVZ'], o['sAcc'], \
						o['pDOP'], o['numSV'] = \
						struct.unpack('!LlhBBlllLlllLHxBx', content)
			elif (data_id&0x00F0) == 0x80:	# Time UTC
				o['iTOW'], o['tAcc'], o['nano'], o['year'], o['month'], \
						o['day'], o['hour'], o['min'], o['sec'], o['valid'] = \
						struct.unpack('!LLlHBBBBBB', content)
			elif (data_id&0x00F0) == 0xA0:	# SV Info
				o['iTOW'], o['numCh'] = struct.unpack('!LBxx', content[:8])
				channels = []
				for i in range(numCh):
					ch['chn'], ch['svid'], ch['flags'], ch['quality'], \
							ch['cno'], ch['elev'], ch['azim'], ch['prRes'] = \
							struct.unpack('!BBBBBbhl', content[8+12*i:20+12*i])
					channels.append(ch)
				o['channels'] = channels
			else:
				raise MTException("unknown packet: 0x%04X."%data_id)
			return o
		def parse_SCR(data_id, content, ffmt):
			o = {}
			if (data_id&0x00F0) == 0x10:	# ACC+GYR+MAG+Temperature
				o['accX'], o['accY'], o['accZ'], o['gyrX'], o['gyrY'], \
						o['gyrZ'], o['magX'], o['magY'], o['magZ'], o['Temp']=\
						struct.unpack("!9Hh", content)
			elif (data_id&0x00F0) == 0x20:	# Gyro Temperature
				o['tempGyrX'], o['tempGyrY'], o['tempGyrZ'] = \
						struct.unpack("!hhh", content)
			else:
				raise MTException("unknown packet: 0x%04X."%data_id)
			return o
		def parse_analog_in(data_id, content, ffmt):
			o = {}
			if (data_id&0x00F0) == 0x10:	# Analog In 1
				o['analogIn1'], = struct.unpack("!H", content)
			elif (data_id&0x00F0) == 0x20:	# Analog In 2
				o['analogIn2'], = struct.unpack("!H", content)
			else:
				raise MTException("unknown packet: 0x%04X."%data_id)
			return o
		def parse_magnetic(data_id, content, ffmt):
			o = {}
			if (data_id&0x00F0) == 0x20:	# Magnetic Field
				o['magX'], o['magY'], o['magZ'] = \
						struct.unpack("!3"+ffmt, content)
			else:
				raise MTException("unknown packet: 0x%04X."%data_id)
			return o
		def parse_velocity(data_id, content, ffmt):
			o = {}
			if (data_id&0x00F0) == 0x10:	# Velocity XYZ
				o['velX'], o['velY'], o['velZ'] = \
						struct.unpack("!3"+ffmt, content)
			else:
				raise MTException("unknown packet: 0x%04X."%data_id)
			return o
		def parse_status(data_id, content, ffmt):
			o = {}
			if (data_id&0x00F0) == 0x10:	# Status Byte
				o['StatusByte'], = struct.unpack("!B", content)
			elif (data_id&0x00F0) == 0x20:	# Status Word
				o['StatusWord'], = struct.unpack("!L", content)
			elif (data_id&0x00F0) == 0x40:	# RSSI
				o['RSSI'], = struct.unpack("!b", content)
			else:
				raise MTException("unknown packet: 0x%04X."%data_id)
			return o

		# data object
		output = {}
		while data:
			try:
				data_id, size = struct.unpack('!HB', data[:3])
				if (data_id&0x0003) == 0x3:
					ffmt = 'd'
				elif (data_id&0x0003) == 0x0:
					ffmt = 'f'
				else:
					raise MTException("fixed point precision not supported.")
				content = data[3:3+size]
				data = data[3+size:]
				group = data_id&0xFF00
				if group == XDIGroup.Temperature:
					output['Temperature'] = parse_temperature(data_id, content, ffmt)
				elif group == XDIGroup.Timestamp:
					output['Timestamp'] = parse_timestamp(data_id, content, ffmt)
				elif group == XDIGroup.OrientationData:
					output['Orientation Data'] = parse_orientation_data(data_id, content, ffmt)
				elif group == XDIGroup.Pressure:
					output['Pressure'] = parse_pressure(data_id, content, ffmt)
				elif group == XDIGroup.Acceleration:
					output['Acceleration'] = parse_acceleration(data_id, content, ffmt)
				elif group == XDIGroup.Position:
					output['Position'] = parse_position(data_id, content, ffmt)
				elif group == XDIGroup.AngularVelocity:
					output['Angular Velocity'] = parse_angular_velocity(data_id, content, ffmt)
				elif group == XDIGroup.GPS:
					output['GPS'] = parse_GPS(data_id, content, ffmt)
				elif group == XDIGroup.SensorComponentReadout:
					output['SCR'] = parse_SCR(data_id, content, ffmt)
				elif group == XDIGroup.AnalogIn:
					output['Analog In'] = parse_analog_in(data_id, content, ffmt)
				elif group == XDIGroup.Magnetic:
					output['Magnetic'] = parse_magnetic(data_id, content, ffmt)
				elif group == XDIGroup.Velocity:
					output['Velocity'] = parse_velocity(data_id, content, ffmt)
				elif group == XDIGroup.Status:
					output['Status'] = parse_status(data_id, content, ffmt)
				else:
					raise MTException("unknown XDI group: 0x%04X."%group)
			except struct.error, e:
				raise MTException("couldn't parse MTData2 message.")

		return output

##############################
# # # XSensDriver object # # #
##############################
class XSensDriver(object):
	
	def __init__(self):
		
		device = '/dev/usbxsens' #ttyUSB0
		self.__baudrate = _baudrate # get the baudrate defined globally at the top
		
		# open device with a default baudrate
		try:
			self.mt = MTDevice(device, 115200)
		except serial.SerialException:
			raise MTException("unable to open %s"%device)
		self.mt.RestoreFactoryDefaults() # restore all the setting to factory defaults. 
		# By this point, sensor baudrate is definitely 115200.
		# If the sensor baudrate does not match the desired value, Set the baudrate and reopen the sensor.
		if self.__baudrate != 115200:
			self.mt.SetBaudrate(self.__baudrate) # apply the preferred baudrate to the sensor
			# re-open device after applying the new baudrate
			try:
				self.mt.device.close()
				self.mt = MTDevice(device, self.__baudrate)
			except serial.SerialException:
				raise MTException("unable to open %s"%device)
			
		# initialize topics
		self.COMPASS_pub = rospy.Publisher('compass_out',compass)
		#set up subscribers
		rospy.Subscriber('compass_old', compass, self.callback_COMPASS_msg)
		# create messages and default values
		self.com = compass()
		
		self.depth = 0.0
		self.depth_filt = 0.0
		self.depth_der = 0.0

	def spin(self):

#		controlRate = 100. # Hz
		r = rospy.Rate(_controlRate)
		controlPeriod = 1./_controlRate

		# make a configuration
		try:
		
			self.mt.SetCurrentScenario(scenario_id)
			self.mt.SetLatLonAlt(LatLonAlt)
			
			# construct a configuration vector that consists of requested packets
			OutputData = ()
			self.dataLength = 0 # initialize expected_data_length of MTData2
			for n in ReqPacket:
				dummy = eval(n)
				OutputData += dummy[:-1]
				self.dataLength += 3+dummy[4] # [packetID]+[packetLEN]+[data]
		
			self.mt.SetOutputConfiguration(OutputData)
			
			# request a current configuration on the xsens
			self.mt.GoToConfig()
			print "Current scenario: %s (id: %d)"%self.mt.ReqCurrentScenario()[::-1]
			
			brid = self.mt.ReqBaudrate()
			print "Current baudrate: %s (id: %s)"%(Baudrates.get_BR(int(brid)),brid)
#			print "Current location : (id: %s)"%self.mt.ReqLatLonAlt()
			self.mt.GoToMeasurement()
			
			try:
				while not rospy.is_shutdown():
					timeRef = time.time()
					self.spin_once()
					timeElapse = time.time()-timeRef
					if timeElapse < controlPeriod:
						r.sleep()
					else:
						str = "xsens rate does not meet the desired value of %.2fHz: actual control rate is %.2fHz" %(_controlRate,1/timeElapse) 
						rospy.logwarn(str)

			# Ctrl-C signal interferes with select with the ROS signal handler
			# should be OSError in python 3.?
			except select.error:
				pass

		except KeyboardInterrupt:
			pass

	def callback_COMPASS_msg(self,data):
		self.depth = data.depth
		self.depth_filt = data.depth_filt
		self.depth_der = data.depth_der

	def spin_once(self):
		
		has_Temp = False
		has_Ori = False
		has_AngVel = False
		has_Acc = False
		pub_IMU = False

		# get data and split it into particular variables
		output = self.mt.read_measurement2(self.dataLength)

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
		
		# fill information where it's due #
		if has_Temp:
			self.com.temperature = out_Temp['Temp']
			pub_IMU = True
		if has_Ori:
			# compensate a heading offset due to a different in the reference frame
			out_Ori['Yaw'] = out_Ori['Yaw'] + 90
			# remap heading from [-pi,pi] to [0,2pi]
			if out_Ori['Yaw']>0:
				out_Ori['Yaw'] = 360-out_Ori['Yaw']
			else:
				out_Ori['Yaw'] = -out_Ori['Yaw']			
			# b-frame convension base on the orientation of sensor mounted on delphin2	
			self.com.roll = -out_Ori['Roll']
			self.com.pitch = out_Ori['Pitch']
			self.com.heading = out_Ori['Yaw']
			pub_IMU = True
		if has_AngVel:
			# b-frame convension base on the orientation of sensor mounted on delphin2
			self.com.angular_velocity_x = -out_AngVel['gyrX']
			self.com.angular_velocity_y = out_AngVel['gyrY']
			self.com.angular_velocity_z = -out_AngVel['gyrZ']
			pub_IMU = True
		if has_Acc: 
			# b-frame convension base on the orientation of sensor mounted on delphin2
			if out_Acc.has_key('freeAccX'):
				self.com.ax = out_Acc['freeAccX']
				self.com.ay = -out_Acc['freeAccY']
				self.com.az = out_Acc['freeAccZ']
			elif out_Acc.has_key('accX'):
				self.com.ax = out_Acc['accX']
				self.com.ay = -out_Acc['accY']
				self.com.az = out_Acc['accZ']
			pub_IMU = True
			
		# publish available information #
		if pub_IMU:
			# add a depth measurement of old device into the new compass_out topic
			self.com.depth = self.depth
			self.com.depth_filt = self.depth_filt
			self.com.depth_der = self.depth_der
			self.COMPASS_pub.publish(self.com)
	
