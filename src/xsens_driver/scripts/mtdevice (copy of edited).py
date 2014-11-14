#!/usr/bin/env python
import serial
import struct

import sys, getopt, time, glob#, traceback

from mtdef import MID, OutputMode, OutputSettings, MTException, Baudrates, XDIGroup, getName, getMIDName

# Verbose flag for debugging
verbose = False

global serialPort

################################################################
# MTDevice class
################################################################
## XSens MT device communication object.
class MTDevice(object):
	"""XSens MT device communication object."""
	def __init__(self, port, baudrate=115200, timeout=0.001, autoconf=True,
			config_mode=False):
		"""Open device."""
		
		## serial interface to the device
		self.device = serial.Serial(port, baudrate, timeout=timeout,
				writeTimeout=timeout)
		self.device.flushInput()	# flush to make sure the port is ready 
		self.device.flushOutput()	# flush to make sure the port is ready 
		
		## timeout for communication.
		self.timeout = 200*timeout

		print '__init__ MTDevice completed'

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
		
		print 'write msg'
		self.device.write(msg)
		if verbose:
			print "MT: Write message id 0x%02X (%s) with %d data bytes: [%s]"%(mid, getMIDName(mid), length,
							' '.join("%02X"% v for v in data))

	## Low-level MTData receiving function.
	# Take advantage of known message length.
	def read_data_msg(self, buf=bytearray()):
		"""Low-level MTData receiving function.
		Take advantage of known message length."""
		start = time.time()
		if self.length>254:
			totlength = 7 + self.length
		else:
			totlength = 5 + self.length
		while (time.time()-start)<self.timeout:
			while len(buf)<totlength:
				buf.extend(self.device.read(totlength-len(buf)))
			preamble_ind = buf.find(self.header)
			if preamble_ind==-1:	# not found
				# discard unexploitable data
				#sys.stderr.write("MT: discarding (no preamble).\n")
				del buf[:-3]
				continue
			elif preamble_ind:	# found but not at start
				# discard leading bytes
				#sys.stderr.write("MT: discarding (before preamble).\n")
				del buf[:preamble_ind]
				# complete message for checksum
				while len(buf)<totlength:
					buf.extend(self.device.read(totlength-len(buf)))
			if 0xFF&sum(buf[1:]):
				#sys.stderr.write("MT: invalid checksum; discarding data and "\
				#		"waiting for next message.\n")
				del buf[:buf.find(self.header)-2]
				continue
			data = str(buf[-self.length-1:-1])
			del buf[:]
			return data
		else:
			raise MTException("could not find MTData message.")
			
	# XXX this section has been added by kantapon XXX
	## Low-level MTData2 receiving function.
	# Take advantage of known message length.
	def read_data_msg2(self, dataLength):
		# TODO get data length from user
		"""Low-level MTData2 receiving function.
		Take advantage of known message length."""

		print "entering read_data_msg2"
		buf=bytearray() # initialize byte array variable

		HeaderMTData2 = '\xFA\xFF\x36'# [Pre][Bid][MID] of MTData2
		HeaderLength = len(HeaderMTData2) # length of header;
		TotLength = HeaderLength+1+dataLength+1 # [HeaderLength]+[LEN]+[dataLength]+[CS]
		bufLength = 2*TotLength

		start = time.time()		
		self.device.flushInput()		# ensure the data is of this moment
		self.device.flushOutput()		# ensure the data is of this moment
		
		while (time.time()-start)<self.timeout:
			
			new_start = time.time()

			# Makes sure the buffer(of computer not of this driver) has 'size' bytes.
			
			def waitfor(size=1):
				while self.device.inWaiting() < size:
					if time.time()-new_start >= self.timeout:
						raise MTException("timeout waiting for message.")
			
			
			waitfor(bufLength-3)			
			
			while len(buf)<bufLength: # read output and store in buffer
				buf.extend(self.device.read(bufLength-len(buf)))				

			preamble_ind = buf.find(HeaderMTData2)
			if preamble_ind==-1: # header not found
				print "header not found"
				del buf[:-3] 	# delete everything except last three pieces of data
				continue		# go back to "while time.time..." to get a new set of output
			else:				# header found
				length_dummy = int(str(buf[preamble_ind+3])) 	# length of unwanted data
				
				# trim unwanted data at the front end
				del buf[:preamble_ind] 		# junk before [Header]
				length_dummy = int(str(buf[3]))					# length of wanted data
		
				
				# trim unwanted data at the rear end
				del buf[3+1+length_dummy+1:] # [Header]+[LEN]+[Data]+[CS]

			
				if 0xFF&sum(buf[1:]):
					#sys.stderr.write("MT: invalid checksum; discarding data and "\
					#		"waiting for next message.\n")
					print "incorrect checksum"
					del buf[:buf.find(self.header)-2]
					continue

				data = str(buf[-length_dummy-1:-1])
				return data
		else:
			raise MTException("could not find MTData message.")		

	## Low-level message receiving function.
	def read_msg(self):
		"""Low-level message receiving function."""
		start = time.time()
		while (time.time()-start)<self.timeout:
			new_start = time.time()

			# Makes sure the buffer has 'size' bytes.
			def waitfor(size=1):
				while self.device.inWaiting() < size:
					if time.time()-new_start >= self.timeout:
						raise MTException("timeout waiting for message.")

			c = self.device.read()
			while (not c) and ((time.time()-new_start)<self.timeout):
				c = self.device.read()
			print 'after while not c', c
			if not c:
				raise MTException("timeout waiting for message.")
			if ord(c)<>0xFA: # search for a begining of package
				continue # go back to "while (time.time()-start)<self.timeout:"
			# second part of preamble
			waitfor(3) # make sure that there are enought pieces of data to be collected
			if ord(self.device.read())<>0xFF:	# we assume no timeout anymore
				continue # go back to "while (time.time()-start)<self.timeout:"
			
			# read message id and length of message
			#msg = self.device.read(2)
			mid, length = struct.unpack('!BB', self.device.read(2))
			if length==255:	# extended length
				waitfor(2)
				length, = struct.unpack('!H', self.device.read(2))
			
			# read contents and checksum
			waitfor(length+1)
			buf = self.device.read(length+1)
			while (len(buf)<length+1) and ((time.time()-start)<self.timeout):
				buf+= self.device.read(length+1-len(buf))
			if (len(buf)<length+1):
				continue
			checksum = ord(buf[-1])
			data = struct.unpack('!%dB'%length, buf[:-1])
			if mid == MID.Error:
				sys.stderr.write("MT error 0x%02X: %s."%(data[0],
						MID.ErrorCodes[data[0]]))
			if verbose:
				print "MT: Got message id 0x%02X (%s) with %d data bytes: [%s]"%(mid, getMIDName(mid), length,
								' '.join("%02X"% v for v in data))
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
		print 'write msg without ack'
		
#		for tries in range(100):
#			mid_ack, data_ack = self.read_msg()
#			if mid_ack==(mid+1):
#				break
#		else:
#			raise MTException("Ack (0x%X) expected, MID 0x%X received instead"\
#					" (after 100 tries)."%(mid+1, mid_ack))
#			pass
#		return data_ack



	############################################################
	# High-level functions
	############################################################

	## Set output configuration #XXX this section has been added by kantapon XXX
	def SetOutputConfiguration(self,data):
		self.GoToConfig()
		print 'now in config mode'
####		print "printed from the inside of SetOutputConfiguration"
		self.write_ack(MID.OutputConfiguration,data)
		self.GoToMeasurement()
		print 'operating in a measurement state'

	## Place MT device in configuration mode.
	def GoToConfig(self):
		"""Place MT device in configuration mode."""
		print 'before write_ack'
		self.write_ack(MID.GoToConfig)
		print 'before after'

	## Place MT device in measurement mode.
	def GoToMeasurement(self):
		"""Place MT device in measurement mode."""
		self.write_ack(MID.GoToMeasurement)

	############################################################
	# High-level utility functions
	############################################################

	## Read and parse a measurement packet
	def read_measurement(self, mode=None, settings=None):
		# getting data
#		data = self.read_data_msg()
		mid, data = self.read_msg()
####		print "%02x"%mid
####		print "".join("%02x"%ord(n) for n in data)	
		
		if mid==MID.MTData2:
			return self.parse_MTData2(data)
		else:
			raise MTException("unknown data message: mid=0x%02X (%s)."%	(mid, getMIDName(mid)))

	# XXX this section has been added by kantapon XXX
	## Read and parse a measurement packet
	def read_measurement2(self, dataLength):
		# getting data
		print 'before reade_data_msg2'
		data = self.read_data_msg2(dataLength)
		print 'after reade_data_msg2'

		print "".join("%02x"%ord(n) for n in data)
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
####		print output
####		print output.keys()
####		print output['Orientation Data']['Roll']
		return output

################################################################
# Main function
################################################################
def main():
	device = '/dev/usbxsens' #ttyUSB0
	baudrate = 115200

	# open device
	try:
		mt = MTDevice(device, baudrate)
		print 'after init mt'
	except serial.SerialException:
		raise MTException("unable to open %s"%device)

	# make a configuration
	print 'trying to config a device'
	try:
		
		Temp 				= (0x08, 0x10, 0x04, 0x80, 0x04) #XDI_Temperature		 	# 4 bytes
		SampleTimeFine 		= (0x10, 0x60, 0x04, 0x80, 0x04) #XDI_SampleTimeFine		# 4 bytes
		Ori 				= (0x20, 0x30, 0x04, 0x80, 0x0c) #XDI_EulerAngles			# 12 bytes
		Acc_lin				= (0x40, 0x20, 0x04, 0x80, 0x0c) #XDI_Acceleration (linear)	# 12 bytes
		LatLon 				= (0x50, 0x43, 0x04, 0x80, 0x10) #XDI_LatLon				# 16 bytes
		Vel_ang				= (0x80, 0x20, 0x04, 0x80, 0x0c) #XDI_AngularVelocityGr		# 12 bytes
		
		# XXX this section has been modified by kantapon
		# construct an OutputData message from particular packetID
		# e.g. "ReqPacket = {'Temp','Ori'}" to output orientation and acceleration
		ReqPacket = {'Acc_lin','Vel_ang','SampleTimeFine','Ori','LatLon','Temp'} #,'SampleTimeFine','SampleTimeCoarse','Acc','LatLon'
		OutputData = ()
		dataLength = 0 # initialize expected_data_length of MTData2
		for n in ReqPacket:
			dummy = eval(n)
			OutputData += dummy[:-1]
			dataLength += 3+dummy[4] # [packetID]+[packetLEN]+[data]
		
		print 'before config'
		mt.SetOutputConfiguration(OutputData)
		print 'after config'		
		
		# read measurement
		print 'read a measure for 50 iterations'
		for i in range(20): 
			print 'before reading a measurement'
			output = mt.read_measurement2(dataLength)
			print 'after reading a measurement'
			print output
			if output.has_key('Temperature'):
				print ">>>>>>>>>inside tem"
				out_Temp = output['Temperature']
				print out_Temp, out_Temp['Temp']	
				
			if output.has_key('Timestamp'):
				print ">>>>>>>>>inside time"
				out_Time = output['Timestamp']
				print out_Time, out_Time['SampleTimeFine']
				
			if output.has_key('Orientation Data'):
				print ">>>>>>>>>inside ori"
				out_Ori = output['Orientation Data']
				print out_Ori, out_Ori['Yaw'], out_Ori['Roll'], out_Ori['Pitch']
				
			if output.has_key('Acceleration'):
				print ">>>>>>>>>inside acc"
				out_Acc = output['Acceleration']
				print out_Acc, out_Acc['accY'], out_Acc['accX'], out_Acc['accZ']
				
			if output.has_key('Position'):
				print ">>>>>>>>>inside posi"
				out_Pos = output['Position']
				print out_Pos, out_Pos['lat'], out_Pos['lon']
				
			if output.has_key('Angular Velocity'):
				print ">>>>>>>>>inside ang vel"
				out_AngVel = output['Angular Velocity']
				print out_AngVel, out_AngVel['gyrZ'], out_AngVel['gyrX'], out_AngVel['gyrY']

#					print type(output.items())
#					asdf = output.values()
#					print output['Acceleration']['accY']

#				while True: mt.read_measurement2(dataLength)
						
	except KeyboardInterrupt:
		pass

if __name__=='__main__':
	main()

