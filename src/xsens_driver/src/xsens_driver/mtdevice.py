from __future__ import division
import rospy
import serial
import struct
import time
import numpy as np
import tf
from mtdef import MID
from mtdef import XDIGroup
from mtdef import Scenarios
from sensor_msgs.msg import Imu
from hardware_interfaces.msg import compass


def print_hex(s):  # TODO: this module is to be removed
    print ":".join("{:02x}".format(ord(c)) for c in s)


class MTDevice(object):
    """XSens MT device communication object."""
    def __init__(self, device, br, timeout=0.02):
        """Open device."""
        self.nodeID = 6
        self.device = serial.Serial(port=device,
                                    baudrate=br,
                                    bytesize=8,
                                    parity='N',
                                    stopbits=1,
                                    timeout=timeout)

        self._timeout = timeout  # serial communication timeout
        self._preamble = '\xFA\xFF'  # preamble for a MTData packet
        self._compass = compass()  # this does not comply with REP103
        self._imu = Imu()
        self._getAll = False
        self._mag_x = 0.
        self._mag_y = 0.
        self._mag_z = 0.

    # ###########################################################
    # Low-level communication
    # ###########################################################
    def write_msg(self, _mid, _data=''):
        """Low-level message sending function."""
        lendat = chr(len(_data))
        packet = self._preamble + _mid + lendat + _data
        packet += chr(0xFF & (-sum([ord(c) for c in packet[1:]])))

        # Flush port and write the message.
        self.device.flushInput()
        self.device.flushOutput()
        self.device.write(packet)

    def write_ack(self, mid, data=''):
        """Low-level message sending function."""
        __nTry_max = 50

        for ite in range(__nTry_max):
            self.write_msg(mid, data)
            data_ack = self.read_msg(chr(ord(mid)+1))

            if not data_ack == -1:
                return data_ack

        rospy.loginfo("Fail to find a valid Mid_ACK: %d" % (ord(mid)+1))
        return -1

    def read_msg(self, _mid_exp):
        """Low-level message receiving function."""
        tStart = time.time()
        _in = self.waitforAndRead(4)
        while (time.time()-tStart) < self._timeout:
            # search for preamble
            if not _in[:-1] == self._preamble+_mid_exp:
                _in = _in[2:] + self.device.read(2)
            else:
                # if the header if found, proceed on reading ID and length
                mid = ord(_in[2])
                length = ord(_in[3])
                # read contents and checksum
                _in += self.waitforAndRead(length+1)

                if length == 0:
                    data = ''
                else:
                    data = _in[4:-1]

                """If checksum is not zero, the packet is invalid."""
                if 0xFF & (sum([ord(c) for c in _in[1:]])):
                    _in = self.waitforAndRead(4)
                    continue  # start over from the while loop

                # return mid and the data
                return data

        else:
            return -1

    def waitforAndRead(self, size):
        """Wait until this many bytes available in the serial buffer."""
        while self.device.inWaiting() < size:
            pass
        else:
            return self.device.read(size)
    # ###########################################################

    # ###########################################################
    # High-level functions
    # ###########################################################
    def SetOutputConfiguration(self, ReqPacket, _controlRate):
        """Configure output data.
        Assume the device is in Config state."""
        # FIXME: At present, only 'ENU' convention works
        # Hence, the coordinate tranformation is done manually.
        __samFreq_hex_str = struct.pack('!H', _controlRate)
        __coorSys_hex_byte = XDIGroup.coorSys['ENU']
        __dataFormat_hex_byte = XDIGroup.dataFormat[self._ffmt]

        OutputData = ''
        for n in ReqPacket:
            OutputData += struct.pack('!H', XDIGroup.Req[n] + __coorSys_hex_byte + __dataFormat_hex_byte) + __samFreq_hex_str

        self.write_ack(MID.OutputConfiguration, OutputData)

    def GoToConfig(self):
        """Place MT device in configuration mode."""
        self.write_ack(MID.GoToConfig)

    def GoToMeasurement(self):
        """Place MT device in measurement mode."""
        self.write_ack(MID.GoToMeasurement)

    def SetCurrentScenario(self, scenario_id):
        """Sets the XKF scenario to use.
        Assume the device is in Config state."""
        self.write_ack(MID.SetCurrentScenario, ('\x00'+scenario_id))

    def ReqCurrentScenario(self):
        """Request the ID of the currently used XKF scenario.
        Assume the device is in Config state."""
        data = self.write_ack(MID.SetCurrentScenario)

        scenario_id = ord(data[1])
        scenario_label = Scenarios.ID2Lable[scenario_id]

        return scenario_label, scenario_id

    def SetLatLonAlt(self, LatLonAlt):
        """Sets the reference location.
        Assume the device is in Config state."""
        self.write_ack(MID.SetLatLonAlt, LatLonAlt)

    def ReqLatLonAlt(self):
        """Request the reference location.
        Assume the device is in Config state."""
        data = self.write_ack(MID.SetLatLonAlt)
        data_interpreted = struct.unpack('!3d', data)
        return data_interpreted

    def Reset(self):
        """Reset the device.
        Assume the device is in Config state."""
        self.write_ack(MID.Reset)

    def RestoreFactoryDefaults(self):
        """Restore MT device configuration to factory defaults.
        Assume the device is in Config state."""
        self.write_ack(MID.RestoreFactoryDef)

    def SetOptionFlags_clear(self):
        """To set optinal flag(s), e.g., AHS and ICC.
        Assume the device is in Config state."""
        # Clear all option flags
        self.write_ack(MID.SetOptionFlags,
                       '\x00\x00\x00\x00\xFF\xFF\xFF\xFF')

    def SetOptionFlags_EnableAHS(self):
        """To enable Active Heading Stabilisation (AHS).
        Assume the device is in Config state."""
        rospy.loginfo("AHS is enabled")
        self.write_ack(MID.SetOptionFlags,
                       '\x00\x00\x00\x10\x00\x00\x00\x00')
            
    def SetOptionFlags_EnableICC(self):
        """To nable In-run Compass Calibration (ICC).
        Assume the device is in Config state."""
        self.write_ack(MID.SetOptionFlags,
                       '\x00\x00\x00\x80\x00\x00\x00\x00')
        rospy.loginfo("In-run Compass Calibration (ICC) is enabled")

    def GyroBiasEstimation(self, stableTime):
        """To calibrate the gyro.
        Assume the device is in Config state."""
        # TODO: this should adapt with the stableTime
        self.write_msg(MID.SetNoRotation,'\x00\x04')
        # TODO: check if this sleep is required or not
        time.sleep(stableTime+1)  # sleep a little longer than the stableTime

    # ###########################################################

    # ###########################################################
    # High-level utility functions
    # ###########################################################
    def read_measurement(self):
        """read MTData2 packet"""
        data = self.read_msg(MID.MTData2)
        if not data == -1:
            self.parse_MTData2(data)
        else:
            self.pubStatus.publish(nodeID = self.nodeID, status = False)
            print data

    def parse_MTData2(self, data):
        """Parse a new MTData2 message"""
        # Functions to parse each type of packet
        # TODO: set status to false if the message parsing failed
        def parse_orientation(data_id, content):
            o = struct.unpack('!'+3*self._ffmt, content)

            """
            The initial yaw reading is zero, when a scenario_id is 54, vru_general.
            Otherwise, the yaw reading indicates 0 when facing east.
            """
            self._compass.heading = -o[2]  # [deg], rotation around z axis
            self._compass.roll = -o[0]     # [deg], rotation around x axis
            self._compass.pitch = o[1]    # [deg], rotation around y axis

            # convert from [deg] to [rad] then [qua]
            q = tf.transformations.quaternion_from_euler(o[0]*np.pi/180,
                                                         o[1]*np.pi/180,
                                                         o[2]*np.pi/180)
            self._imu.orientation.x = q[0]  # [qua]
            self._imu.orientation.y = q[1]  # [qua]
            self._imu.orientation.z = q[2]  # [qua]
            self._imu.orientation.w = q[3]  # [qua]

            self._getAll = True

        def parse_angular_velocity(data_id, content):
            o = struct.unpack('!'+3*self._ffmt, content)
            self._compass.angular_velocity_x = -o[0]*180/np.pi  # [deg/s]
            self._compass.angular_velocity_y = o[1]*180/np.pi  # [deg/s]
            self._compass.angular_velocity_z = -o[2]*180/np.pi  # [deg/s]
            self._imu.angular_velocity.x = o[0]  # [rad/s]
            self._imu.angular_velocity.y = o[1]  # [rad/s]
            self._imu.angular_velocity.z = o[2]  # [rad/s]


        def parse_acceleration(data_id, content):
            o = struct.unpack('!'+3*self._ffmt, content)
            self._compass.ax = -o[0]  # [m/s2]
            self._compass.ay = o[1]  # [m/s2]
            self._compass.az = -o[2]  # [m/s2]
            self._imu.linear_acceleration.x = o[0]  # [m/s2]
            self._imu.linear_acceleration.y = o[1]  # [m/s2]
            self._imu.linear_acceleration.z = o[2]  # [m/s2]

        def parse_magnetic_field_vectors(data_id, content):
            self._mag_x = o[0]
            self._mag_y = o[1]
            self._mag_z = o[2]


        # data object
        while data:
            data_id, size = struct.unpack('!HB', data[:3])
            # determind a data format (double or float)

            content = data[3:3+size]
            data = data[3+size:]
            group = data_id & 0xFFF0

            if group == XDIGroup.OrientationData:
                parse_orientation(data_id, content)
            elif group == XDIGroup.Acceleration or group == XDIGroup.Acceleration_free:
                parse_acceleration(data_id, content)
            elif group == XDIGroup.AngularVelocity:
                parse_angular_velocity(data_id, content)
            elif group == XDIGroup.MagneticFieldVectors:
                parse_magnetic_field_vectors(data_id, content)
