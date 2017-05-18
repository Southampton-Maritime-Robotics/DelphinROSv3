#!/usr/bin/env python

"""
A driver for xsens sensors (tested on MTi-3).
It is developed based on the driver written by Francis Colas:
http://wiki.ros.org/xsens_driver.

user can set:
- sample rate
- scenario id
- reference location for magnetic field map (lat, lon)
- reset the device
- RestoreFactoryDefaults (this will discard MFM calibration!)

The following convention agrees with REP103 and REP105
# units
- orientation: radian
- angular velocity: rad/s
- acceleration: m/s2 (either free fall or with gravity)
# axis orientation (converted from ENU)
- x: forward
- y: left
- z: up


TODO: Expected IMU to read zero yaw when facing east [to comply with REP103 and REP105].
TODO: used when the sensor fusion algorithm outputs erroneous data or when you need to read out the eMTS device of the devic.
TODO: verify the heading measurement quality with the magnetometer reading. The norm is expected to maintain stable with a magnitude close to 1.
TODO: a seperated code to do ICC
TODO: a seperated code to do gyro bias estimation

Note that:
- ICC is not yet fully supported by xsens, it should be available

"""

from __future__ import division
import rospy
import time
import struct
from sensor_msgs.msg import Imu
from xsens_driver.mtdevice import MTDevice


# request data packets
class XSensDriver(object):
    def __init__(self):
        self._device = '/dev/usbxsens'
        self._ReqPacket = {'linAcc', 'angVel', 'ori'}
        
        self.mt = MTDevice(device=self._device, br=115200, timeout=0.2)
        
        # General: 50; high_mag_dep: 51; dynamic: 52; north_reference: 53; vru_general: 54
        self.mt._scenario_id = chr(rospy.get_param('xsens/filter_profile', 53))
        # if 'd' is selected, higher buadrate may be necessary
        self.mt._ffmt = rospy.get_param('xsens/data_format', 'f')
        # some numbers are not supported by the sensor, e.g., 40 Hz
        self._controlRate = rospy.get_param('loop_timing/state_updaters', 20.)
        lat_ori = rospy.get_param('launch_location/lat_ori', 50.938539)
        lon_ori = rospy.get_param('launch_location/lon_ori', -1.384939)

        # Hz (loop four times as fast as the sensor sample rate)
        self._r = rospy.Rate(self._controlRate*4.)
        self._LatLonAlt = struct.pack('!3d', lat_ori, lon_ori, 0)

        self.deviceReset = True  # perform soft reset
        if self.mt._scenario_id==chr(54) or self.mt._scenario_id==chr(43):
            # To enable Active Heading Stabilisation (AHS)
            # needed only when using vru_general
            self.enableAHS = True
        else:
            self.enableAHS = False

        # To enable the In-run Compass Calibration (ICC)
        # Not yet fully supported by xsens
        # TODO: should be implemented as a separated file instead.
        self.enableICC = False

        self.config()
        self.pubSensorIMU_deg = rospy.Publisher('imu/MTi3_deg', Imu, queue_size=10)
        self.pubSensorIMU = rospy.Publisher('imu/data', Imu, queue_size=10)

        self.mainloop()

    def config(self):
        # Put sensor in configuration mode:
        self.mt.GoToConfig()

        # Set:
        if self.deviceReset:
            rospy.loginfo("Reset to the xsens device")
            self.mt.Reset()
            self.mt.GoToConfig()
        
        self.mt.SetOptionFlags_clear()
        if self.enableAHS:
            self.mt.SetOptionFlags_EnableAHS()
            
        if self.enableICC:
            # Not yet fully supported by xsens.
            self.mt.SetOptionFlags_EnableICC()
            # TODO: should be implemented as a separated file instead
            # with the following procedure using the low level communication:
            # - RepresentativeMotion: start
            # - perform calibration
            # - RepresentativeMotion: stop [is the feedback of 4 good enough?]
            # - RepresentativeMotion: store

        self.mt.SetLatLonAlt(self._LatLonAlt)
        self.mt.SetCurrentScenario(self.mt._scenario_id)
        self.mt.SetOutputConfiguration(self._ReqPacket, self._controlRate)

        # Req:
        rospy.loginfo("Current scenario: %s (id: %d)" % self.mt.ReqCurrentScenario())
        rospy.loginfo("Current location: Lat=%.4f, Lon=%.4f, Alt=%.4f" % self.mt.ReqLatLonAlt())

        # Put sensor in measurement mode:
        self.mt.GoToMeasurement()

    def mainloop(self):
        while not rospy.is_shutdown():
            self.mt.read_measurement()
            if self.mt._getAll:
                self.mt._imu_deg.header.stamp = rospy.Time.now()
                self.mt._imu_deg.header.frame_id = 'base_link'
                self.pubSensorIMU_deg.publish(self.mt._imu_deg)
                self.mt._imu.header.stamp = rospy.Time.now()
                self.mt._imu.header.frame_id = 'base_link'  # does this need to be done every time
                self.pubSensorIMU.publish(self.mt._imu)
            self.mt._getAll = False
            self._r.sleep()

if __name__ == '__main__':
    time.sleep(1)  # allow the system to come online

    rospy.init_node('xsens_driver', anonymous=True)

    # get the xsens object

    try:
        XSensDriver()
    except rospy.ROSInterruptException:
        pass
