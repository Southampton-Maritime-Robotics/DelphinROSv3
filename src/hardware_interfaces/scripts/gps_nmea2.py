#!/usr/bin/python

"""
A driver for GPS.

The gps sensor works at 1Hz by default. The driver, however, loops twice as fast as the sensor sample rate to clear the serial buffer.

Within one cycle, the sensor provices information via various NMEA message types.

The driver only publishes once the getAll flag is raise.

#######################################################
#Notes
- X corresponds to East
- Y corresponds to North
- A gps fix is either (0=invalid, 1=GPS_fix or 2=Diff._GPS_fix)
  But it may briefly become 6 when the invalid flag is just received

"""
# TODO: veryfy trajectory of (x,y) obtained from this driver. 
# TODO: need to think about how to operate across the UTM zones

from __future__ import division
import rospy
import serial
import time
import threading
from hardware_interfaces.msg import gps
from hardware_interfaces.msg import status
from std_msgs.msg import String
import pynmea2
import utm

class gpsDriver(object):
    def __init__(self):

        # initialise message structure
        self._gpsOut = gps()

        self.nmeaID = {'GGA' : self.gga,
                       'RMC' : self.rmc,
                       'GSA' : self.gsa,
                       'GSV' : self.gsv,
                       'VTG' : self.vtg
        }
        
        # get parameters # TODO Sophia will have to loop at this section
        self._controlRate = 1. # Hz
        lat_ori = rospy.get_param('lat_orig', 50.9567)
        lon_ori = rospy.get_param('long_orig', -1.36735)
        self.dt_status = rospy.get_param('status_timing', 2.)
        
        self.x_UTM_ori, self.y_UTM_ori, _, _ = utm.from_latlon(lat_ori, lon_ori)

        self._r = rospy.Rate(self._controlRate*2)  # Hz

        self._getAll = False

        #Define Publishers
        self.pubStatus = rospy.Publisher('status', status)
        self.pub = rospy.Publisher('gps_out', gps)
        self.pubMissionLog = rospy.Publisher('MissionStrings', String)
        
        self.serialPort = self.openSerialPort()
        rospy.on_shutdown(self.onShutdownEvents)

        self.repeatedlyPublishNodeStatus(nodeID=4, interv=2)

        self.mainloop()

    # ###########################################################
    # Parsing NMEA sentence
    # ###########################################################
    def gga(self, msg):
        # GPS quality indicator (0=invalid; 1=GPS_fix; 2=Diff._GPS_fix)
        self._gpsOut.fix = msg.gps_qual
        # Number of satellites in use [not those in view]
        self._gpsOut.number_of_satelites = int(msg.num_sats)
        # latitude in decimal degree
        self._gpsOut.latitude = msg.latitude
        # longitude in decimal degree
        self._gpsOut.longitude = msg.longitude

        x_UTM_NOW, y_UTM_NOW, _, _ = utm.from_latlon(self._gpsOut.latitude,
                                                     self._gpsOut.longitude)

        self._gpsOut.x = x_UTM_NOW - self.x_UTM_ori
        self._gpsOut.y = y_UTM_NOW - self.y_UTM_ori

        self._getAll = True

    def rmc(self, msg):
        # Speed over ground in m/s
        if self._gpsOut.fix == 0:
            self._gpsOut.speed = 0
        else:
            self._gpsOut.speed = msg.spd_over_grnd*0.51444444444

    def gsa(self, msg):
        pass

    def gsv(self, msg):
        pass

    def vtg(self, msg):
        pass
    # ###########################################################

    # ###########################################################
    # Utilities
    # ###########################################################
    def repeatedlyPublishNodeStatus(self, nodeID, interv):
        def doPub():
            self.pubStatus.publish(nodeID = nodeID, status = True)
            threading.Timer(interv, doPub).start()
        doPub()

    def onShutdownEvents(self):
        self.pubStatus.publish(nodeID = 4, status = False)
        self.serialPort.close()

    def openSerialPort(self):
        serialPort = serial.Serial(port='/dev/usbgps', 
                                   baudrate='4800',
                                   bytesize=8,
                                   parity='N',
                                   stopbits=1,
                                   timeout=0.2) # may need to change timeout if having issues!
        return serialPort
    # ###########################################################

    def mainloop(self):
        self.serialPort.flushInput()
        self.serialPort.flushOutput()
                
        while not rospy.is_shutdown():

            while self.serialPort.inWaiting() > 0:
                if self.serialPort.read(1) == '$':
                    data = self.serialPort.readline()
                    msg = pynmea2.parse(data)
                    self.nmeaID[msg.sentence_type](msg)

            else:
                if self._getAll:
                    self.pub.publish(self._gpsOut)
                    
                self._getAll = False
                self._r.sleep()

if __name__ == '__main__':
    time.sleep(4) #Allow System to come Online
    
    rospy.init_node('gps_sensor')
    
    try:
        gpsDriver()
    except rospy.ROSInterruptException:
        pass

