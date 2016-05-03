"""Constant and messages definition for MT communication."""

class location(object):
    info = {## Boldrewood campus: 50.937314, -1.404341, 0
            'Boldrewood_Campus': (0x40, 0x49, 0x77, 0xF9, 0xE7, 0xB8, 0x0A, 0x9E, 
                                  0xBF, 0xF6, 0x78, 0x2E, 0x44, 0xB6, 0xE9, 0x36, 
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
            ## Common Park: 50.934014, -1.407368, 0
            'Common_Park': (      0x40, 0x49, 0x77, 0x8D, 0xC5, 0x50, 0x00, 0xC9,
                                  0xBF, 0xF6, 0x84, 0x94, 0x4E, 0xD6, 0xFD, 0xA8, 
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00),        
            ## Eastleight Lake: 50.957127, -1.367413, 0
            'Eastleight_Lake': (  0x40, 0x49, 0x7A, 0x83, 0x23, 0x35, 0x8F, 0x2E,
                                  0xBF, 0xF5, 0xE0, 0xEC, 0x74, 0x32, 0x01, 0x04, 
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00)}
           
    @classmethod
    def get_name(cls, LatLonAlt):
        """Get location name for a given LatLonAlt."""
        for _name, _LatLonAlt in cls.info.items():
            if LatLonAlt == _LatLonAlt:
                return _name
        raise MTException("unsupported location name.")
    @classmethod
    def get_LatLonAlt(cls, name):
        """Get LatLonAlt for a given location name."""
        for _name, _LatLonAlt in cls.info.items():
            if name == _name:
                return _LatLonAlt
        raise MTException("unsupported LatLonAlt.")
    @classmethod
    def get_print(cls, a):
        print a
                            
class req:
    Temp               = (0x08, 0x10, 0x04, 0x80, 0x04) #XDI_Temperature        # 4 bytes
    SampleTimeFine     = (0x10, 0x60, 0x04, 0x80, 0x04) #XDI_SampleTimeFine     # 4 bytes
    Ori                = (0x20, 0x30, 0x04, 0x80, 0x0c) #XDI_EulerAngles        # 12 bytes
    Acc_lin            = (0x40, 0x20, 0x04, 0x80, 0x0c) #XDI_Acceleration       # 12 bytes
    FreeAcc_lin        = (0x40, 0x30, 0x04, 0x80, 0x0c) #XDI_FreeAcceleration   # 12 bytes
    LatLon             = (0x50, 0x43, 0x04, 0x80, 0x10) #XDI_LatLon             # 16 bytes
    Vel_ang            = (0x80, 0x20, 0x04, 0x80, 0x0c) #XDI_AngularVelocityGr  # 12 bytes
    MagField           = (0xC0, 0x20, 0x04, 0x80, 0x0c) #XDI_MagneticField      # 12 bytes

class MID:
    """Values for the message id (MID)"""
    ## Error message, 1 data byte
    Error = 0x42
    ErrorCodes = {
        0x03: "Invalid period",
        0x04: "Invalid message",
        0x1E: "Timer overflow",
        0x20: "Invalid baudrate",
        0x21: "Invalid parameter"}

    # State MID
    ## Wake up procedure
    WakeUp = 0x3E
    ## Switch to config state
    GoToConfig = 0x30
    ## Switch to measurement state
    GoToMeasurement = 0x10
    ## Reset device
    Reset = 0x40
    
    ReqLatLonAlt = 0x6E
    
    # Informational messages
    ## Request device id
    ReqDID = 0x00
    ## DeviceID, 4 bytes: HH HL LH LL
    DeviceID = 0x01
    ## Compatibility for XBus Master users
    InitMT = 0x02
    InitMTResults = 0x03
    ## Request product code in plain text
    ReqProductCode = 0x1C
    ## Product code (max 20 bytes data)
    ProductCode = 0x1D
    ## Request firmware revision
    ReqFWRev = 0x12
    ## Firmware revision, 3 bytes: major minor rev
    FirmwareRev = 0x13
    ## Request data length according to current configuration
    ReqDataLength = 0x0A
    ## Data Length, 2 bytes
    DataLength = 0x0B
    ## Request GPS status (MTi-G only)
    ReqGPSStatus = 0xA6
    ## GPS status (MTi-G only)
    GPSStatus = 0xA7

    # Device specific messages
    ## Baudrate, 1 byte
    SetBaudrate = 0x18
    ## Error mode, 2 bytes, 0000, 0001, 0002, 0003 (default 0001)
    SetErrorMode = 0xDA
    ## Location ID, 2 bytes, arbitrary, default is 0
    SetLocationID = 0x84
    ## Restore factory defaults
    RestoreFactoryDef = 0x0E
    ## Transmit delay (RS485), 2 bytes, number of clock ticks (1/29.4912 MHz)
    SetTransmitDelay = 0xDC

    # Synchronization messages
    ## Synchronization settings (MTi-10/100 series only), N*12 bytes
    SetSyncSettings = 0x2C
    ## SyncIn setting (MTi only), (1+) 2 or 4 bytes depending on request
    SetSyncInSettings = 0xD6
    ## SyncOut setting (MTi/MTi-G only), (1+) 2 or 4 bytes depending on request
    SetSyncOutSettings = 0xD8

    # Configuration messages
    ## Request configuration
    ReqConfiguration = 0x0C
    ## Configuration, 118 bytes
    Configuration = 0x0D
    ## Output configuration (MTi-10/100 series only), N*4 bytes
    OutputConfiguration = 0xC0
    ## Sampling period (MTi/MTi-G only), 2 bytes
    SetPeriod = 0x04
    ## Skip factor (MTi/MTi-G only), 2 bytes
    SetOutputSkipFactor = 0xD4
    ## Object alignment matrix, 9*4 bytes
    SetObjectAlignment = 0xE0
    ## Output mode (MTi/MTi-G only), 2 bytes
    SetOutputMode = 0xD0
    ## Output settings (MTi/MTi-G only), 4 bytes
    SetOutputSettings = 0xD2

    # Data messages
    ## Request MTData message (for 65535 skip factor)
    ReqData = 0x34
    ## Legacy data packet
    MTData = 0x32
    ## Newer data packet (MTi-10/100 series only)
    MTData2 = 0x36

    # XKF Filter messages
    ## Heading (MTi only), 4 bytes
    SetHeading = 0x82
    ## Reset orientation, 2 bytes
    ResetOrientation = 0xA4
    ## Request UTC time from sensor (MTI-G and MTi-10/100 series)
    ReqUTCTime = 0x60
    ## UTC Time (MTI-G and MTi-10/100 series), 12 bytes
    UTCTime = 0x61
    ## Request the available XKF scenarios on the device
    ReqAvailableScenarios = 0x62
    ## Available Scenarios
    AvailableScenarios = 0x63
    ## Current XKF scenario, 2 bytes
    SetCurrentScenario = 0x64
    ## Magnitude of the gravity used for the sensor fusion mechanism, 4 bytes
    SetGravityMagnitude = 0x66
    ## Lever arm of the GPSin sensor coordinates (MTi-G and MTi-700 only), 3*4 bytes
    SetLeverArmGPS = 0x68
    ## Magnetic declination (MTi-G only), 4 bytes
    SetMagneticDeclination = 0x6A
    ## Latitude, Longitude and Altitude for local declination and gravity
    # (MTi-10/100 series only), 24 bytes
    SetLatLonAlt = 0x6E
    ## Processing flags (not on firmware 2.2 or lower for MTi/MTi-g), 1 byte
    SetProcessingFlags = 0x20
    ## Initiate No Rotation procedure (not on MTi-G), 2 bytes
    SetNoRotation = 0x22



def getName(cls, value):
    '''Return the name of the first found member of class cls with given
    value.'''
    for k, v in cls.__dict__.iteritems():
        if v==value:
            return k
    return ''


def getMIDName(mid):
    '''Return the name of a message given the message id.'''
    name = getName(MID, mid)
    if name:
        return name
    if mid&1:
        name = getName(MID, mid-1)
        if name:
            return name+'Ack'
    return 'unknown MID'


class Baudrates(object):
    """Baudrate information and conversion."""
    ## Baudrate mapping between ID and value
    Baudrates = [
        (0x80, 921600),
        (0x0A, 921600),
        (0x00, 460800),
        (0x01, 230400),
        (0x02, 115200),
        (0x03,  76800),
        (0x04,  57600),
        (0x05,  38400),
        (0x06,  28800),
        (0x07,  19200),
        (0x08,  14400),
        (0x09,   9600),
        (0x0B,   4800),
        (0x80, 921600)]
    @classmethod
    def get_BRID(cls, baudrate):
        """Get baudrate id for a given baudrate."""
        for brid, br in cls.Baudrates:
            if baudrate==br:
                return brid
        raise MTException("unsupported baudrate.")
    @classmethod
    def get_BR(cls, baudrate_id):
        """Get baudrate for a given baudrate id."""
        for brid, br in cls.Baudrates:
            if baudrate_id==brid:
                return br
        raise MTException("unknown baudrate id.")
    

class OutputMode:
    """Values for the output mode."""
    Temp         = 0x0001
    Calib         = 0x0002
    Orient         = 0x0004
    Auxiliary     = 0x0008
    Position     = 0x0010
    Velocity     = 0x0020
    Status         = 0x0800
    RAWGPS         = 0x1000    # supposed to be incompatible with previous
    RAW         = 0x4000    # incompatible with all except RAWGPS


class OutputSettings:
    """Values for the output settings."""
    Timestamp_None            = 0x00000000
    Timestamp_SampleCnt     = 0x00000001
    OrientMode_Quaternion     = 0x00000000
    OrientMode_Euler        = 0x00000004
    OrientMode_Matrix        = 0x00000008
    CalibMode_AccGyrMag     = 0x00000000
    CalibMode_GyrMag         = 0x00000010
    CalibMode_AccMag         = 0x00000020
    CalibMode_Mag             = 0x00000030
    CalibMode_AccGyr         = 0x00000040
    CalibMode_Gyr             = 0x00000050
    CalibMode_Acc             = 0x00000060
    CalibMode_Mask             = 0x00000070
    DataFormat_Float         = 0x00000000
    DataFormat_12_20         = 0x00000100    # not supported yet
    DataFormat_16_32         = 0x00000200    # not supported yet
    DataFormat_Double         = 0x00000300    # not supported yet
    AuxiliaryMode_NoAIN1     = 0x00000400
    AuxiliaryMode_NoAIN2     = 0x00000800
    PositionMode_LLA_WGS84     = 0x00000000
    VelocityMode_MS_XYZ     = 0x00000000
    Coordinates_NED         = 0x80000000


class XDIGroup:
    """Values for the XDI groups."""
    Temperature                = 0x0800
    Timestamp                = 0x1000
    OrientationData            = 0x2000
    Pressure                = 0x3000
    Acceleration            = 0x4000
    Position                = 0x5000
    AngularVelocity            = 0x8000
    GPS                        = 0x8800
    SensorComponentReadout    = 0xA000
    AnalogIn                = 0xB000
    Magnetic                = 0xC000
    Velocity                = 0xD000
    Status                    = 0xE000


class MTException(Exception):
    def __init__(self, message):
        self.message = message
    def __str__(self):
        return "MT error: " + self.message

