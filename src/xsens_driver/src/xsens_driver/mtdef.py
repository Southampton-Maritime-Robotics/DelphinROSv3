"""Constant and messages definition for MT communication."""


class Scenarios:
    """Scenario ID (for MTi-10 and MTi-1 series)"""
    ID2Lable = {39: 'general',
                40: 'high_mag_dep',
                41: 'dynamic',
                42: 'low_mag_dep',
                43: 'vru_general',
                50: 'General',
                51: 'high_mag_dep',
                52: 'dynamic',
                53: 'north_reference',
                54: 'vru_general'}


class MID:
    """State setting messages"""
    # Switch to config state
    GoToConfig = '\x30'
    # Switch to measurement state
    GoToMeasurement = '\x10'

    """Device specific messages"""
    # Reset
    #Reset = '\x40'
    # Restore factory defaults
    #RestoreFactoryDef = '\x0E'
    # Output configuration (MTi-10/100 series only), N*4 bytes
    OutputConfiguration = '\xC0'
    # Latitude, Longitude and Altitude for local declination and gravity
    SetLatLonAlt = '\x6E'  # Set and Req use the same MID
    # Changes the state of the option flags
    SetOptionFlags = '\x48'
    # Gyro bias estimation
    SetNoRotation = '\x22'

    """Data message"""
    # Newer data packet (MTi-1/10/100 series)
    MTData2 = '\x36'

    """XKF Filter messages"""
    # Request the available XKF scenarios on the device
    ReqAvailableScenarios = '\x62'
    # Current XKF scenario
    SetCurrentScenario = '\x64'  # Set and Req use the same MID


class XDIGroup:
    """Values for the XDI groups."""
    OrientationData = 0x2030  # Euler angles
    Acceleration = 0x4020  # acceleration with gravity
    Acceleration_free = 0x4030  # free acceleration
    AngularVelocity = 0x8020  # rate of turn
    MagneticFieldVector = 0xC020

    Req = {'ori': OrientationData,
           'linAcc': Acceleration,
           'linAcc_free': Acceleration_free,
           'angVel': AngularVelocity,
           'magField': MagneticFieldVector}

    # FIXME: At present, only 'ENU' convention works
    # Hence, the coordinate tranformation is done manually.
    coorSys = {'ENU': 0x0000,
               'NED': 0x0004,
               'NWU': 0x0008}

    dataFormat = {'f': 0x0000,
                  'd': 0x0003}
