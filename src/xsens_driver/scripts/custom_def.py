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
    Temp             = (0x08, 0x10, 0x04, 0x80, 0x04) #XDI_Temperature        # 4 bytes
    SampleTimeFine     = (0x10, 0x60, 0x04, 0x80, 0x04) #XDI_SampleTimeFine    # 4 bytes
    Ori             = (0x20, 0x30, 0x04, 0x80, 0x0c) #XDI_EulerAngles        # 12 bytes
    Acc_lin            = (0x40, 0x20, 0x04, 0x80, 0x0c) #XDI_Acceleration        # 12 bytes
    FreeAcc_lin        = (0x40, 0x30, 0x04, 0x80, 0x0c) #XDI_FreeAcceleration    # 12 bytes
    LatLon             = (0x50, 0x43, 0x04, 0x80, 0x10) #XDI_LatLon            # 16 bytes
    Vel_ang            = (0x80, 0x20, 0x04, 0x80, 0x0c) #XDI_AngularVelocityGr    # 12 bytes
    MagField        = (0xC0, 0x20, 0x04, 0x80, 0x0c) #XDI_MagneticField        # 12 bytes
