from enum import Enum

class SonairEvkStatus(Enum):
    # We have power, but we are not yet operational
    Startup = 0x00
    # We are operational and no objects are detected in the stop zone
    Ok = 0x01
    # We have detected an object in the stop zone
    ObjectInStopZone = 0x20
    # A serious error, contact support
    Error10 = 0x10
    # A serious error, contact support
    Error11 = 0x11
