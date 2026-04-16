import ctypes

PACKETSIZE = 128

def CalculateChecksum(packet_bytes):
    cs = 0
    for b in packet_bytes[:-1]:
        cs ^= b
    return cs

class ServoCmd_t(ctypes.Structure):
    _fields_ = [
        ("forward", ctypes.c_float),
        ("pitch", ctypes.c_float),
        ("roll", ctypes.c_float),
    ]

class ThrusterCmd_t(ctypes.Structure): 
    _fields_ = [
        ("thrust", ctypes.c_float),
    ]

class IMUCom_t(ctypes.Structure): 
    _fields_ = [
        ("acc_x", ctypes.c_float),
        ("acc_y", ctypes.c_float),
        ("acc_z", ctypes.c_float),
        ("gyro_x", ctypes.c_float),
        ("gyro_y", ctypes.c_float),
        ("gyro_z", ctypes.c_float),
    ]

class MagCom_t(ctypes.Structure):
    _fields_ = [
        ("head_x", ctypes.c_float),
        ("head_y", ctypes.c_float),
        ("head_z", ctypes.c_float),
    ]

class Bar30Com_t(ctypes.Structure):
    _fields_ = [
        ("pressure", ctypes.c_float),
        ("water_temp", ctypes.c_float),
    ]

class HumidCom_t(ctypes.Structure):
    _fields_ = [
        ("humidity", ctypes.c_float),
        ("air_temp", ctypes.c_float),
    ]

class BoardCom_t(ctypes.Structure):
    _fields_ = [
        ("board_temp", ctypes.c_float),
    ]

class InPacket(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("SoP", ctypes.c_uint8*2),
        ("pad0", ctypes.c_uint8*2), # mimic STM32 padding
        ("IMUCom", IMUCom_t),
        ("MagCom", MagCom_t),
        ("Bar30Com", Bar30Com_t),
        ("HumidCom", HumidCom_t),
        ("BoardCom", BoardCom_t),
        ("CheckSum", ctypes.c_uint8),
        ("pad1", ctypes.c_uint8), # bring size to 64 bytes total
    ]

class OutPacket(ctypes.Structure):
    _fields_ = [
        ("SoP", ctypes.c_uint8*2),
        ("ServoCmd", ServoCmd_t),
        ("ThrusterCmd", ThrusterCmd_t),
        ("CheckSum", ctypes.c_uint8),
    ]
