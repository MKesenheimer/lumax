"""
Define the C-variables and functions from the C-files that are needed in Python
"""
from ctypes import *
import sys

lib_path = 'libs/liblumax_%s.so' % (sys.platform)
try:
    lumax_lib = CDLL(lib_path)
except:
    try:
        lumax_lib = CDLL('libs/lumax.dll')
    except:
        print('OS %s not recognized or library not found.' % (sys.platform))

class lpoint(Structure):
    #_pack_=2
    _fields_ = [("x", c_int, 16),
                ("y", c_int, 16),
                ("r", c_int, 16),
                ("g", c_int, 16),
                ("b", c_int, 16),
                ("Ch6", c_int, 16),
                ("Ch7", c_int, 16),
                ("Ch8", c_int, 16),
                ("TTL", c_int, 16)]
c_point_p = POINTER(lpoint)

class lpoints(Structure):
    #_pack_=2
    _fields_ = [('length', c_int),
                # an array of structs
                ('struct_arr', c_point_p)]

    def __init__(self, num_of_structs):
        elems = (lpoint * num_of_structs)()
        self.struct_arr = cast(elems, c_point_p)
        self.length = num_of_structs

        for num in range(0, num_of_structs):
            self.struct_arr[num].x = 0
            self.struct_arr[num].y = 0
            self.struct_arr[num].r = 0
            self.struct_arr[num].g = 0
            self.struct_arr[num].b = 0


c_int_p = POINTER(c_int)

lumax_lib.Lumax_GetApiVersion.argtypes = None
lumax_lib.Lumax_GetApiVersion.restype = c_int

lumax_lib.Lumax_GetPhysicalDevices.argtypes = None
lumax_lib.Lumax_GetPhysicalDevices.restype = c_int

lumax_lib.Lumax_OpenDevice.argtypes = (c_int, c_int)
lumax_lib.Lumax_OpenDevice.restype = c_void_p

lumax_lib.Lumax_SetTTL.argtypes = (c_void_p, c_int)
lumax_lib.Lumax_SetTTL.restype = c_int

lumax_lib.Lumax_WaitForBuffer.argtypes = (c_void_p, c_int, c_int_p, c_int_p)
lumax_lib.Lumax_WaitForBuffer.restype = c_int

lumax_lib.Lumax_SendFrame.argtypes = (c_void_p, c_point_p, c_int, c_int, c_int, c_int_p)
lumax_lib.Lumax_SendFrame.restype = c_int

lumax_lib.Lumax_StopFrame.argtypes = (c_void_p,)
lumax_lib.Lumax_StopFrame.restype = c_int

lumax_lib.Lumax_CloseDevice.argtypes = (c_void_p,)
lumax_lib.Lumax_CloseDevice.restype = c_int


def GetApiVersion():
    global lumax_lib
    return int(lumax_lib.Lumax_GetApiVersion())

def GetPhysicalDevices():
    global lumax_lib
    return int(lumax_lib.Lumax_GetPhysicalDevices())

def OpenDevice(numDev, channel):
    global lumax_lib
    return int(lumax_lib.Lumax_OpenDevice(c_int(numDev), c_int(channel)))

def SetTTL(handle, ttl):
    global lumax_lib
    return int(lumax_lib.Lumax_SetTTL(c_void_p(handle), c_int(ttl)))

def WaitForBuffer(handle, timeOut):
    global lumax_lib
    timeToWait = c_int(0)
    bufferChanged = c_int(0)
    ret = lumax_lib.Lumax_WaitForBuffer(c_void_p(handle), c_int(timeOut), byref(timeToWait), byref(bufferChanged))
    return int(ret), int.from_bytes(timeToWait, byteorder='big', signed=True), int.from_bytes(bufferChanged, byteorder='big', signed=True)

def SendFrame(handle, points, scanSpeed, updateMode):
    global lumax_lib
    numOfPoints = points.length
    #print("SendFrame: numOfPoints = {}".format(numOfPoints))
    timeToWait = c_int(0)
    ret = lumax_lib.Lumax_SendFrame(c_void_p(handle), points.struct_arr, c_int(numOfPoints), c_int(scanSpeed), c_int(updateMode), byref(timeToWait))
    return int(ret), int.from_bytes(timeToWait, byteorder='big', signed=True)

def StopFrame(handle):
    global lumax_lib
    return int(lumax_lib.Lumax_StopFrame(c_void_p(handle)))

def CloseDevice(handle):
    global lumax_lib
    return int(lumax_lib.Lumax_CloseDevice(c_void_p(handle)))