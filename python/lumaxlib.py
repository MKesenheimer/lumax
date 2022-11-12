"""
Define the C-variables and functions from the C-files that are needed in Python
"""
from ctypes import *
import sys
import math

class lpoint(Structure):
    #_pack_=2
    _fields_ = [("x", c_uint16),
                ("y", c_uint16),
                ("r", c_uint16),
                ("g", c_uint16),
                ("b", c_uint16),
                ("Ch6", c_uint16),
                ("Ch7", c_uint16),
                ("Ch8", c_uint16),
                ("TTL", c_uint16)]
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
        
class lumax:
    global lumax_lib
    lib_path = './lumax/python/libs/liblumax_%s.so' % (sys.platform)
    try:
        lumax_lib = CDLL(lib_path)
    except:
        try:
            lumax_lib = CDLL('./lumax/python/libs/lumax.dll')
        except:
            lib_path = './libs/liblumax_%s.so' % (sys.platform)
            try:
                lumax_lib = CDLL(lib_path)
            except:
                try:
                    lumax_lib = CDLL('./libs/lumax.dll')
                except:
                    print('OS %s not recognized or library not found.' % (sys.platform))

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

    

    def get_api_version():
        global lumax_lib
        return int(lumax_lib.Lumax_GetApiVersion())

    def get_physical_devices():
        global lumax_lib
        return int(lumax_lib.Lumax_GetPhysicalDevices())

    def open_device(numDev, channel):
        global lumax_lib
        try:
            return int(lumax_lib.Lumax_OpenDevice(c_int(numDev), c_int(channel)))
        except:
            return 0

    def setTTL(handle, ttl):
        global lumax_lib
        return int(lumax_lib.Lumax_SetTTL(c_void_p(handle), c_int(ttl)))

    def wait_for_buffer(handle, timeOut):
        global lumax_lib
        timeToWait = c_int(0)
        bufferChanged = c_int(0)
        ret = lumax_lib.Lumax_WaitForBuffer(c_void_p(handle), c_int(timeOut), byref(timeToWait), byref(bufferChanged))
        return int(ret), int.from_bytes(timeToWait, byteorder='big', signed=True), int.from_bytes(bufferChanged, byteorder='big', signed=True)

    def send_frame(handle, points, scanSpeed, updateMode):
        global lumax_lib
        numOfPoints = points.length
        timeToWait = c_int(0)
        ret = lumax_lib.Lumax_SendFrame(c_void_p(handle), points.struct_arr, c_int(numOfPoints), c_int(scanSpeed), c_int(updateMode), byref(timeToWait))
        return int(ret), int.from_bytes(timeToWait, byteorder='big', signed=True)

    def stop_frame(handle):
        global lumax_lib
        return int(lumax_lib.Lumax_StopFrame(c_void_p(handle)))

    def close_device(handle):
        global lumax_lib
        return int(lumax_lib.Lumax_CloseDevice(c_void_p(handle)))

    @staticmethod
    def circle_point(x, y, r, i, npoints):
        th = 2 * math.pi / npoints * i
        xunit = int(r * math.cos(th) + x)
        yunit = int(r * math.sin(th) + y)
        return xunit, yunit

    @staticmethod
    def gen_circle(x, y, r, npoints, rd, gr, bl):
        points = lpoints(npoints)
        for i in range(0, npoints):
            x, y = lumax.circle_point(128 * 255, 128 * 255, 5000, i, npoints)
            points.struct_arr[i].x = x
            points.struct_arr[i].y = y
            points.struct_arr[i].r = rd
            points.struct_arr[i].g = gr
            points.struct_arr[i].b = bl
        return points