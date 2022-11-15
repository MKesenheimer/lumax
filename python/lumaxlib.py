"""
Define the C-variables and functions from the C-files that are needed in Python
"""
from ctypes import *
import sys
import math
import numpy

def restrict(x, minx, maxx):
    return max(min(maxx, x), minx)

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

class shape:
    def __init__(self):
        """ empty constructor """
        self.points = numpy.empty((0, 5), dtype='uint16')
        self.npoints = 0

    def __init__(self, points, color):
        """ define a shape by numpy arrays for coordinates and color.
            points: for example: numpy.array([[0, 1], [2, 3], [4, 5]]) 
            color: for example: numpy.array([[128, 255, 128], [255, 255, 128], [128, 255, 255]]) 
            note: points and color must be equal in size, or color must be of size 1 (one color for all points) """

        plength = len(points)
        clength = len(color)
        if plength == 0 or clength == 0:
            raise Exception("Arrays must not be empty.")

        if plength != clength and clength != 1:
            raise Exception("Points and color array must be equal in size, or color array must be of size 1.") 

        self.points = numpy.empty((plength, 5), dtype='uint16')
        for i in range(0, plength):
            self.points[i, 0] = points[i, 0]
            self.points[i, 1] = points[i, 1]
            self.points[i, 2] = color[i % clength, 0]
            self.points[i, 3] = color[i % clength, 1]
            self.points[i, 4] = color[i % clength, 2]

        self.npoints = plength

    def get_points(self):
        return self.points

    def get_number_of_points(self):
        return self.npoints

    def add_point(self, xypoint):
        pass


class geometry:
    def circle_point(x, y, r, i, npoints):
        th = 2 * math.pi / npoints * i
        xunit = int(r * math.cos(th) + x)
        yunit = int(r * math.sin(th) + y)
        return xunit, yunit

    def new_circle(x0, y0, r, npoints, rd, gr, bl):
        points = numpy.empty((npoints, 2), dtype='uint16')
        colors = numpy.array([[rd, gr, bl]])
        for i in range(0, npoints):
            x, y = geometry.circle_point(x0, y0, r, i, npoints)
            points[i, 0] = x
            points[i, 1] = y
        return shape(points, colors)



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



class lumax_renderer:
    def __init__(self):
        print("API version: {}".format(lumax.get_api_version()))
        print("Number of physical devices: {}".format(lumax.get_physical_devices()))
        self.lhandle = lumax.open_device(1, 0)
        print("Lumax handle: {}".format(self.lhandle))

        print("SetTTL return: {}".format(lumax.setTTL(self.lhandle, 0)))

        ret, timeToWait, bufferChanged = lumax.wait_for_buffer(self.lhandle, 17)
        print("WaitForBuffer return: {}, {}, {}".format(ret, timeToWait, bufferChanged))

        self.shapes = numpy.array([])
        self.totnpoints = 0

    def new_frame(self):
        self.shapes = numpy.array([])
        self.totnpoints = 0

    def add_shape_to_frame(self, shape):
        self.shapes = numpy.append(self.shapes, shape)
        self.totnpoints += shape.get_number_of_points()

    def send_frame(self, pointrate : int):
        if len(self.shapes) == 0:
            raise Exception("Frame is empty.")

        buffer = lpoints(self.totnpoints)
        lastlen = 0
        for i in range(0, len(self.shapes)):
            p = self.shapes[i].get_points()
            for j in range(0, len(p)):
                buffer.struct_arr[i * lastlen + j].x = p[j, 0]
                buffer.struct_arr[i * lastlen + j].y = p[j, 1]
                buffer.struct_arr[i * lastlen + j].r = p[j, 2]
                buffer.struct_arr[i * lastlen + j].g = p[j, 3]
                buffer.struct_arr[i * lastlen + j].b = p[j, 4]
            lastlen = len(p)

        # print the points
        #for i in range(0, buffer.length):
        #    print("p{} = {}, {}, {}, {}, {}".format(i, buffer.struct_arr[i].x, buffer.struct_arr[i].y, buffer.struct_arr[i].r, buffer.struct_arr[i].g, buffer.struct_arr[i].b))


        ret, timeToWait = lumax.send_frame(self.lhandle, buffer, pointrate, 0)
        print("SendFrame return: {}, {}".format(ret, timeToWait))
        ret, timeToWait, bufferChanged = lumax.wait_for_buffer(self.lhandle, 17)
        return

    def stop_frame(self):
        print("StopFrame return: {}".format(lumax.stop_frame(self.lhandle)))
        return

    def close_device(self):
        print("StopFrame return: {}".format(lumax.stop_frame(self.lhandle)))
        print("CloseDevice return: {}".format(lumax.close_device(self.lhandle)))
        return