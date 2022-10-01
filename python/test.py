from lumaxlib import *
import time

# test
print("API version: {}".format(GetApiVersion()))
print("Number of physical devices: {}".format(GetPhysicalDevices()))
lhandle = OpenDevice(1, 0)
print("Lumax handle: {}".format(lhandle))

print("SetTTL return: {}".format(SetTTL(lhandle, 0)))

ret, timeToWait, bufferChanged = WaitForBuffer(lhandle, 17)
print("WaitForBuffer return: {}, {}, {}".format(ret, timeToWait, bufferChanged))

points = lpoints(1)
points.struct_arr[0].x = 32768
points.struct_arr[0].y = 34768
points.struct_arr[0].r = 25600
points.struct_arr[0].g = 47770
points.struct_arr[0].b = 3429

ret, timeToWait = SendFrame(lhandle, points, 10000, 0)
print("SendFrame return: {}, {}".format(ret, timeToWait))
ret, timeToWait, bufferChanged = WaitForBuffer(lhandle, 17)
time.sleep(1)

print("StopFrame return: {}".format(StopFrame(lhandle)))
print("CloseDevice return: {}".format(CloseDevice(lhandle)))