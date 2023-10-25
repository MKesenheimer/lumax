from lumaxlib import *
import time
import math

def restrict(x, minx, maxx):
    return max(min(maxx, x), minx)

# test
print("API version: {}".format(lumax.get_api_version()))
print("Number of physical devices: {}".format(lumax.get_physical_devices()))
lhandle = lumax.open_device(1, 0)
print("Lumax handle: {}".format(lhandle))

print("SetTTL return: {}".format(lumax.setTTL(lhandle, 0)))

ret, timeToWait, bufferChanged = lumax.wait_for_buffer(lhandle, 17)
print("WaitForBuffer return: {}, {}, {}".format(ret, timeToWait, bufferChanged))

brigthness = 1.0 # 0 to 1
r = int(brigthness * 255 * 255)
g = int(brigthness * 255 * 255)
b = int(brigthness * 255 * 255)
r = restrict(r, 0, 255 * 255)
g = restrict(g, 0, 255 * 255)
b = restrict(b, 0, 255 * 255)
npoints = 100
points = lpoints(npoints)
for i in range(0, npoints):
    x, y = geometry.circle_point(128 * 255, 128 * 255, 5000, i, npoints)
    points.struct_arr[i].x = x
    points.struct_arr[i].y = y
    points.struct_arr[i].r = r
    points.struct_arr[i].g = g
    points.struct_arr[i].b = b

# print the points
#for i in range(0, points.length):
#    print("p{} = {}, {}, {}, {}, {}".format(i, points.struct_arr[i].x, points.struct_arr[i].y, points.struct_arr[i].r, points.struct_arr[i].g, points.struct_arr[i].b))

ret, timeToWait = lumax.send_frame(lhandle, points, 4000, 0)
print("SendFrame return: {}, {}".format(ret, timeToWait))
ret, timeToWait, bufferChanged = lumax.wait_for_buffer(lhandle, 17)
time.sleep(1)

print("StopFrame return: {}".format(lumax.stop_frame(lhandle)))
print("CloseDevice return: {}".format(lumax.close_device(lhandle)))
