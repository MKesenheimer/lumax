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

points = lpoints(5)
dist = 50
cnt = 0
for i in (1, -1):
  for j in (-1, 1):
    print("cnt = {}, i = {}, j = {}".format(cnt, i, j))
    points.struct_arr[cnt].x = (128 + i*dist)*255
    points.struct_arr[cnt].y = (128 + j*dist)*255
    points.struct_arr[cnt].r = (128 + i*dist)*255
    points.struct_arr[cnt].g = (128 + j*dist)*255
    points.struct_arr[cnt].b = (128 + i*j*dist)*255
    cnt+=1

# back to start
points.struct_arr[4].x = points.struct_arr[0].x
points.struct_arr[4].y = points.struct_arr[0].y
points.struct_arr[4].r = points.struct_arr[0].r
points.struct_arr[4].g = points.struct_arr[0].g
points.struct_arr[4].b = points.struct_arr[0].b

# print out
for i in range(0, points.length):
    print("p{} = {}, {}, {}, {}, {}".format(i, points.struct_arr[i].x, points.struct_arr[i].y, points.struct_arr[i].r, points.struct_arr[i].g, points.struct_arr[i].b))

ret, timeToWait = SendFrame(lhandle, points, 100, 0)
print("SendFrame return: {}, {}".format(ret, timeToWait))
ret, timeToWait, bufferChanged = WaitForBuffer(lhandle, 17)
time.sleep(10)

print("StopFrame return: {}".format(StopFrame(lhandle)))
print("CloseDevice return: {}".format(CloseDevice(lhandle)))
