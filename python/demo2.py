from lumaxlib import *
import time
import math
import sys
import os
import numpy

def main():
    try:
        renderer = lumax_renderer()

        # setup frame, generate a circle
        brigthness = 0.45 # 0 to 1
        r = int(brigthness * 255 * 255)
        g = int(brigthness * 255 * 255)
        b = int(brigthness * 255 * 255)
        r = restrict(r, 0, 255 * 255)
        g = restrict(g, 0, 255 * 255)
        b = restrict(b, 0, 255 * 255)
        circle = geometry.circle(128 * 255, 128 * 255, 5000, 100, r, g, b)
        renderer.add_shape_to_frame(circle)

        # add another circle
        circle1 = geometry.circle(150 * 255, 150 * 255, 500, 10, 0, 255 * 255, 0)
        renderer.add_shape_to_frame(circle1)

        # add a point
        point = numpy.array([128 * 255, 128 * 255, 255 * 255, 255 * 255, 255 * 255])
        renderer.add_point_to_frame(point)

        # send the frame to the device
        renderer.send_frame(1000)

        # wait
        time.sleep(1)

        # new frame
        renderer.new_frame()

        # add multiple points (triangle)
        points = numpy.array([[190 * 255, 170 * 255, 0, 255 * 255, 255 * 255], 
                              [190 * 255, 190 * 255, 0, 255 * 255, 255 * 255],
                              [200 * 255, 190 * 255, 0, 255 * 255, 255 * 255],
                              [190 * 255, 170 * 255, 0, 255 * 255, 255 * 255]])
        renderer.add_points_to_frame(points)

        # add line with interpolated points
        line = geometry.line(160 * 255, 160 * 255, 178 * 255, 178 * 255, 5, 255 * 255, 255 * 255, 255 * 255)
        renderer.add_shape_to_frame(line)

        # add triangle
        triangle = geometry.triangle(100 * 255, 100 * 255, 156 * 255, 100 * 255, 128 * 255, 90 * 255, 10, 255 * 255, 255 * 255, 0)
        renderer.add_shape_to_frame(triangle)

        # add square
        square = geometry.tetragon(118 * 255, 118 * 255, 138 * 255, 118 * 255, 138 * 255, 138 * 255, 118 * 255, 138 * 255, 10, 255 * 255, 0, 255 * 255)
        renderer.add_shape_to_frame(square)

        # send the frame to the device
        renderer.send_frame(1000)

        # wait
        time.sleep(100)

        # close device
        renderer.close_device()

    except KeyboardInterrupt:
        print("Exiting.")
        renderer.close_device()
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)

# main program
if __name__ == '__main__':
    #parser = argparse.ArgumentParser(description="")
    #parser.add_argument('--posX', dest='posx', type=int, default=500, help='x-position to read the data from')
    #parser.add_argument('--posY', dest='posy', type=int, default=500, help='y-position to read the data from')
    #parser.add_argument('--pack', dest='pack', type=int, default=24, help='y-position to read the data from')
    #args = parser.parse_args()

    main()

