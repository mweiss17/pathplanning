import numpy as np
import math
import rospy
import cv2
from collections import deque





class Visualizer(object)

    def __init__(self, world_params, our_duckie_params, other_duckie_params):
        self.road_width = world_params["road_width"]
        self.our_duckie_radius = our_duckie_params["radius"]
        self.other_duckie_radius = other_duckie_params["radius"]

    def image_step(self, state):
        height = 1000
        width = 400
        image = np.zeros((width,height 3), np.uint8)
        m2pix = 100
        road_width_pix = self.road_width*m2pix

        image[:,0:(width-road_width_pix)/2] = (20, 200, 20) # grass left side of the road
        image[:,(width+road_width_pix)/2:width] = (20, 200, 20) # grass right side of the road
        image[:, (width-road_width_pix)/2:width/2] = (60, 60, 100) # wrong lane (a bit red)
        image[:, width/2:(width+road_width_pix)/2:] = (60, 100, 60) # wrong lane (a bit green)
    