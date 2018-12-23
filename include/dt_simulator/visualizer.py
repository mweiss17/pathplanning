import numpy as np
import math
import rospy
import cv2
from collections import deque



class Visualizer(object):

    def __init__(self, image_params, world_params, our_duckie_params, other_duckie_params, sim_params):
        self.road_width = world_params["road_width"]
        self.our_duckie_radius = our_duckie_params["radius"]
        self.other_duckie_radius = other_duckie_params["radius"]
        self.image_width = image_params["width"]
        self.image_height = image_params["height"]
        self.m2pix = image_params["m2pix"]
        self.y_baseline = image_params["y_baseline"]



        self.trajectory = []
        self.start_pose = our_duckie_params["start_pose"]
        self.velocity = our_duckie_params["velocity"]
        self.dt = sim_params["dt"]
        
    def create_base_image(self):
        # Create road image

        image = np.zeros((self.image_height,self.image_width, 3), np.uint8)
        road_width_pix = self.road_width*self.m2pix

        image[:,0:int((self.image_width-road_width_pix)/2)] = (20, 200, 20) # grass left side of the road
        image[:,int((self.image_width+road_width_pix)/2):int(self.image_width)] = (20, 200, 20) # grass right side of the road
        image[:, int((self.image_width-road_width_pix)/2):int(self.image_width/2)] = (60, 60, 100) # wrong lane (a bit red)
        image[:, int(self.image_width/2):int((self.image_width+road_width_pix)/2):] = (60, 100, 60) # wrong lane (a bit green)
        
        return image


    def add_bot(self, image, pose, radius, color_code):
        # Add one bot

        # Conversions in pixels
        road_width_pix = self.road_width*self.m2pix
        radius_pix = int(radius*self.m2pix)


        # Origin of our cartesian frame in picture frame
        center_u = self.image_width/2 + road_width_pix/4
        center_y = self.image_height - self.y_baseline

        # Position of bot center in picture frame
        x = pose[0]
        y = pose[1]
        theta = pose[2]
        u_bot = int(x*self.m2pix + center_u)
        v_bot = int(center_y - y*self.m2pix)
        
        # Fill circle around this position with given radius
        for u in range(u_bot-radius_pix, u_bot+radius_pix):     # For pixels the square around the bot
            for v in range(v_bot-radius_pix, v_bot+radius_pix):
                if ((u-u_bot)*(u-u_bot) + (v-v_bot)*(v-v_bot)) < radius_pix*radius_pix: # Check if in a circle shape
                    if v < self.image_height and v >= 0 and u < self.image_width and u >= 0:   # Check that it is in the image dimensions
                        image[v, u] = color_code
                        if abs((u-u_bot) - math.tan(theta)*(v_bot-v)) <= 1.5 and (v_bot-v)*math.cos(theta+0.01) >= 0: # Check if point is on orientation line and in the right direction
                            image[v, u] = (0, 0, 0)                                     # Put in black


        return image

    def update_trajectory(self, command_msg, start_pose, velocity):
        self.trajectory = command_msg.orientation_seq.data
        self.start_pose = start_pose
        self.velocity = velocity

    def add_trajectory(self, image):
        # Conversions in pixels
        road_width_pix = self.road_width*self.m2pix
        distance_pix = int(self.velocity*self.dt*self.m2pix)

        # Origin of our cartesian frame in picture frame
        center_u = self.image_width/2 + road_width_pix/4
        center_y = self.image_height - self.y_baseline

        # Position of bot center in picture frame
        x_start = self.start_pose[0]
        y_start = self.start_pose[1]
        u_bot = int(x_start*self.m2pix + center_u)
        v_bot = int(center_y - y_start*self.m2pix)

        for traj_el in self.trajectory:
            for u in range(u_bot-distance_pix, u_bot + distance_pix):
                for v in range(v_bot - distance_pix, v_bot + distance_pix):
                    if ((u-u_bot)*(u-u_bot) + (v-v_bot)*(v-v_bot)) < distance_pix*distance_pix:
                        if v < self.image_height and v >= 0 and u < self.image_width and u >= 0:   # Check that it is in the image dimensions
                            if abs((u-u_bot) - math.tan(traj_el)*(v_bot-v)) <= 1.5 and (v_bot-v)*math.cos(traj_el+0.01) >= 0: # Check if point is on trajectory line and in the right direction
                                image[v, u] = (255, 255, 255)                                     # Put in white
            x_start += self.dt*self.velocity*math.sin(traj_el)
            y_start += self.dt*self.velocity*math.cos(traj_el)
            u_bot = int(x_start*self.m2pix + center_u)
            v_bot = int(center_y - y_start*self.m2pix)

        return image



    def create_image_state(self, our_duckie_pose, other_duckie_pose):
        image = self.create_base_image()
        image = self.add_bot(image, our_duckie_pose, self.our_duckie_radius, (0, 0, 255)) # Adding our bot in red
        image = self.add_bot(image, other_duckie_pose, self.other_duckie_radius, (0, 255, 255)) # Adding other bot in yellow
        image = self.add_trajectory(image)
        return image