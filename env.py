import numpy as np
import pandas as pd



class World(Object):
    def __init__(my_pos, other_pos, instability, lane_width=10, num_lanes=2, world_dim=100, speed_of_light=1):
        self.lane_width = lane_width
        self.num_lanes = num_lanes
        self.world_dim = (world_dim, world_dim)
        self.speed_of_light = speed_of_light
        self.my_pos = my_pos
        self.other_pos = other_pos
        self.instability = instability


    def step(new_pos):

    def check_my_pos():


    def check_square(x, y):
