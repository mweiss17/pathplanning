import logging
import rospy
from enums import Ground, SafetyStatus
from .Planner import RRT_Dubins


class Agent(object):
    lane_width = 10
    num_lanes = 2
    world_dim = (100, 100)
    speed_of_light = 1

    def __init():

        self.ourduckie_horizon = 12
        self.time_steps = 12 #??
        self.ourPlanner = RRT_Dubins(self.lane_width, self.ourduckie_horizon)

	def compute_our_plan(other_duckie_obs): 
	    orientation_seq = self.ourPlanner.update_plan(other_duckie_obs,[self.other_bot.radius])
	    
	    #time step info pass on
