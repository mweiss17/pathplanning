import logging
import rospy
from enums import Ground, SafetyStatus
from .Planner import RRT_Dubins


class Agent(object):
    num_lanes = 2
    world_dim = (100, 100)
    speed_of_light = 1

    def __init(roadwidth,horizon):

        self.ourduckie_horizon = ourduckie_horizon                              ##in terms of horizon or time
        self.lane_width = roadwidth
        self.time_steps = 12 #??
        self.ourPlanner = RRT_Dubins(self.lane_width, self.ourduckie_horizon)

	def compute_our_plan(other_duckie_obs): 
	    orientation_seq = self.ourPlanner.update_plan(other_duckie_obs,[self.other_bot.radius])
	    
	    #time step info pass on
