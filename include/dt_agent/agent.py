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

        #the dubins rrt method will compute rrt with an added constraint -> (min turning radius(so that unrealistic/jerky turns aren't used), and that bot can go only fwd)
        #rrt samples points from the random_area grid defined, and iteratively adds them to the nodelist(which starts with just the start node)
        #points added choose a parent in the tree acc to min dist, and after adding the point, the tree is rewired so that any existing node closer to added node gets detached and connected to this node as its child
        #idea: extension to rrt for dynamic obstacles -- > give propagated positions in time (of the obstacles) to the rrt class, and do the collision check with the propagated value (storing with each node a 't' value to know at which time step this node would be reached) 
        self.ourPlanner = RRT_Dubins(self.lane_width, self.ourduckie_horizon)

	def compute_our_plan(other_duckie_obs): 
	    orientation_seq = self.ourPlanner.update_plan(other_duckie_obs,[self.other_bot.radius])
	    
	    #time step info pass on

