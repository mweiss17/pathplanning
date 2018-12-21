import logging
import rospy
from dt_comm.enums import Ground, SafetyStatus
from dt_agent.planner import RRT_Dubins
from .predictor import PredictorDiscretePropagation
#from mcts import mctsPlanner

class Agent(object):
    num_lanes = 2
    world_dim = (100, 100)
    speed_of_light = 1

    def __init__(self, agent_params, sim_params):

        self.time_horizon = agent_params["time_horizon"]
        self.y_resolution = agent_params["y_resolution"]
        self.vel_resolution = agent_params["vel_resolution"]
        self.comp_time_mean = agent_params["comp_time_mean"]
        self.comp_time_std_dev = agent_params["comp_time_std_dev"]

        self.dt = sim_params["dt"]
        self.road_width = sim_params["road_width"]
        self.other_duckie_type = sim_params["other_duckie_type"]
        self.other_duckie_max_acceleration = sim_params["other_duckie_max_acceleration"]


        # Params


        self.predictor = PredictorDiscretePropagation(agent_params, sim_params)
        #init mcts planner class 
        #self.planner = mctsPlanner(predictor, self.dt)  ##needs to have access to predictor

        #the dubins rrt method will compute rrt with an added constraint -> (min turning radius(so that unrealistic/jerky turns aren't used), 
        #and that bot can go only fwd)
        #rrt samples points from the random_area grid defined, and iteratively adds them to the nodelist(which starts with just the start node)
        #points added choose a parent in the tree acc to min dist, and after adding the point, the tree is rewired so that any existing node closer 
        #to added node gets detached and connected to this node as its child
        #idea: extension to rrt for dynamic obstacles -- > give propagated positions in time (of the obstacles) to the rrt class, 
        #and do the collision check with the propagated value (storing with each node a 't' value to know at which time step this node would be reached) 

        # self.other_bot.radius = 0 ##set this
        #self.ourPlanner = RRT_Dubins(self.lane_width, self.ourduckie_horizon)

    def compute_our_plan(self, other_duckie_obs): 

        self.predictor.predict(other_duckie_obs)  ##change

        #prob = self.predictor.get_collision_probability(-0.25, 1, 20)
        #rospy.loginfo("Probability of collision at position -0.25, 1 at time 20 is: " + str (prob))

        plan = [0, 0, 0, 0, 0]
        timesteps = 5

        return plan, timesteps

        #orientation_seq = self.ourPlanner.update_plan(other_duckie_obs,[self.other_bot.radius]) #note: other_duckie_obs here is a list where we have observed positions of each of the duckeibots other than us
        
        #TODO: pass on time step info 
        
        #return orientation_seq


        ## ! there needs to be a parameter that decides what to do if collision check is true for us with a rolled out obstacle which might not be in that position at the same time as us, but kind of near in time
