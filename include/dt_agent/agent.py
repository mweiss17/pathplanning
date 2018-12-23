import logging
import rospy
from dt_comm.enums import Ground, SafetyStatus
from dt_agent.planner import RRT_Dubins
from .predictor import PredictorDiscretePropagation
import numpy as np
from mcts import mctsPlanner
from pathplan_uncertainty.msg import Int32TimeStep, Pose2DTimeStep, WorldState, Observation, AgentCommand

class Agent(object):
    num_lanes = 2
    world_dim = (100, 100)
    speed_of_light = 1

    def __init__(self, agent_params, sim_params, our_duckie_params, reward_params):

        # Params

        self.time_horizon = agent_params["time_horizon"]
        self.y_resolution = agent_params["y_resolution"]
        self.vel_resolution = agent_params["vel_resolution"]
        self.comp_time_mean = agent_params["comp_time_mean"]
        self.comp_time_std_dev = agent_params["comp_time_std_dev"]

        self.dt = sim_params["dt"]
        self.road_width = sim_params["road_width"]
        self.other_duckie_type = sim_params["other_duckie_type"]
        self.other_duckie_max_acceleration = sim_params["other_duckie_max_acceleration"]


        self.our_duckie_velocity = our_duckie_params["velocity"]
        self.our_duckie_radius = our_duckie_params["radius"] 
        # Objects
        self.predictor = PredictorDiscretePropagation(agent_params, sim_params)


        

        #init mcts planner class 

        self.planner = mctsPlanner(self.predictor, agent_params, sim_params, our_duckie_params, reward_params)  ##needs to have access to predictor


        #the dubins rrt method will compute rrt with an added constraint -> (min turning radius(so that unrealistic/jerky turns aren't used), 
        #and that bot can go only fwd)
        #rrt samples points from the random_area grid defined, and iteratively adds them to the nodelist(which starts with just the start node)
        #points added choose a parent in the tree acc to min dist, and after adding the point, the tree is rewired so that any existing node closer 
        #to added node gets detached and connected to this node as its child
        #idea: extension to rrt for dynamic obstacles -- > give propagated positions in time (of the obstacles) to the rrt class, 
        #and do the collision check with the propagated value (storing with each node a 't' value to know at which time step this node would be reached) 

        # self.other_bot.radius = 0 ##set this
        #self.ourPlanner = RRT_Dubins(self.lane_width, self.ourduckie_horizon)

    def compute_our_plan(self, obs_msg): 

        self.planner.predictor.predict(obs_msg)  ##change

        #prob = self.predictor.get_collision_probability(-0.25, 1, 20)
        #rospy.loginfo("Probability of collision at position -0.25, 1 at time 20 is: " + str (prob))

        #plan = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        timesteps = self.draw_computation_time_steps()
        goal = [0, obs_msg.our_duckie_pose.y + 8]
        path, angles, moves, collision, rewards, visits, lost_candidates, lost_cand_collisions = self.planner.computePlan(goal, obs_msg)

        print('other duckie pose')
        print(str(obs_msg.other_duckie_pose.x) + '  ...  ' +str(obs_msg.other_duckie_pose.y)) 

        # print('angles')
        # print(angles)
        print('moves:')
        print(moves)
        print('collision costs:')
        print(collision)
        print('path:')
        print(path)
        print('rewards:')
        print(rewards)
        print('node visits: ')
        print(visits)


        plan = []
        angle = obs_msg.our_duckie_pose.theta
        for move in moves:
            angle += move
            plan.append(angle)
        print('radii: us and other duckie')
        print(self.our_duckie_radius)
        print(obs_msg.other_duckie_radius)
        print('candidate rewards')
        print(lost_candidates)
        print('candidate collsiion costs')
        print(lost_cand_collisions)
        print('')
        print('')
        print('')
        print('')

        return plan, timesteps

    def draw_computation_time_steps(self):
        number_steps = int(round(np.random.normal(self.comp_time_mean, self.comp_time_std_dev)))
        if number_steps < 1:
            number_steps = 1
        if number_steps > self.comp_time_mean + 3*self.comp_time_std_dev:
            number_steps = int(round(self.comp_time_mean + 3*self.comp_time_std_dev))
        return number_steps
