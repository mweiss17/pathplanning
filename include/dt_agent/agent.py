import logging
import rospy
from dt_comm.enums import Ground, SafetyStatus
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
        self.planner = mctsPlanner(self.predictor, agent_params, sim_params, our_duckie_params, reward_params)  ##needs to have access to predictor


    def compute_our_plan(self, obs_msg): 

        self.planner.predictor.predict(obs_msg)  ##change

        timesteps = self.draw_computation_time_steps()
        goal = [0, obs_msg.our_duckie_pose.y + 8]
        path, angles, moves, collision, rewards, visits, lost_candidates, lost_cand_collisions = self.planner.computePlan(goal, obs_msg)

        plan = []
        angle = obs_msg.our_duckie_pose.theta
        for move in moves:
            angle += move
            plan.append(angle)

        return plan, timesteps

    def draw_computation_time_steps(self):
        number_steps = int(round(np.random.normal(self.comp_time_mean, self.comp_time_std_dev)))
        if number_steps < 1:
            number_steps = 1
        if number_steps > self.comp_time_mean + 3*self.comp_time_std_dev:
            number_steps = int(round(self.comp_time_mean + 3*self.comp_time_std_dev))
        return number_steps
