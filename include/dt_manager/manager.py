import numpy as np
import math
from dt_comm.enums import Ground, SafetyStatus
import rospy

class Manager(object):

    def __init__(self, rewards):
        # counts Agent points based on safety_status and ground_status
        # logs recording and visualization
        # counts timesteps and starts and stops sim
        self.rewards = rewards

        self.time = []
        self.rewards_log = []
        self.score_log = []
        self.my_bot_path = []
        self.other_bot_path = []
        self.safety_statuses = []
        self.ground_types = []

    def step(self, time, state):
        self.time.append(time)
        self.rewards_log.append(self.calc_reward(state["safety_status"], state["ground_type"]))
        self.score_log.append(sum(self.rewards_log))
        self.my_bot_path.append(state["pose_our_duckie"])
        self.other_bot_path.append(state["pose_other_duckie"])
        self.safety_statuses.append(state["safety_status"])
        self.ground_types.append(state["ground_type"])


    def calc_reward(self, safety_status, ground_type):
        reward = 0
        if safety_status == SafetyStatus.FINE:
            reward += self.rewards["fine"]
        if safety_status == SafetyStatus.COLLISION:
            reward += self.rewards["collision"]
        if ground_type == Ground.LOST:
            reward += self.rewards["lost"]
        if ground_type == Ground.RIGHT_LANE:
            reward += self.rewards["right_lane"]
        if ground_type == Ground.WRONG_LANE:
            reward += self.rewards["wrong_lane"]
        if ground_type == Ground.PARTIALLY_OUT_OF_ROAD:
            reward +- self.rewards["part_out"]
        return reward

    def get_records(self):
        return self.t, self.timesteps, self.rewards_log, self.score_log, self.my_bot_path, self.other_bot_path, self.safety_statuses, self.ground_types
