import numpy as np
import math
from enums import Ground, SafetyStatus
from world import World

class Manager(object):
    dt = .2

    def __init__(self):
        # counts Agent points based on safety_status and ground_status
        # logs recording and visualization
        # counts timesteps and starts and stops sim
        self.t = 0
        self.rewards = []
        self.my_bot_path = []
        self.other_bot_path = []
        self.world = World(manager=self, show_logs=True)

    def step(self, my_bot, other_bot, safety_status, ground_type):
        self.t += Manager.dt
        self.rewards.append(self.calc_reward(safety_status, ground_type))
        self.my_bot_path.append(my_bot.pos())
        self.other_bot_path.append(other_bot.pos())

    @staticmethod
    def calc_reward(ground_type, safety_status):
        reward = 0
        if safety_status == SafetyStatus.FINE:
            reward += 1
        if safety_status == SafetyStatus.COLLISION:
            reward -= 1000
        if safety_status == SafetyStatus.LOST:
            reward -= 1000
        if ground_type == Ground.RIGHT_LANE:
            reward += 1
        if ground_type == Ground.WRONG_LANE:
            reward -= 1
        if ground_type == Ground.PARTIALLY_OUT_OF_ROAD:
            reward -= 3
        if ground_type == Ground.LOST:
            reward -= 1000
        return reward
