import numpy as np
import math
from dt_comm.enums import Ground, SafetyStatus
from pathplan_uncertainty.msg import Int32TimeStep, Pose2DTimeStep
import rospy
#from geometry_msgs import Pose2D

class Manager(object):

    def __init__(self, rewards):
        # counts Agent score based on safety_status and ground_status
        # logs recording and visualization
        self.rewards = rewards

        self.time = []
        self.rewards_log = []
        self.score_log = []
        self.our_bot_path = []
        self.other_bot_path = []
        self.safety_statuses = []
        self.ground_types = []

    def step(self, state_msg):
        self.time.append(state_msg.time)
        self.rewards_log.append(self.calc_reward(state_msg.safety_status, state_msg.ground_type))
        self.score_log.append(sum(self.rewards_log))
        self.our_bot_path.append(state_msg.our_duckie_pose)
        self.other_bot_path.append(state_msg.other_duckie_pose)
        self.safety_statuses.append(state_msg.safety_status)
        self.ground_types.append(state_msg.ground_type)


    def calc_reward(self, safety_status, ground_type):
        reward = 0

        if safety_status == SafetyStatus.FINE.value:
            reward += self.rewards["fine"]
        if safety_status == SafetyStatus.COLLISION.value:
            reward += self.rewards["collision"]
        if ground_type == Ground.LOST.value:
            reward += self.rewards["lost"]
        if ground_type == Ground.RIGHT_LANE.value:
            reward += self.rewards["right_lane"]
        if ground_type == Ground.WRONG_LANE.value:
            reward += self.rewards["wrong_lane"]
        if ground_type == Ground.PARTIALLY_OUT_OF_ROAD.value:
            reward +- self.rewards["part_out"]

        return reward

    def get_records(self):
        return self.time, self.rewards_log, self.score_log, self.our_bot_path, self.other_bot_path, self.safety_statuses, self.ground_types

    def get_current_score(self):
        return self.score_log[-1]