import numpy as np
import math
import random
import rospy
from collections import deque

class Bot(object):
    limit = .2
    def __init__(self, duckie_params, dt):
        self.x = duckie_params["start_pose"][0]
        self.y = duckie_params["start_pose"][1]
        self.theta = duckie_params["start_pose"][2]
        self.plan = np.array([])
        self.dt = dt
        self.radius = duckie_params["radius"]
        self.type = duckie_params["type"]
        self.angle_change_limit = duckie_params["angle_change_limit"]
        self.velocity = duckie_params["velocity"]
        if "max_acceleration" in duckie_params:
            self.max_acceleration = duckie_params["max_acceleration"]
        if "max_velocity" in duckie_params:
            self.max_velocity = duckie_params["max_velocity"]
        if "min_velocity" in duckie_params:
            self.min_velocity = duckie_params["min_velocity"]

    def pos(self):
        return (self.x, self.y, self.theta)

    def vel(self):
        return self.velocity

    def update_plan(self, plan):
        self.plan = deque(plan)

    def drive(self, angle, velocity):
        if self.theta - angle > self.limit:
            angle = self.theta - self.limit
        elif angle - self.theta > self.limit:
            angle = self.theta + self.limit
        self.theta = angle
        self.x += math.sin(angle) * velocity * self.dt
        self.y += math.cos(angle) * velocity * self.dt


    def sample_plan(self):
        raise NotImplementedError


class MyBot(Bot):

    def sample_plan(self):
        if len(self.plan) != 0:
            angle = self.plan.popleft()
            self.drive(angle, self.velocity)

        else:
            self.drive(0, 0)

class ConstantSpeedBot(Bot):
    def sample_plan(self):
        self.drive(self.theta, self.velocity)

class UnstableSpeedBot(Bot):
    def sample_plan(self):
        random_acceleration = (random.random() - 0.5) * self.max_acceleration #between -0.5 and 0.5
        self.velocity += random_acceleration * self.dt
        if self.velocity < self.min_velocity:
            self.velocity = self.min_velocity
        elif self.velocity > self.max_velocity:
            self.velocity = self.max_velocity
        self.drive(self.theta, self.velocity)

