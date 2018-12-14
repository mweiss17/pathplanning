import numpy as np
import math
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


    def pos(self):
        return (self.x, self.y, self.theta)

    def update_plan(self, plan):
        self.plan = deque(plan)

    def drive(self, angle, speed):
        if self.theta - angle > self.limit:
            angle = self.theta - self.limit
        elif angle - self.theta > self.limit:
            angle = self.theta + self.limit
        self.theta = angle
        self.x += math.sin(angle) * speed * self.dt
        self.y += math.cos(angle) * speed * self.dt
        rospy.loginfo("theta: " + str(self.theta))


    def sample_plan(self):
        raise NotImplementedError


class MyBot(Bot):

    def sample_plan(self):
        if len(self.plan) != 0:
            angle = self.plan.popleft()
            self.drive(angle, 1.0)
            rospy.loginfo("sample MyBot")

        else:
            self.drive(0, 0)

class SlowBot(Bot):

    def sample_plan(self):
        self.drive(0, .25)
        rospy.loginfo("sample Slowbot")

