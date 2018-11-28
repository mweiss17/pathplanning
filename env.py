import numpy as np
import pandas as pd
import math

class Manager(object):
    dt = 1.0

    def __init__(self):
        # counts Agent points based on safety_status and ground_status
        # logs recording and visualization
        # counts timesteps and starts and stops sim
        self.t = 0
        self.reward = np.array([])
        self.my_bot_path = np.array([])
        self.other_bot_path = np.array([])

    def calc_reward(self, safety_status, ground_type):
        return safety_status + ground_type

    def step(self, my_bot, other_bot, safety_status, ground_type):
        self.t += Manager.dt
        self.reward.append(calc_reward(safety_status, ground_type))
        self.my_bot_path.append(my_bot.pos())
        self.other_bot_path.append(other_bot.pos())

class Bot(object):
    limit = .2
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def pos(self):
        return (self.x, self.y, self.theta)

    def update_plan(self, plan):
        self.plan = plan

    def drive(self, angle, speed):
        if abs(self.theta - angle) > self.limit:
            angle = math.copysign(cls.limit, self.theta - angle)
        self.x += math.cos(angle) * speed * Manager.dt
        self.y += math.sin(angle) * speed * Manager.dt

    def sample_plan(self):
        raise NotImplementedError


class MyBot(Bot):
    def __init__(self):
        super().__init__(x=0, y=0, theta=0)
    def sample_plan(self):
        if self.plan:
            angle = self.plan.pop()
            self.drive(angle, World.speed_of_light)
        else:
            self.drive(0, 0)

class SlowBot(Bot):
    def __init__(self):
        super().__init__(x=0, y=10, theta=0)

    def sample_plan(self):
        self.drive(0, .25)


class World(object):
    lane_width = 10
    num_lanes = 2
    world_dim = (100, 100)
    speed_of_light = 1

    def __init__(self, my_pos, other_pos, instability):
        self.my_bot = Bot(my_pos)
        self.other_pos = Bot(other_pos)
        self.instability = instability

    def step(new_pos):
        self.my_pos =

    def check_pos(x, y):
