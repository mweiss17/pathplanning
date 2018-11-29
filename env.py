import numpy as np
import pandas as pd
import math
from enum import Enum

class Ground(Enum):
    RIGHT_LANE = 0
    WRONG_LANE = 1
    PARTIALLY_OUT_OF_ROAD = 2
    LOST = 3

class SafetyStatus(Enum):
    FINE = 1
    COLLISION = -1
    LOST = 2

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
    radius = 2 # Fat Sphere Duckie
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.plan = np.array([])

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

    def __init__(self):
        self.bots = [MyBot(), SlowBot()] # Always keep MyBot as first bot otherwise my sh*tty code will break

    def step(self, new_pos):
        for bot in self.bots:
            bot.sample_plan()

        ground_state = self.check_ground(self.bots[0])
        safety_state = self.check_safety(self.bots)

    def check_safety(self, bots):
        my_bot = bots.pop()
        x1, y1, _ = my_bot.pos()
        for bot in bots:
            x2, y2, _ = bot.pos()
            if abs(x1 - x2) < 2 * bot.radius and abs(y1 - y2) < 2 * bot.radius:
                return SafetyStatus.COLLISION
        return SafetyStatus.FINE

    def check_ground(self, bot):
        x = bot.pos()[0]
        if abs(x) < 5:
            return Ground.RIGHT_LANE
        elif x < -5 and x > -15:
            return Ground.WRONG_LANE
        elif (x > - 15 - bot.radius and x < -15 + bot.radius) or (x > 5 - bot.radius and x < 5 + bot.radius):
            return Ground.PARTIALLY_OUT_OF_ROAD
        else:
            return Ground.LOST
