import numpy as np
import math

class Bot(object):
    limit = .2
    radius = 1 # Fat Sphere Duckie
    def __init__(self, x, y, theta, dt=1.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.plan = np.array([])
        self.dt = dt

    def pos(self):
        return (self.x, self.y, self.theta)

    def update_plan(self, plan):
        self.plan = plan

    def drive(self, angle, speed):
        if abs(self.theta - angle) > self.limit:
            angle = math.copysign(Bot.limit, self.theta - angle)
        self.x += math.sin(angle) * speed * self.dt
        self.y += math.cos(angle) * speed * self.dt

    def sample_plan(self):
        raise NotImplementedError


class MyBot(Bot):
    def __init__(self, dt):
        x = 0
        y = 0
        theta = 0
        super(MyBot, self).__init__(x, y, theta, dt)

    def sample_plan(self):
        if len(self.plan) != 0:
            angle = self.plan.pop()
            self.drive(angle, 1.0)
        else:
            self.drive(0, 0)

class SlowBot(Bot):
    def __init__(self, dt):
        x = 0
        y = 10
        theta = 0
        super(SlowBot, self).__init__(x, y, theta, dt)

    def sample_plan(self):
        self.drive(0, .25)
