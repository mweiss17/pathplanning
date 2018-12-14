import logging
import rospy
from bot import MyBot, SlowBot
from visualizer import Visualizer
from dt_comm.enums import Ground, SafetyStatus

class RammedAFreakinDuckiebot(Exception):
    pass

class DroveOffTheFreakinRoad(Exception):
    pass

class World(object):

    def __init__(self, sim_params, world_params, our_duckie_params, other_duckie_params):
        self.dt = sim_params["dt"]
        self.time = 0
        self.road_width = world_params["road_width"]
        # Creating our duckie
        self.my_bot = MyBot(our_duckie_params, self.dt)
        
        # Creating other duckie
        if other_duckie_params["type"] == "slow_duckie":
            self.other_bot = SlowBot(other_duckie_params, self.dt)
        else:
            rospy.logerr("[sim_node][world] Unknown other duckie type. Look in pathplan_uncertainty/config/sim.yaml and make sure it is fine!")
        
        self.my_bot_safety_status = self.check_safety(self.my_bot, self.other_bot)
        self.my_bot_ground_type = self.check_ground(self.my_bot.pos(), self.my_bot.radius)

    def step(self):
        for bot in [self.my_bot, self.other_bot]:
            rospy.loginfo("[sim_node][world] Step at time: " + str(self.time) + " with "+ bot.type + ": " + str(bot.pos()))
            bot.sample_plan()

        self.time += self.dt
        self.my_bot_ground_type = self.check_ground(self.my_bot.pos(), self.my_bot.radius)
        self.my_bot_safety_status = self.check_safety(self.my_bot, self.other_bot)

    def get_state(self):
        return self.time, self.my_bot.pos(), self.my_bot_safety_status, self.my_bot_ground_type, self.other_bot.pos()

    def update_our_duckie_plan(self, plan):
        self.my_bot.update_plan(plan)

    def check_safety(self, my_bot, other_bot):
        x1, y1, _ = my_bot.pos()
        x2, y2, _ = other_bot.pos()
        # Checking if collision (assuming same circular shape for both bots)
        if (x1-x2)**2 + (y1-y2)**2 < (my_bot.radius + other_bot.radius)**2:
            rospy.loginfo("COLLISION!!!!")
            return SafetyStatus.COLLISION
        return SafetyStatus.FINE

    def check_ground(self, pose, radius):
        # Returns the type of ground for a duckie pose
        x = pose[0]
        if abs(x) <= 0.25*self.road_width:
            return Ground.RIGHT_LANE
        elif x < -0.25*self.road_width and x >= -0.75*self.road_width :
            return Ground.WRONG_LANE
        elif (x > -0.75*self.road_width - radius and x < -0.75*self.road_width) or (x > 0.25*self.road_width and x < 0.25*self.road_width + radius):
            return Ground.PARTIALLY_OUT_OF_ROAD
        else:
            return Ground.LOST
