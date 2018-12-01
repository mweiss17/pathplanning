import logging
import rospy
from bot import MyBot, SlowBot
from enums import Ground, SafetyStatus
from .Planner import RRT_Dubins

class RammedAFreakinDuckiebot(Exception):
    pass

class DroveOffTheFreakinRoad(Exception):
    pass

class World(object):
    lane_width = 10
    num_lanes = 2
    world_dim = (100, 100)
    speed_of_light = 1

    def __init__(self, dt, our_duckie_params, other_duckie_params):
        self.dt = dt
        # Creating our duckie
        self.my_bot = MyBot(our_duckie_params, self.dt)
        
        # Creating other duckie
        if other_duckie_params["type"] == "slow_duckie":
            self.other_bot = SlowBot(other_duckie_params, self.dt)
        else:
            rospy.logerr("[sim_node][world] Unknown other duckie type. Look in pathplan_uncertainty/config/sim.yaml and make sure it is fine!")
        


        self.ourduckie_horizon = 12
        self.time_steps = 12 #??
        self.ourPlanner = RRT_Dubins(lane_width, self.ourduckie_horizon)



        self.my_bot_safety_status = self.check_safety(self.my_bot, self.other_bot)
        self.my_bot_ground_type = self.check_ground(self.my_bot.pos(), self.my_bot.radius)

    def compute_our_plan(other_duckie_obs): 
        self.ourPlanner.update_plan(other_duckie_obs,[self.other_bot.radius])
        
        #time step info pass on
        #send back computed plan in orientation_seq format


    def step(self):
        for bot in [self.my_bot, self.other_bot]:
            rospy.loginfo("[sim_node][world] Step: " + bot.type + ": " + str(bot.pos()))
            bot.sample_plan()

        self.my_bot_ground_type = self.check_ground(self.my_bot.pos(), self.my_bot.radius)
        self.my_bot_safety_status = self.check_safety(self.my_bot, self.other_bot)

    def get_state(self):
        #loginfo("[SimNode][World] Get state: my_bot_safety_status = " + str(my_bot_safety_status))
        return self.my_bot.pos(), self.my_bot_safety_status, self.my_bot_ground_type, self.other_bot.pos()

    def update_our_duckie_plan(self, plan):
        self.my_bot.update_plan(plan)

    def check_safety(self, my_bot, other_bot):
        x1, y1, _ = my_bot.pos()
        x2, y2, _ = other_bot.pos()
        # Checking if collision (assuming same circular shape for both bots)
        if abs(x1 - x2) < 2 * my_bot.radius and abs(y1 - y2) < 2 * my_bot.radius:
            rospy.loginfo("COLLISION!!!!")
            return SafetyStatus.COLLISION
        return SafetyStatus.FINE

    def check_ground(self, pose, radius):
        # Returns the type of ground for a duckie pose
        x = pose[0]
        if abs(x) < 5:
            return Ground.RIGHT_LANE
        elif x < -5 and x > -15:
            return Ground.WRONG_LANE
        elif (x < - 15 - radius and x > -15 + radius) or (x > 5 - radius and x < 5 + radius):
            return Ground.PARTIALLY_OUT_OF_ROAD
        return Ground.LOST
