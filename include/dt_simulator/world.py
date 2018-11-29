import logging
from enums import Ground, SafetyStatus
from bot import MyBot, SlowBot

class RammedAFreakinDuckiebot(Exception):
    pass

class DroveOffTheFreakinRoad(Exception):
    pass

class World(object):
    lane_width = 10
    num_lanes = 2
    world_dim = (100, 100)
    speed_of_light = 1

    def __init__(self, manager=None, show_logs=False):
        self.manager = manager
        self.bots = [MyBot(self.manager.dt), SlowBot(self.manager.dt)] # Always keep MyBot as first bot otherwise my sh*tty code will break
        logging.getLogger().setLevel(logging.CRITICAL)
        if show_logs:
            logging.getLogger().setLevel(level=logging.INFO)

    def step(self):
        for bot in self.bots:
            logging.info(str(type(bot)) + ": " + str(bot.pos()))
            bot.sample_plan()

        ground_type = self.check_ground(self.bots[0])
        safety_state = self.check_safety(self.bots)
        self.manager.step(self.bots[0], self.bots[1], ground_type, safety_state)
        # self.agent.callback(whatever)

        if safety_state == SafetyStatus.COLLISION:
            raise RammedAFreakinDuckiebot
        elif ground_type == Ground.LOST:
            raise DroveOffTheFreakinRoad

        return self.bots[0], self.bots[1], ground_type, safety_state

    @staticmethod
    def check_safety(bots):
        x1, y1, _ = bots[0].pos()
        x2, y2, _ = bots[1].pos()
        if abs(x1 - x2) < 2 * bots[0].radius and abs(y1 - y2) < 2 * bots[0].radius:
            return SafetyStatus.COLLISION
        return SafetyStatus.FINE

    @staticmethod
    def check_ground(bot):
        x = bot.pos()[0]
        if abs(x) < 5:
            return Ground.RIGHT_LANE
        elif x < -5 and x > -15:
            return Ground.WRONG_LANE
        elif (x > - 15 - bot.radius and x < -15 + bot.radius) or (x > 5 - bot.radius and x < 5 + bot.radius):
            return Ground.PARTIALLY_OUT_OF_ROAD
        return Ground.LOST
