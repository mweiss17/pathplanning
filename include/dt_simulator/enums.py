from enum import Enum
import rospy

class Ground(Enum):
    RIGHT_LANE = 0
    WRONG_LANE = 1
    PARTIALLY_OUT_OF_ROAD = 2
    LOST = 3

class SafetyStatus(Enum):
    FINE = 1
    COLLISION = -1
    LOST = 2
