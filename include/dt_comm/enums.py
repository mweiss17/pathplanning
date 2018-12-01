from enum import Enum
import rospy


class Ground(Enum):
    RIGHT_LANE = rospy.get_param('/ground_type_protocol/right_lane')
    WRONG_LANE = rospy.get_param('/ground_type_protocol/wrong_lane')
    PARTIALLY_OUT_OF_ROAD = rospy.get_param('/ground_type_protocol/partially_out')
    LOST = rospy.get_param('/ground_type_protocol/lost')

class SafetyStatus(Enum):
    FINE = rospy.get_param('/safety_status_protocol/fine')
    COLLISION = rospy.get_param('/safety_status_protocol/collision')