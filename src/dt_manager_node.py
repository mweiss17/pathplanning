#!/usr/bin/env python

import rospy

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D
from pathplan_uncertainty.srv import GroundType
from dt_simulator.manager import Manager
from dt_simulator.world import World, DroveOffTheFreakinRoad, RammedAFreakinDuckiebot 


class ManagerNode(object):

    def pose_our_duckie_cb(self, pose_msg):
        pass

    def pose_other_duckie_cb(self, pose_msg):
        pass

    def safety_status_cb(self, int_msg):
        pass

    def duckie_ground_type_cb(self, int_msg)
        pass

    def __init__(self):

        # Parameters load
        



        self.manager = Manager()

        # Subscribers
        self.sub_pose_our_duckie = rospy.Subscriber("/sim/pose_our_duckie",Pose2D, self.pose_our_duckie_cb)
        self.sub_pose_other_duckie = rospy.Subscriber("/sim/pose_other_duckie",Pose2D, self.pose_other_duckie_cb)
        self.sub_safety_status = rospy.Subscriber("/sim/our_duckie_safety_status",Int32, self.safety_status_cb)
        self.ground_type = rospy.Subscriber("/sim/our_duckie_ground_type", Int32, self.duckie_ground_type_cb)
    
        
        # Services
        self.srv_ground_type = rospy.Service('get_ground_type',GroundType, self.ground_type_srv_cb)


        rospy.loginfo("[ManagerNode] Initialized.")



    def onShutdown(self):
        rospy.loginfo("[ManagerNode] Shutdown again.")



if __name__ == '__main__':
    rospy.init_node('manager_node',anonymous=False)
    manager_node = ManagerNode()
    rospy.on_shutdown(manager_node.onShutdown)
    rospy.spin()