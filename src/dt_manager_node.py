#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D
from pathplan_uncertainty.srv import GroundType
from dt_manager.manager import Manager


class ManagerNode(object):


    def __init__(self):

        # Parameters load
        self.dt = rospy.get_param("/dt_manager_node/dt")

        rwrd_fine = rospy.get_param("/dt_manager_node/reward/status_fine")
        rwrd_collision = rospy.get_param("/dt_manager_node/reward/status_collision")
        rwrd_right_lane = rospy.get_param("/dt_manager_node/reward/type_right_lane")
        rwrd_wrong_lane = rospy.get_param("/dt_manager_node/reward/type_wrong_lane")
        rwrd_part_out = rospy.get_param("/dt_manager_node/reward/type_partially_out")
        rwrd_lost = rospy.get_param("/dt_manager_node/reward/type_lost")
        self.rewards = {"fine": rwrd_fine, "collision": rwrd_collision, "right_lane": rwrd_right_lane, "wrong_lane": rwrd_wrong_lane, "part_out": rwrd_part_out, "lost": rwrd_lost}

        # Manager object
        self.manager = Manager(self.dt, self.rewards)

        # Subscribers
        self.sub_pose_our_duckie = rospy.Subscriber("/sim/pose_our_duckie",Pose2D, self.pose_our_duckie_cb)
        self.sub_pose_other_duckie = rospy.Subscriber("/sim/pose_other_duckie",Pose2D, self.pose_other_duckie_cb)
        self.sub_safety_status = rospy.Subscriber("/sim/our_duckie_safety_status",Int32, self.safety_status_cb)
        self.ground_type = rospy.Subscriber("/sim/our_duckie_ground_type", Int32, self.duckie_ground_type_cb)

        rospy.loginfo("[ManagerNode] Initialized.")

        # Temporary variables
        self.our_duckie_pose = []
        self.received_our_duckie_pose = False
        self.other_duckie_pose = []
        self.received_other_duckie_pose = False
        self.safety_status = 0
        self.received_safety_status = False
        self.ground_type = 0
        self.received_ground_type = False

    def pose_our_duckie_cb(self, pose_msg):
        x = pose_msg.x
        y = pose_msg.y
        theta = pose_msg.theta
        self.our_duckie_pose = (x, y, theta)
        self.received_our_duckie_pose = True

        self.check_and_step()

    def pose_other_duckie_cb(self, pose_msg):
        x = pose_msg.x
        y = pose_msg.y
        theta = pose_msg.theta
        self.other_duckie_pose = (x, y, theta)
        self.received_other_duckie_pose = True

        self.check_and_step()

    def safety_status_cb(self, int_msg):
        self.safety_status = int_msg.data
        self.received_safety_status = True

        self.check_and_step()

    def duckie_ground_type_cb(self, int_msg):
        self.ground_type = int_msg.data
        self.received_ground_type = True

        self.check_and_step()

    def check_and_step(self):
        received_all = self.received_ground_type and self.received_safety_status and self.received_our_duckie_pose and self.received_other_duckie_pose
        if received_all:
            rospy.loginfo("[ManagerNode] Received all!")
            self.manager.step(self.our_duckie_pose, self.other_duckie_pose, self.safety_status, self.ground_type)
            received_ground_type = False
            received_safety_status = False
            received_other_duckie_pose = False
            received_our_duckie_pose = False


    def onShutdown(self):
        rospy.loginfo("[ManagerNode] Shutdown again.")



if __name__ == '__main__':
    rospy.init_node('manager_node',anonymous=False)
    manager_node = ManagerNode()
    rospy.on_shutdown(manager_node.onShutdown)
    rospy.spin()