#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D
from pathplan_uncertainty.msg import Int32TimeStep, Pose2DTimeStep
from pathplan_uncertainty.srv import GroundType
from dt_manager.manager import Manager


class ManagerNode(object):


    def __init__(self):

        # Parameters load
        rwrd_fine = rospy.get_param("/reward/status_fine")
        rwrd_collision = rospy.get_param("/reward/status_collision")
        rwrd_right_lane = rospy.get_param("/reward/type_right_lane")
        rwrd_wrong_lane = rospy.get_param("/reward/type_wrong_lane")
        rwrd_part_out = rospy.get_param("/reward/type_partially_out")
        rwrd_lost = rospy.get_param("/reward/type_lost")
        self.rewards = {"fine": rwrd_fine, "collision": rwrd_collision, "right_lane": rwrd_right_lane, "wrong_lane": rwrd_wrong_lane, "part_out": rwrd_part_out, "lost": rwrd_lost}

        # Manager object
        self.manager = Manager(self.rewards)

        # Subscribers
        self.sub_pose_our_duckie = rospy.Subscriber("/sim/gt/pose_our_duckie",Pose2DTimeStep, self.pose_our_duckie_cb)
        self.sub_pose_other_duckie = rospy.Subscriber("/sim/gt/pose_other_duckie",Pose2DTimeStep, self.pose_other_duckie_cb)
        self.sub_safety_status = rospy.Subscriber("/sim/gt/our_duckie_safety_status",Int32TimeStep, self.safety_status_cb)
        self.ground_type = rospy.Subscriber("/sim/gt/our_duckie_ground_type", Int32TimeStep, self.duckie_ground_type_cb)

        rospy.loginfo("[ManagerNode] Initialized.")

        # Received messages by time
        
        self.received_messages = {}

    def pose_our_duckie_cb(self, pose_ts_msg):
        rospy.loginfo("[ManagerNode] Received pose_our_duckie!")
        time = pose_ts_msg.time
        x = pose_ts_msg.x
        y = pose_ts_msg.y
        theta = pose_ts_msg.theta

        if time not in self.received_messages:
            self.received_messages[time] = {}
        self.received_messages[time]['pose_our_duckie'] = (x, y, theta)

        self.check_and_step(time)

    def pose_other_duckie_cb(self, pose_ts_msg):
        rospy.loginfo("[ManagerNode] Received pose_other_duckie!")
        time = pose_ts_msg.time
        x = pose_ts_msg.x
        y = pose_ts_msg.y
        theta = pose_ts_msg.theta
        if time not in self.received_messages:
            self.received_messages[time] = {}
        self.received_messages[time]['pose_other_duckie'] = (x, y, theta)

        self.check_and_step(time)

    def safety_status_cb(self, int_ts_msg):
        rospy.loginfo("[ManagerNode] Received safety_status!")
        time = int_ts_msg.time
        if time not in self.received_messages:
            self.received_messages[time] = {}
        self.received_messages[time]['safety_status'] = int_ts_msg.data

        self.check_and_step(time)

    def duckie_ground_type_cb(self, int_ts_msg):
        rospy.loginfo("[ManagerNode] Received ground_type!")
        time = int_ts_msg.time
        if time not in self.received_messages:
            self.received_messages[time] = {}
        self.received_messages[time]['ground_type'] = int_ts_msg.data
        self.check_and_step(time)

    def check_and_step(self, time):
        received_all = 'pose_our_duckie' in self.received_messages[time] and 'pose_other_duckie' in self.received_messages[time] and 'safety_status' in self.received_messages[time] and 'ground_type' in self.received_messages[time]
        if received_all:
            rospy.loginfo("[ManagerNode] Received all at time: " + str(time))
            self.manager.step(time, self.received_messages.pop(time))



    def onShutdown(self):
        rospy.loginfo("[ManagerNode] Shutdown.")



if __name__ == '__main__':
    rospy.init_node('manager_node',anonymous=False)
    manager_node = ManagerNode()
    rospy.on_shutdown(manager_node.onShutdown)
    rospy.spin()