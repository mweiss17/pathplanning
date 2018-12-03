#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D
from pathplan_uncertainty.msg import Int32TimeStep, Pose2DTimeStep, WorldState
from pathplan_uncertainty.srv import ManagerRecords, ManagerRecordsResponse
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

        # Subscriber
        self.sub_pose_our_duckie = rospy.Subscriber("/sim/gt/world_state",WorldState, self.world_state_cb)
        
        # Service
        self.get_records_serv = rospy.Service('/manager/get_manager_records',ManagerRecords, self.manager_records_cb_srv)

        rospy.loginfo("[ManagerNode] Initialized.")


    def world_state_cb(self, world_step_msg):
        self.manager.step(world_step_msg)


    def manager_records_cb_srv(self, req_msg):
        # Response is all the records from the manager
        time, rewards, score, path_our_duckie, path_other_duckie, safety_statuses, ground_types = self.manager.get_records()
        response = ManagerRecordsResponse()
        response.safety_statuses = safety_statuses
        response.ground_types = ground_types
        response.time = time
        response.rewards = rewards
        response.score =  score
        response.path_our_duckie = path_our_duckie
        response.path_other_duckie = path_other_duckie
        return response

    def onShutdown(self):
        rospy.loginfo("[ManagerNode] Shutdown.")


if __name__ == '__main__':
    rospy.init_node('manager_node',anonymous=False)
    manager_node = ManagerNode()
    rospy.on_shutdown(manager_node.onShutdown)
    rospy.spin()