#!/usr/bin/env python

import rospy

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D
from pathplan_uncertainty.srv import GroundType
from dt_simulator.manager import Manager
from dt_simulator.world import World, DroveOffTheFreakinRoad, RammedAFreakinDuckiebot 


class SimNode(object):

    def orientation_seq_cb(self, ori_seq_msg):
        pass

    def computation_time_cb(self, k):
        pass

    def ground_type_srv_cb(self, req_msg):
        return 1

    def __init__(self):
        self.manager = Manager()
        
        # Parameters
        self.dt = rospy.get_param("/dt_simulator_node/dt")

        self.our_duckie_start_pose = rospy.get_param("/dt_simulator_node/our_duckie/start_pose")
        self.our_duckie_velocity = rospy.get_param("/dt_simulator_node/our_duckie/velocity")

        self.other_duckie_start_pose = rospy.get_param("/dt_simulator_node/other_duckie/start_pose")
        self.other_duckie_velocity = rospy.get_param("/dt_simulator_node/other_duckie/velocity")
        self.other_duckie_type = rospy.get_param("/dt_simulator_node/other_duckie/type")

        self.world = World(self.manager)

        # Publishers
        self.pub_pose_our_duckie = rospy.Publisher("/sim/pose_our_duckie",Pose2D, queue_size=1)
        self.pub_pose_other_duckie = rospy.Publisher("/sim/pose_other_duckie",Pose2D, queue_size=1)
    
        # Subscribers
        self.sub_agent_orientation_seq = rospy.Subscriber("/agent/orientation_seq",Pose2D, self.orientation_seq_cb)
        self.sub_agent_computation_time_steps = rospy.Subscriber("/agent/computation_time_steps", Int32, self.computation_time_cb)

        # Services
        self.srv_ground_type = rospy.Service('get_ground_type',GroundType, self.ground_type_srv_cb)

        rospy.loginfo("[SimNode] Initialized.")



    def orientation_seq_cb(self, ori_seq_msg):
        pass

    def computation_time_cb(self, k):
        pass

    def ground_type_srv_cb(self, req_msg):
        response = Int32()
        response.data = 1
        return response

    def onShutdown(self):
        rospy.loginfo("[SimNode] Shutdown again.")



if __name__ == '__main__':
    rospy.init_node('sim_node',anonymous=False)
    sim_node = SimNode()
    rospy.on_shutdown(sim_node.onShutdown)
    rospy.spin()