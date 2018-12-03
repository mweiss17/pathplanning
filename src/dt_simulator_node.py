#!/usr/bin/env python

import rospy

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D
from pathplan_uncertainty.msg import Int32TimeStep, Pose2DTimeStep
from pathplan_uncertainty.srv import GroundType
from dt_simulator.world import World, DroveOffTheFreakinRoad, RammedAFreakinDuckiebot 


class SimNode(object):

    def __init__(self):        
        # Parameters
        self.dt = rospy.get_param("/sim/dt")

        self.road_width = rospy.get_param("/sim/world/road/width")

        self.world_params = {"road_width": self.road_width}

        self.our_duckie_start_pose = rospy.get_param("/sim/our_duckie/start_pose")
        self.our_duckie_velocity = rospy.get_param("/sim/our_duckie/velocity")
        self.our_duckie_radius = rospy.get_param("/sim/our_duckie/radius")
        self.our_duckie_type = rospy.get_param("/sim/our_duckie/type")

        self.our_duckie_params = {"start_pose": self.our_duckie_start_pose, "velocity": self.our_duckie_velocity, "radius": self.our_duckie_radius, "type": self.our_duckie_type}


        self.other_duckie_start_pose = rospy.get_param("/sim/other_duckie/start_pose")
        self.other_duckie_velocity = rospy.get_param("/sim/other_duckie/velocity")
        self.other_duckie_radius = rospy.get_param("/sim/other_duckie/radius")
        self.other_duckie_type = rospy.get_param("/sim/other_duckie/type")

        self.other_duckie_params = {"start_pose": self.other_duckie_start_pose, "velocity": self.other_duckie_velocity, "radius": self.other_duckie_radius, "type": self.other_duckie_type}

      
        # World
        self.world = World(self.dt, self.our_duckie_params, self.other_duckie_params, self.world_params)

        # Publishers
        self.pub_pose_our_duckie = rospy.Publisher("/sim/gt/pose_our_duckie",Pose2DTimeStep, queue_size=1)
        self.pub_pose_other_duckie = rospy.Publisher("/sim/gt/pose_other_duckie",Pose2DTimeStep, queue_size=1)
        self.pub_safety_status = rospy.Publisher("/sim/gt/our_duckie_safety_status",Int32TimeStep, queue_size=1)
        self.pub_our_duckie_ground_type = rospy.Publisher("/sim/gt/our_duckie_ground_type",Int32TimeStep, queue_size=1)

        self.pub_pose_our_duckie_obs = rospy.Publisher("/sim/obs/pose_our_duckie",Pose2DTimeStep, queue_size=1)
        self.pub_pose_other_duckie_obs = rospy.Publisher("/sim/obs/pose_other_duckie",Pose2DTimeStep, queue_size=1)
        
        # Subscribers
        self.sub_agent_orientation_seq = rospy.Subscriber("/agent/orientation_seq",Float32MultiArray, self.orientation_seq_cb)
        self.sub_agent_computation_time_steps = rospy.Subscriber("/agent/computation_time_steps", Int32, self.computation_time_cb)

        # Services
        self.srv_ground_type = rospy.Service('get_ground_type',GroundType, self.ground_type_srv_cb)


        # Temporary variables
        self.orientation_seq = []
        self.received_comp_time = False
        self.computation_time_steps = 10
        self.received_ori_seq = False

        rospy.loginfo("[SimNode] Initialized.")


    def orientation_seq_cb(self, ori_seq_msg):
        rospy.loginfo("[SimNode] Received orientation sequence.")
        self.orientation_seq = ori_seq_msg.data
        self.received_ori_seq = True
        if self.received_comp_time:
            self.publish_obs()
            self.propagate_action()

    def computation_time_cb(self, int_msg):
        rospy.loginfo("[SimNode] Received computation time.")
        self.computation_time_steps = int_msg.data
        self.received_comp_time = True
        if self.received_ori_seq:
            self.publish_obs()
            self.propagate_action()

    def publish_obs(self):
        rospy.loginfo("[SimNode] Publishing observations.")

        # Publishing the observations
        time, ourd_p, _, _, othd_p = self.world.get_state()

        # Publish our duckie pose
        ourd_p_msg = Pose2DTimeStep()
        ourd_p_msg.time = time
        ourd_p_msg.x = ourd_p[0]
        ourd_p_msg.y = ourd_p[1]
        ourd_p_msg.theta = ourd_p[2]
        self.pub_pose_our_duckie_obs.publish(ourd_p_msg)

        # Publish other duckie pose
        othd_p_msg = Pose2DTimeStep()
        othd_p_msg.time = time
        othd_p_msg.x = othd_p[0]
        othd_p_msg.y = othd_p[1]
        othd_p_msg.theta = othd_p[2]
        self.pub_pose_our_duckie_obs.publish(othd_p_msg)

    def publish_state(self):
        rospy.loginfo("[SimNode] Publishing state.")

        # Publishing the state
        time, ourd_p, ourd_ss, ourd_gt, othd_p = self.world.get_state()

        # Publish our duckie pose
        ourd_p_msg = Pose2DTimeStep()
        ourd_p_msg.time = time
        ourd_p_msg.x = ourd_p[0]
        ourd_p_msg.y = ourd_p[1]
        ourd_p_msg.theta = ourd_p[2]
        self.pub_pose_our_duckie.publish(ourd_p_msg)

        # Publish our duckie safety status
        ourd_ss_msg = Int32TimeStep()
        ourd_ss_msg.time = time
        ourd_ss_msg.data = ourd_ss.value
        self.pub_safety_status.publish(ourd_ss_msg)

        # Publish our duckie ground type
        ourd_gt_msg = Int32TimeStep()
        ourd_gt_msg.time = time
        ourd_gt_msg.data = ourd_gt.value
        self.pub_our_duckie_ground_type.publish(ourd_gt_msg)

        # Publish other duckie pose
        othd_p_msg = Pose2DTimeStep()
        othd_p_msg.time = time
        othd_p_msg.x = othd_p[0]
        othd_p_msg.y = othd_p[1]
        othd_p_msg.theta = othd_p[2]
        self.pub_pose_other_duckie.publish(othd_p_msg)

    def propagate_action(self):
        rospy.loginfo("[SimNode] Propagation actions: ")

        # Propagate action in world
        self.world.update_our_duckie_plan(self.orientation_seq)

        for k in range(self.computation_time_steps):
            rospy.loginfo("[SimNode] k = " + str(k + 1))
            self.world.step()
            self.publish_state()


        # Reset temp. variables
        rospy.loginfo("[SimNode] Reset of temp. variables.")

        self.orientation_seq = []
        self.received_ori_seq = False
        self.received_comp_time = False
        pass

    def ground_type_srv_cb(self, req_msg):
        pose = (req_msg.x.data, req_msg.y.data, req_msg.theta.data)
        radius = req_msg.bot_radius.data
        response = Int32()
        rospy.loginfo("[SimNode] ground type: " + str(self.world.check_ground(pose, radius).value))
        response.data = self.world.check_ground(pose, radius).value
        return response

    def onShutdown(self):
        rospy.loginfo("[SimNode] Shutdown.")



if __name__ == '__main__':
    rospy.init_node('sim_node',anonymous=False)
    sim_node = SimNode()
    rospy.on_shutdown(sim_node.onShutdown)
    rospy.spin()