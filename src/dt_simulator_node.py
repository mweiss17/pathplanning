#!/usr/bin/env python

import rospy

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from pathplan_uncertainty.msg import Int32TimeStep, Pose2DTimeStep, WorldState
from pathplan_uncertainty.srv import GroundType
from dt_simulator.world import World, DroveOffTheFreakinRoad, RammedAFreakinDuckiebot 
from dt_simulator.visualizer import Visualizer
from cv_bridge import CvBridge, CvBridgeError


class SimNode(object):

    def __init__(self):        
        # Utils
        self.bridge = CvBridge()

        # Simulation parameters
        self.dt = rospy.get_param("/sim/dt")
        self.dt_in_sim = rospy.get_param("/sim/dt_in_sim")
        self.sim_parameters = {"dt": self.dt, "dt_in_sim": self.dt_in_sim}

        self.output_image = rospy.get_param("/sim/image/output_image")
        self.image_height = rospy.get_param("/sim/image/height")
        self.image_width = rospy.get_param("/sim/image/width")
        self.image_m2pix = rospy.get_param("/sim/image/m2pix")
        self.image_y_baseline = rospy.get_param("/sim/image/y_baseline")

        self.image_params = {"output_image": self.output_image, "height": self.image_height, "width": self.image_width, "m2pix": self.image_m2pix, "y_baseline": self.image_y_baseline}

        ## World parameters
        self.road_width = rospy.get_param("/sim/world/road/width")

        self.world_params = {"road_width": self.road_width}

        ## Our duckie parameters
        self.our_duckie_start_pose = rospy.get_param("/sim/our_duckie/start_pose")
        self.our_duckie_velocity = rospy.get_param("/sim/our_duckie/velocity")
        self.our_duckie_radius = rospy.get_param("/sim/our_duckie/radius")
        self.our_duckie_type = rospy.get_param("/sim/our_duckie/type")
        self.our_duckie_angle_change_limit = rospy.get_param("/sim/our_duckie/angle_change_limit")

        self.our_duckie_params = {"start_pose": self.our_duckie_start_pose, "velocity": self.our_duckie_velocity, "radius": self.our_duckie_radius, "type": self.our_duckie_type, "angle_change_limit": self.our_duckie_angle_change_limit}

        ## Other duckie parameters
        self.other_duckie_start_pose = rospy.get_param("/sim/other_duckie/start_pose")
        self.other_duckie_velocity = rospy.get_param("/sim/other_duckie/velocity")
        self.other_duckie_radius = rospy.get_param("/sim/other_duckie/radius")
        self.other_duckie_type = rospy.get_param("/sim/other_duckie/type")
        self.other_duckie_angle_change_limit = rospy.get_param("/sim/other_duckie/angle_change_limit")

        self.other_duckie_params = {"start_pose": self.other_duckie_start_pose, "velocity": self.other_duckie_velocity, "radius": self.other_duckie_radius, "type": self.other_duckie_type, "angle_change_limit": self.other_duckie_angle_change_limit}
      
        # World
        self.world = World(self.sim_parameters, self.world_params, self.our_duckie_params, self.other_duckie_params)
        # Visualizer
        self.visualizer = Visualizer(self.image_params, self.world_params, self.our_duckie_params, self.other_duckie_params)

        # Publishers
        self.pub_pose_our_duckie = rospy.Publisher("/sim/gt/pose_our_duckie",Pose2DTimeStep, queue_size=1)
        self.pub_pose_other_duckie = rospy.Publisher("/sim/gt/pose_other_duckie",Pose2DTimeStep, queue_size=1)
        self.pub_safety_status = rospy.Publisher("/sim/gt/our_duckie_safety_status",Int32TimeStep, queue_size=1)
        self.pub_our_duckie_ground_type = rospy.Publisher("/sim/gt/our_duckie_ground_type",Int32TimeStep, queue_size=1)
        self.pub_world_state = rospy.Publisher("sim/gt/world_state", WorldState, queue_size=1)

        self.pub_pose_our_duckie_obs = rospy.Publisher("/sim/obs/pose_our_duckie",Pose2DTimeStep, queue_size=1)
        self.pub_pose_other_duckie_obs = rospy.Publisher("/sim/obs/pose_other_duckie",Pose2DTimeStep, queue_size=1)

        self.pub_image = rospy.Publisher("/sim/road_image", Image, queue_size=1)
        
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
        self.orientation_seq = ori_seq_msg.data
        self.received_ori_seq = True
        if self.received_comp_time:
            self.publish_obs()
            self.propagate_action()

    def computation_time_cb(self, int_msg):
        self.computation_time_steps = int_msg.data
        self.received_comp_time = True
        if self.received_ori_seq:
            self.publish_obs()
            self.propagate_action()

    def publish_obs(self):
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

        # Publish all together
        world_state_msg = WorldState()
        world_state_msg.time = time
        world_state_msg.our_duckie_pose = ourd_p_msg
        world_state_msg.other_duckie_pose = othd_p_msg
        world_state_msg.safety_status = ourd_ss.value
        world_state_msg.ground_type = ourd_gt.value
        self.pub_world_state.publish(world_state_msg)

        # Publish image
        if self.output_image:
            image = self.visualizer.create_image_state(ourd_p, othd_p)
            image_msg_out = self.bridge.cv2_to_imgmsg(image, "bgr8")
            self.pub_image.publish(image_msg_out)


    def propagate_action(self):
        # Propagate action in world
        self.world.update_our_duckie_plan(self.orientation_seq)

        for k in range(self.computation_time_steps):
            rospy.sleep(self.dt_in_sim)
            self.world.step()
            self.publish_state()


        # Reset temp. variables
        self.orientation_seq = []
        self.received_ori_seq = False
        self.received_comp_time = False
        pass

    def ground_type_srv_cb(self, req_msg):
        # Returns the ground type for a given pose and radius (see dt_comm/enums for ground_types)
        pose = (req_msg.x.data, req_msg.y.data, req_msg.theta.data)
        radius = req_msg.bot_radius.data
        response = Int32()
        response.data = self.world.check_ground(pose, radius).value
        return response

    def onShutdown(self):
        rospy.loginfo("[SimNode] Shutdown.")



if __name__ == '__main__':
    rospy.init_node('sim_node',anonymous=False)
    sim_node = SimNode()
    rospy.on_shutdown(sim_node.onShutdown)
    rospy.spin()