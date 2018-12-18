#!/usr/bin/env python

import rospy

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from pathplan_uncertainty.msg import Pose2DTimeStep
from pathplan_uncertainty.srv import GroundType
from dt_agent.agent import Agent

class AgentNode(object):
    def __init__(self):  

        # Parameters 

        ## Time
        self.dt = rospy.get_param("/sim/dt")

        ## World parameters
        self.road_width = rospy.get_param("/sim/world/road/width")

        ## Agent parameters
        self.horizon_length = rospy.get_param("/agent/horizon_length")


        #Publishers
        self.pub_agent_orientation_seq = rospy.Publisher("/agent/orientation_seq",Float32MultiArray, queue_size=1)
        self.pub_agent_computation_time_steps = rospy.Publisher("/agent/computation_time_steps", Int32, queue_size=1)
        
        # Subscribers
        self.sub_pose_other_duckie_obs = rospy.Subscriber("/sim/obs/pose_other_duckie",Pose2DTimeStep, self.other_duckie_pose_cb)
        self.sub_pose_our_duckie_obs = rospy.Subscriber("/sim/obs/pose_our_duckie",Pose2DTimeStep, self.our_duckie_pose_cb)
        
        # Agent
        self.agent = Agent(self.road_width, self.horizon_length)

        self.duckie_poses = {}

        rospy.loginfo("[AgentNode] Initialized.")



    def other_duckie_pose_cb(self, duckie_pose_msg):   
        rospy.loginfo("[AgentNode] Received other duckie's obs.")
        time = duckie_pose_msg.time
        if time in self.duckie_poses:
            self.duckie_poses[time]["other_duckie"] = (duckie_pose_msg.x, duckie_pose_msg.y, duckie_pose_msg.theta)
            self.compute_our_plan(time)
        else:
            self.duckie_poses[time] = {}
            self.duckie_poses[time]["other_duckie"] = (duckie_pose_msg.x, duckie_pose_msg.y, duckie_pose_msg.theta)


    def our_duckie_pose_cb(self, duckie_pose_msg):   
        rospy.loginfo("[AgentNode] Received our duckie's obs.")
        time = duckie_pose_msg.time

        if time in self.duckie_poses:
            self.duckie_poses[time]["our_duckie"] = (duckie_pose_msg.x, duckie_pose_msg.y, duckie_pose_msg.theta)
            self.compute_our_plan(time)
        else:
            self.duckie_poses[time] = {}
            self.duckie_poses[time]["our_duckie"] = (duckie_pose_msg.x, duckie_pose_msg.y, duckie_pose_msg.theta)

    def compute_our_plan(self, time):
        our_duckie_pose = self.duckie_poses[time]["our_duckie"]
        other_duckie_pose = self.duckie_poses[time]["other_duckie"]
        rospy.loginfo("[AgentNode] Compute our plan at time " + str(time))
        #plan, timesteps = self.agent.compute_our_plan(our_duckie_pose, other_duckie_pose)

        plan = [0, 0, 0, 0, 0]
        timesteps = 5

        del self.duckie_poses[time]
        self.publish_plan(plan, timesteps)

    def publish_plan(self, plan, timesteps):
        plan_msg = Float32MultiArray()
        plan_msg.data = plan
        self.pub_agent_orientation_seq.publish(plan_msg)

        timesteps_msg = Int32()
        timesteps_msg.data = timesteps
        self.pub_agent_computation_time_steps.publish(timesteps_msg)





        #self.othduckie_obs = othduckie_obs_msg.data


    #TODO: do world coord transformation for other duckie if needed since we are always at 0,0 

    #plan = self.agent.compute_our_plan([self.othduckie_obs])

    #publish seq of actions and time steps
    #self.pub_agent_orientation_seq.publish(plan)
    #self.pub_agent_computation_time_steps.publish()  #how to compute

    def onShutdown(self):
        rospy.loginfo("[AgentNode] Shutdown.")


if __name__ == '__main__':
    rospy.init_node('agent_node',anonymous=False)
    agent_node = AgentNode()
    rospy.on_shutdown(agent_node.onShutdown)
    rospy.spin()