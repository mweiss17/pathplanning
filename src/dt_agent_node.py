#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from pathplan_uncertainty.msg import Pose2DTimeStep, Observation, AgentCommand
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
        self.pub_agent_command = rospy.Publisher("/agent/command", AgentCommand, queue_size = 5)

        # Subscribers
        self.sub_observations = rospy.Subscriber("/sim/obs/observations", Observation, self.observation_cb)
        
        # Agent
        self.agent = Agent(self.road_width, self.horizon_length)


        rospy.loginfo("[AgentNode] Initialized.")

        rospy.sleep(1)
        
        rospy.loginfo("[AgentNode] Starting the process.")
        self.start_process()



    def observation_cb(self, obs_msg):
        time = obs_msg.our_duckie_pose.time
        self.compute_our_plan(time)


    def compute_our_plan(self, time):
        rospy.loginfo("[AgentNode] Compute our plan at time " + str(time))

        plan = [0, 0, 0, 0, 0]
        timesteps = 5

        self.publish_plan(plan, timesteps)

    def publish_plan(self, plan, timesteps):
        command_msg = AgentCommand()

        plan_msg = Float32MultiArray()
        plan_msg.data = plan
        command_msg.orientation_seq = plan_msg

        command_msg.computation_time_steps = timesteps

        self.pub_agent_command.publish(command_msg)

    def start_process(self):
        plan = []
        timesteps = 1

        self.publish_plan(plan, timesteps)


    def onShutdown(self):
        rospy.loginfo("[AgentNode] Shutdown.")


if __name__ == '__main__':
    rospy.init_node('agent_node',anonymous=False)
    agent_node = AgentNode()
    rospy.on_shutdown(agent_node.onShutdown)
    rospy.spin()