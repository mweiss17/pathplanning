import rospy

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D
from pathplan_uncertainty.srv import GroundType
from dt_agent.agent import Agent

class AgentNode(object):
    def __init__(self):  
        # Parameters 
        self.dt = rospy.get_param("/dt_agent_node/dt")
        self.roadwidth = rospy.get_param("/dt_agent_node/road/width")
        self.horizon_length = rospy.get_param("/dt_agent_node/road/horizon_length")


        # Subscribers
        self.sub_pose_other_duckie_obs = rospy.Subscriber("/sim/obs/pose_other_duckie",Pose2D, self.compute_ourplan_cb)
        self.othduckie_obs = Pose2D()
        self.received_othduckie_obs = False
        self.agent = Agent(self.roadwidth, self.horizon_length)

def compute_ourplan_cb(self, othduckie_obs_msg):   
    rospy.loginfo("[SimNode] Received other duckie's obs.")
    self.othduckie_obs = othduckie_obs_msg.data
    self.received_othduckie_obs = True
    # do world coord transformation for other duckie if needed since we are always at 0,0 


    plan = self.agent.compute_our_plan([self.othduckie_obs])
    #publish seq of actions and time steps