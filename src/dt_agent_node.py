import rospy

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D
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
        self.sub_pose_other_duckie_obs = rospy.Subscriber("/sim/obs/pose_other_duckie",Pose2D, self.compute_ourplan_cb)
        
        self.othduckie_obs = Pose2D()
        self.received_othduckie_obs = False
        self.agent = Agent(self.road_width, self.horizon_length)

def compute_ourplan_cb(self, othduckie_obs_msg):   
    rospy.loginfo("[SimNode] Received other duckie's obs.")
    self.othduckie_obs = othduckie_obs_msg.data
    self.received_othduckie_obs = True
    #TODO: do world coord transformation for other duckie if needed since we are always at 0,0 

    plan = self.agent.compute_our_plan([self.othduckie_obs])

    #publish seq of actions and time steps
    self.pub_agent_orientation_seq.publish(plan)
    self.pub_agent_computation_time_steps.publish()  #how to compute
