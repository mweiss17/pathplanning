#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Pose2D
from dt_simulator.manager import Manager
#from dt_simulator.manager import Manager
from dt_simulator.world import World, DroveOffTheFreakinRoad, RammedAFreakinDuckiebot 


class SimNode(object):

    def __init__(self):
	    self.manager = Manager()
	    self.world = World(self.manager)
	    rospy.loginfo("[SimNode] Initialized.")
        self.pose_duckie_msg = rospy.Publisher("~wheels_cmd_executed",Pose2D, queue_size=1)


    def onShutdown(self):
        rospy.loginfo("[SimNode] Shutdown again.")



if __name__ == '__main__':
    rospy.init_node('sim_node',anonymous=False)
    sim_node = SimNode()
    rospy.on_shutdown(sim_node.onShutdown)
    rospy.spin()