import numpy as np
import math
import rospy
import matplotlib.pyplot as plt
from .rrt_star_dubins import RRT
from .motion_models import driving_towards_us

class Planner(object):
	def __init__(self):
		pass

class RRT_Dubins(Planner):
	def __init__(self, gridsizex, gridsizey):
		super(RRT, self).__init__()
		self.start =  [0.0, 0.0, np.deg2rad(0.0)]
		self.goal =  [0.0, 12.0, np.deg2rad(0.0)]
		self.current
		self.time_steps
		self.obstacle_model = driving_towards_us() 						##argss
		self.gridsizex = gridsizex
		self.gridsizey = gridsizey

    	self.static_obstacleList = [										##for now
							        # (0, 10, 1),
							        # (3, 6, 2),
							    	]  # [x,y,size(radius)]		


	    self.rrt = RRT(start, goal, randAreax=[-gridsizex/2.0, gridsizex/2.0], randAreay=[0.0, gridsizey], obstacleList=obstacleList)
	    #grid

	def new_obs_received():

	def update_plan(other_duckie_obs, radii): #for now only xy is important in obstacle list, will factor in theta later
		obstacle_list = []
		for obs, radius in zip(other_duckie_obs,radii):
			obstacle_list.append(obs.x, obs.y, radius)  
		self.rrt.setValues(obs)
		path = self.rrt.Planning

	def postprocess_plan():



		

	def generate_goal_for_rrt():										##later, for now coded to 12 distance units ahead in center of lane
		pass

def DrawGraph(self, rnd=None):
    u"""
    Draw Graph
    """
    plt.clf()
    if rnd is not None:
        plt.plot(rnd.x, rnd.y, "^k")
    for node in self.nodeList:
        if node.parent is not None:
            plt.plot(node.path_x, node.path_y, "-g")
            #  plt.plot([node.x, self.nodeList[node.parent].x], [
            #  node.y, self.nodeList[node.parent].y], "-g")

    for (ox, oy, size) in self.obstacleList:
        plt.plot(ox, oy, "ok", ms=30 * size)

    dubins_path_planning.plot_arrow(
        self.start.x, self.start.y, self.start.yaw)
    dubins_path_planning.plot_arrow(
        self.end.x, self.end.y, self.end.yaw)

    plt.axis([-2, 15, -2, 15])
    plt.grid(True)
    plt.pause(0.01)

    #  plt.show()
    #  input()



