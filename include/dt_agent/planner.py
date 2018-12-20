import numpy as np
import math
import rospy
import matplotlib.pyplot as plt
from dt_agent.rrt_star_dubins_planner import RRT
from dt_agent.motion_models import driving_towards_us

class Planner(object):
    def __init__(self):
        pass

class RRT_Dubins(Planner):
    def __init__(self, gridsizex, gridsizey):
        super(RRT, self).__init__()
        self.start =  [0.0, 0.0, np.deg2rad(0.0)]
        self.goal =  [0.0, 12.0, np.deg2rad(0.0)]
        self.time_steps = 20
        self.obstacle_model = driving_towards_us(dt, velx, vely)                         ##define these here or in caller (assumng constant velocity)

        self.gridsizex = gridsizex
        self.gridsizey = gridsizey

        self.static_obstacleList = []  # [x,y,size(radius)]                        ##for now is empty initially, and is populated when other_duckie obs is received
                                        
        self.rrt = RRT(self.start, self.goal, randAreax=[-gridsizex/2.0, gridsizex/2.0], randAreay=[0.0, gridsizey], obstacleList = self.static_obstacleList)
       


    def update_plan(other_duckie_obs, radii):                                     #for now only xy is important in obstacle list, will factor in theta later
        
        self.static_obstacleList = []                                            ##might not want to clear every time  (ex if other bot hasn't moved)
        for obs, radius in zip(other_duckie_obs,radii):

            rolledout_pos = self.obstacle_model.rollout_model(obs, radius, time_steps = self.time_steps) #should be transformed coordinates 
            self.static_obstacleList.append(rolledout_pos)                      #rolledout_pos is a list of list of [x,y,radius] for each 'other' duckie

        self.rrt.setValues(self.start, self.goal,randAreax=[-gridsizex/2.0, gridsizex/2.0], randAreay=[0.0, gridsizey], obstacleList = self.static_obstacleList)  #optional, nothing really changes
        path = self.rrt.Planning()
        final_path = postprocess_plan(path)
        orientation_seq = convert_to_thetas(final_path)
        return orientation_seq
    
    def postprocess_plan(path):
        return path

    def convert_to_thetas(xy_points):
        angles = []
        for point_itr in range(len(xy_points)-1):

            angles.append(math.atan2(xy_points[point_itr+1][1]-xy_points[point_itr][1], xy_points[point_itr+1][0]-xy_points[point_itr][0]))
        return angles

    def generate_goal_for_rrt():                                        ## later, for now coded to '12' units distance units ahead in center of lane
                                                                        ## should be checking for whether theres an obstacle at that point or not?
        pass

def DrawGraph(self, rnd=None):
    """
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




