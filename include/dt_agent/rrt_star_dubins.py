"""
code modified from :
Path Planning Sample Code with RRT and Dubins path (author: AtsushiSakai(@Atsushi_twi)) (MIT license)
"""

import random
import math
import copy
import numpy as np
import dubins_path_planning
import matplotlib.pyplot as plt

show_animation = True


class RRT():

    def __init__(self, start, goal, obstacleList, randAreax, randAreay, gridsizex, gridsizey, radius,
                 goalSampleRate=10, maxIter=60):


        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]

        """
        self.start = Node(start[0], start[1], start[2])
        self.end = Node(goal[0], goal[1], goal[2])

        self.minrandx = randAreax[0]
        self.maxrandx = randAreax[1]
        self.minrandy = randAreay[0]
        self.maxrandy = randAreay[1]

        self.gridsizex = gridsizex
        self.gridsizey = gridsizey

        self.robotradius = radius

        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList

    def setValues(self, start, goal, obstacleList, randAreax, randAreay, gridsizex, gridsizey, radius,
                 goalSampleRate=10, maxIter=50):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]

        """
        self.start = Node(start[0], start[1], start[2])
        self.end = Node(goal[0], goal[1], goal[2])

        self.minrandx = randAreax[0]
        self.maxrandx = randAreax[1]
        self.minrandy = randAreay[0]
        self.maxrandy = randAreay[1]

        self.gridsizex = gridsizex
        self.gridsizey = gridsizey

        self.robotradius = radius

        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList

    def Planning(self, animation=False):

        self.nodeList = [self.start]
        for i in range(self.maxIter):
            if(i % 5 == 0):
                self.goalSampleRate += 3
            rnd = self.get_random_point() 
            nind = self.GetNearestListIndex(self.nodeList, rnd)

            newNode = self.steer(rnd, nind)
            #  print(newNode.cost)

            if self.CollisionCheck(newNode, self.obstacleList):
                nearinds = self.find_near_nodes(newNode)
                newNode = self.choose_parent(newNode, nearinds)
                self.nodeList.append(newNode)
                self.rewire(newNode, nearinds)

            if animation and i % 5 == 0:
                self.DrawGraph(rnd=rnd)

        # generate coruse
        lastIndex = self.get_best_last_index()
        #  print(lastIndex)

        if lastIndex is None:
            return None

        path = self.gen_final_course(lastIndex)
        return path

    def choose_parent(self, newNode, nearinds):                             ##using cost to decide parent
        if len(nearinds) == 0:
            return newNode

        dlist = []
        for i in nearinds:
            tNode = self.steer(newNode, i)
            if self.CollisionCheck(tNode, self.obstacleList):
                dlist.append(tNode.cost)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        minind = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            print("mincost is inf")
            return newNode

        newNode = self.steer(newNode, minind)

        return newNode

    def pi_2_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def steer(self, rnd, nind):                                             ##comuting cost based on path length 
        #  print(rnd)
        curvature = 1.0

        nearestNode = self.nodeList[nind]

        px, py, pyaw, mode, clen = dubins_path_planning.dubins_path_planning(
            nearestNode.x, nearestNode.y, nearestNode.yaw, rnd.x, rnd.y, rnd.yaw, curvature)

        newNode = copy.deepcopy(nearestNode)
        newNode.x = px[-1]
        newNode.y = py[-1]
        newNode.yaw = pyaw[-1]

        newNode.path_x = px
        newNode.path_y = py
        newNode.path_yaw = pyaw
        newNode.cost += clen
        newNode.parent = nind

        return newNode

    def get_random_point(self):

        if random.randint(0, 100) > self.goalSampleRate:
            rnd = [random.uniform(self.minrandx, self.maxrandx),
                   random.uniform(self.minrandy, self.maxrandy),
                   random.uniform(-math.pi, math.pi)
                   ]
        else:  # goal point sampling
            rnd = [self.end.x, self.end.y, self.end.yaw]

        node = Node(rnd[0], rnd[1], rnd[2])

        return node

    def get_best_last_index(self):                                                          ##using cost to get best node
        #  print("get_best_last_index")

        YAWTH = np.deg2rad(1.0)
        XYTH = 0.5

        goalinds = []
        for (i, node) in enumerate(self.nodeList):
            if self.calc_dist_to_goal(node.x, node.y) <= XYTH:
                goalinds.append(i)

        # angle check
        fgoalinds = []
        for i in goalinds:
            if abs(self.nodeList[i].yaw - self.end.yaw) <= YAWTH:
                fgoalinds.append(i)

        if len(fgoalinds) == 0:
            return None

        mincost = min([self.nodeList[i].cost for i in fgoalinds])
        for i in fgoalinds:
            if self.nodeList[i].cost == mincost:
                return i

        return None

    def gen_final_course(self, goalind):
        path = [[self.end.x, self.end.y]]
        while self.nodeList[goalind].parent is not None:
            node = self.nodeList[goalind]
            for (ix, iy) in zip(reversed(node.path_x), reversed(node.path_y)):
                path.append([ix, iy])
            #  path.append([node.x, node.y])
            goalind = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def calc_dist_to_goal(self, x, y):
        return np.linalg.norm([x - self.end.x, y - self.end.y])

    def find_near_nodes(self, newNode):
        nnode = len(self.nodeList)
        r = 50.0 * math.sqrt((math.log(nnode) / nnode))
        #  r = self.expandDis * 5.0
        dlist = [(node.x - newNode.x) ** 2 +
                 (node.y - newNode.y) ** 2 +
                 (node.yaw - newNode.yaw) ** 2
                 for node in self.nodeList]
        nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]
        return nearinds

    def rewire(self, newNode, nearinds):                                            #using cost to compare and rewire

        nnode = len(self.nodeList)

        for i in nearinds:
            nearNode = self.nodeList[i]
            tNode = self.steer(nearNode, nnode - 1)

            obstacleOK = self.CollisionCheck(tNode, self.obstacleList)
            imporveCost = nearNode.cost > tNode.cost

            if obstacleOK and imporveCost:
                #  print("rewire")
                self.nodeList[i] = tNode

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

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd.x) ** 2 +
                 (node.y - rnd.y) ** 2 +
                 (node.yaw - rnd.yaw) ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))

        return minind

    def CollisionCheck(self, node, obstacleList):                                   #extend later

        for (ox, oy, size) in obstacleList:
            for (ix, iy) in zip(node.path_x, node.path_y):
                dx = ox - ix
                dy = oy - iy
                d = dx * dx + dy * dy
                if d <= size ** 2:
                    return False  # collision

        return True  # safe


class Node():                                                                   ##initing cost to zero (can init instead to cost for inital metrics acc to node position)

    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.path_x = []
        self.path_y = []
        self.path_yaw = []
        self.cost = 0.0
        self.parent = None

    def get_cost_lane_dev():
        pass


#center of lane
#on road (radius of car)
#no collisions
#shortest distance


##add radius to collision checks



