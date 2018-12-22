#!/usr/bin/env python
import math
import hashlib
import rospy
import logging
import argparse
import numpy as np
import matplotlib.pyplot as plt
import random
import scipy.stats as stats
from dt_comm.enums import Ground

costMat = np.zeros((100,100,100))

#MCTS scalar.  Larger scalar will increase exploitation, smaller will increase exploration. 
SCALAR=1/math.sqrt(2.0)  			

#gradually decrease exploitation

#to make sure:
#	don't surpass goal
#	prefer shorter straighter path
#	keep track of x-closeness to goal as well as y


class mctsPlanner():

	def __init__(self, predictor, agent_params, sim_params, our_duckie_params, reward_params):

		self.predictor = predictor
		self.dt = sim_params["dt"]

		self.time_horizon = agent_params["time_horizon"]
		self.road_width = sim_params["road_width"]
		self.v = our_duckie_params["velocity"]
		self.radius = our_duckie_params["radius"]
		self.reward_params = reward_params
		self.number_time_steps = int(self.time_horizon/self.dt)

        #self.number_y_steps = int(self.y_horizon/self.y_resolution)
        
        # self.y_resolution = agent_params["y_resolution"]
        # self.vel_resolution = agent_params["vel_resolution"]
        # self.comp_time_mean = agent_params["comp_time_mean"]
        # self.comp_time_std_dev = agent_params["comp_time_std_dev"]

        # self.other_duckie_type = sim_params["other_duckie_type"]
        # self.other_duckie_max_acceleration = sim_params["other_duckie_max_acceleration"]


	def computePlan(self, goal, obs_msg):

		self.goal = goal
		levels = 1
		current_node = Node(State())
		current_node.x = obs_msg.our_duckie_pose.x
		current_node.y = obs_msg.our_duckie_pose.y
		current_node.cum_angle = obs_msg.our_duckie_pose.theta

		for l in range(levels):
			current_node=self.UCTSEARCH(100,current_node)
			print("level %d"%l)
			print("Num Children: %d"%len(current_node.children))
			for i,c in enumerate(current_node.children):
				print(i,c)
			print("state: %s"%current_node.state)
		return self.bestPath(current_node)

	def bestPath(self, node):
		path = []
		angles = []
		moves = []
		while node is not None:
			path.append([node.state.x, node.state.y]) ##or just add cum_angle
			angles.append(node.state.cum_angle)
			moves = node.state.moves
			print("Best Child: %s"%node.state)
			node = self.BESTCHILD(node, 0)
		return path, angles, moves

	def UCTSEARCH(self, budget, root):
		for iter in range(int(budget)):
			if iter%10 == 9999:
				rospy.loginfo("simulation: %d"%iter)
				rospy.loginfo(root)
			front = self.TREEPOLICY(root)
			reward = self.GETREWARD(front.state) 
			self.BACKUP(front,reward)
		return root	
		# return self.BESTCHILD(root,0)

	def TREEPOLICY(self, node):
		#a hack to force 'exploitation' in a game where there are many options, and you may never/not want to fully expand first
		while node.state.terminal() == False:
			if len(node.children) == 0:
				return self.EXPAND(node)
			elif random.uniform(0,1)<.5:	
				node = self.BESTCHILD(node,SCALAR)
			else:
				if node.fully_expanded() == False:	
					return self.EXPAND(node)
				else:
					node = self.BESTCHILD(node,SCALAR)
		return node


	def EXPAND(self, node):						
		tried_children = [c.state for c in node.children]
		new_state = self.next_state(node.state)
		while new_state in tried_children:
			new_state = self.next_state(node.state)
		node.add_child(new_state)
		return node.children[-1]

	def BESTCHILD(self, node, scalar):
		bestscore = 0.0
		bestchildren = []
		for c in node.children:

			exploit = c.reward/c.visits  
			explore = math.sqrt(2.0*math.log(node.visits)/float(c.visits))	

			score = exploit+scalar*explore
			if score == bestscore:
				bestchildren.append(c)
			if score>bestscore:
				bestchildren = [c]
				bestscore = score
		if len(bestchildren) == 0:
			rospy.loginfo("no best child found, probably fatal")
			return None
		return random.choice(bestchildren)

	def get_collision_cost(self, state):
		collsion_cost = self.predictor.get_collision_probability(state.x, state.y, self.number_time_steps - state.turn ,self.radius)
		return collsion_cost

	def GETREWARD(self,state):		
		# while state.terminal() == False:
		# 	state = self.next_state(state)
		ground_type = self.check_ground(state)
		ground_reward = self.reward_params[ground_type]
		reward = ground_reward + self.get_collision_cost(state)
		return reward 

	def BACKUP(self, node,reward):
		while node != None:
			node.visits += 1
			node.reward += reward			
			node = node.parent
		return

	def next_state(self, state):
		nextmove = random.choice(state.MOVES)	
		nextstate = State(state.moves+[nextmove], state.turn-1) 
		nextstate.cum_angle = state.cum_angle + nextmove

		nextstate.x = state.y + ((self.v * self.dt) * math.sin(nextstate.cum_angle ))  
		nextstate.y = state.x + ((self.v * self.dt) /(1.0 * math.cos(nextstate.cum_angle)))
		return nextstate

	def check_ground(self, state):		
	    # Returns the type of ground for a duckie pose
	    x = state.x
	    if abs(x) <= 0.25*self.road_width:
	        return Ground.RIGHT_LANE
	    elif x < -0.25*self.road_width and x >= -0.75*self.road_width :
	        return Ground.WRONG_LANE
	    elif (x > -0.75*self.road_width - self.radius and x < -0.75*self.road_width) or (x > 0.25*self.road_width and x < 0.25*self.road_width + self.radius):
	        return Ground.PARTIALLY_OUT_OF_ROAD
	    else:
	        return Ground.LOST

class State():

	def __init__(self, moves = [], turn = 20):
		self.turn = turn
		self.moves = moves
		self.MOVES = [-0.2, 0, 0.2]
		self.cum_angle = 0   #cumulative angle from start
		self.x = 0
		self.y = 0
		self.num_moves = len(self.MOVES)

	def terminal(self):
		if self.turn == 0:
			return True
		return False

	def __hash__(self):
		return int(hashlib.md5(str(self.moves).encode('utf-8')).hexdigest(),16)
	def __eq__(self,other):
		if hash(self) == hash(other):
			return True
		return False
	def __repr__(self):
		s = "Moves: %s"%(self.moves)
		return s
	

class Node():
	def __init__(self, state, parent=None):
		self.visits = 1
		self.reward = 0.0	
		self.state = state
		self.children = []
		self.parent = parent	

	def add_child(self,child_state):
		child = Node(child_state,self)
		self.children.append(child)
	def update(self,reward):
		self.reward += reward
		self.visits += 1
	def fully_expanded(self):
		if len(self.children) == self.state.num_moves:
			return True
		return False
	def __repr__(self):
		s = "Node; children: %d; visits: %d; reward: %f"%(len(self.children),self.visits,self.reward)
		return s
