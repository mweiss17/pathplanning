#!/usr/bin/env python
#original: https://github.com/haroldsultan/MCTS/blob/master/mcts.py
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
import time

costMat = np.zeros((100,100,100))

#MCTS scalar.  Larger scalar will increase exploitation, smaller will increase exploration
SCALAR=1.5/math.sqrt(2.0)  			

class mctsPlanner():

	def __init__(self, predictor, agent_params, sim_params, our_duckie_params, reward_params):

		self.predictor = predictor
		self.dt = sim_params["dt"]

		self.time_horizon = agent_params["time_horizon"]
		self.road_width = sim_params["road_width"]
		self.v = our_duckie_params["velocity"]
		self.radius = our_duckie_params["radius"]
		self.reward_params = reward_params
		# self.number_time_steps = int(self.time_horizon/self.dt)

		self.number_time_steps = 20


	def computePlan(self, goal, obs_msg):
		self.goal = goal
		levels = 1
		current_node = Node(State(turn = self.number_time_steps))

		current_node.state.x = obs_msg.our_duckie_pose.x
		current_node.state.y = obs_msg.our_duckie_pose.y
		current_node.state.cum_angle = obs_msg.our_duckie_pose.theta

		self.startx = current_node.state.x
		self.starty = current_node.state.y
		self.start_time = round(obs_msg.our_duckie_pose.time, 2)

		self.start_theta = current_node.state.cum_angle

		for l in range(levels):
			current_node=self.UCTSEARCH(2500,current_node)
		return self.bestPath(current_node)

	def bestPath(self, node):
		path = []
		angles = []
		moves = []
		collision = []
		rewards = []
		visits = []
		lost_candidates = []
		lost_can_collisions = []
		while node is not None:
			path.append([node.state.x, node.state.y]) 
			angles.append(node.state.cum_angle)
			collision.append(node.state.collision_cost)
			rewards.append(node.reward/(node.visits *1.0) )
			moves = node.state.moves
			visits.append(node.visits)
			node, candidates, lost_collisions = self.BESTCHILD_FINAL(node, 0)
			lost_candidates.append(candidates)	
			lost_can_collisions.append(lost_collisions)

		return path, angles, moves, collision, rewards, visits, lost_candidates, lost_can_collisions

	def UCTSEARCH(self, budget, root):

		for iter in range(int(budget)):
				# rospy.loginfo(root)
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
					node = self.BESTCHILD(node, SCALAR)
		return node


	def EXPAND(self, node):						
		tried_children = [c.state for c in node.children]
		tried_moves = [c.state.moves[-1] for c in node.children]
		valid = list(set(node.state.MOVES).difference(tried_moves))
		new_state = self.next_state(node.state, moves = valid)

		node.add_child(new_state)
		return node.children[-1]

	def BESTCHILD_FINAL(self, node, scalar):
		bestscore = -9999999
		bestchildren = []
		candidate_rewards = []
		lost_can_collisions = []
		for c in node.children:
			exploit = c.reward/c.visits  
			candidate_rewards.append(exploit)
			lost_can_collisions.append(c.state.collision_cost)
			explore = math.sqrt(2.0*math.log(node.visits)/float(c.visits))	

			score = exploit+scalar*explore
			if score == bestscore:
				bestchildren.append(c)
			if score>bestscore:

				bestchildren = [c]
				bestscore = score
		# print('length of bestchildren array : '+str(len(bestchildren)))
		if len(bestchildren) == 0:
			# print('children count '+str(len(node.children)))
			# rospy.loginfo("no best child found, probably fatal")
			return None, [], []
		return random.choice(bestchildren), candidate_rewards, lost_can_collisions

	def BESTCHILD(self, node, scalar):
		bestscore = -9999999
		bestchildren = []
		for c in node.children:

			exploit = c.reward/(c.visits *1.0) 
			explore = math.sqrt(2.0*math.log(node.visits)/float(c.visits))	

			score = exploit+scalar*explore
			if score == bestscore:
				bestchildren.append(c)
			if score>bestscore:

				bestchildren = [c]
				bestscore = score
		if len(bestchildren) == 0:
			# print('children count '+str(len(node.children)))
			rospy.loginfo("no best child found, probably fatal")
			return None
		return random.choice(bestchildren)


	def centerlane_incentive(self, state):
		incentive = abs(state.x)
		return -incentive


	def forward_incentive(self, state):

		diffy = state.y - self.starty
		incentive = diffy

		return incentive

	def goal_reward(self, state):
		dist_to_goal = abs(state.x-self.startx) + abs(state.y-self.starty) 
		#transform to an appropriate value
		return 0

	def get_collision_cost(self, state):
		collision_cost = self.predictor.get_collision_probability(state.x, state.y, self.start_time+((self.number_time_steps - state.turn)*self.dt), self.radius)
		return collision_cost*(-1000)

	def GETREWARD(self, state):		

		ground_type = self.check_ground(state)
		# print(ground_type)
		ground_reward = self.reward_params[ground_type]
		collision_cost = self.get_collision_cost(state)
		state.collision_cost = collision_cost
		reward = ground_reward + collision_cost + self.forward_incentive(state) + self.centerlane_incentive(state)

		return reward 

	def BACKUP(self, node, reward):
		# print('backing up node at x y '+str(node.state.x) + ' ' + str(node.state.y))
		while node != None:
			node.visits += 1
			node.reward += reward			
			node = node.parent
		return

	def next_state(self, state, moves =[]):
		if(random.uniform(0,1)<.1):
			nextmove = 0
		else:
			if(len(moves)!=0):
				nextmove = random.choice(moves)					
			else:
				print('ERROR should not happen')
				time.sleep(100)
				nextmove = random.choice(state.MOVES)	
		nextstate = State(state.moves+[nextmove], state.turn-1) 
		nextstate.cum_angle = state.cum_angle + nextmove

		nextstate.x = state.x + ((self.v * self.dt) * math.sin(nextstate.cum_angle ))  
		nextstate.y = state.y + ((self.v * self.dt) /(1.0 * math.cos(nextstate.cum_angle)))


		# if(nextstate.y < state.y):
		# 	print('y '+str(nextstate.y))
		# 	print('y is less cause angle is : '+str(nextstate.cum_angle))

		# 	time.sleep(6)
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
		self.MOVES = [-0.2, 0,  0.2]
		self.cum_angle = 0 
		self.x = 0
		self.y = 0
		self.num_moves = len(self.MOVES)
		self.collision_cost = 0

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
		s = "Moves: %s "%(self.moves)
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
