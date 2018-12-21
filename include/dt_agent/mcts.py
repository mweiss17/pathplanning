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

costMat = np.zeros((100,100,100))

#MCTS scalar.  Larger scalar will increase exploitation, smaller will increase exploration. 
SCALAR=1/math.sqrt(2.0)  			


#todo:
#call P(x,y,t) function to get collsion prob
#gradually decrease exploitation
#decide how to define terminal states

#to make sure:
#	don't surpass goal
#	prefer shorter straighter path
#	keep track of x-closeness to goal as well as y

class mctsPanner():

    def __init__(self, predictor, dt):

    	self.predictor = predictor
    	self.dt = dt
    	self.v = v

    def computePlan():
    	levels = 3
		current_node = Node(State())
		for l in range(levels):
			current_node=self.UCTSEARCH(args.num_sims/(l+1),current_node)
			print("level %d"%l)
			print("Num Children: %d"%len(current_node.children))
			for i,c in enumerate(current_node.children):
				print(i,c)
			print("Best Child: %s"%current_node.state)
	

	def bestPath(self, node):
		while node is not None:					## or replace with another terminal condition 
			print("Best Child: %s"%node.state)
			node = self.BESTCHILD(node, 0):


	def UCTSEARCH(self, budget,root):
		for iter in range(int(budget)):
			if iter%10000 == 9999:
				rospy.loginfo("simulation: %d"%iter)
				rospy.loginfo(root)
			front = self.TREEPOLICY(root)
			reward = self.DEFAULTPOLICY(front.state) 
			self.BACKUP(front,reward)
		return self.BESTCHILD(root,0)

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

	def DEFAULTPOLICY(self,state):
		while state.terminal() == False:
			state = self.next_state(state)
		return state.reward()  

	def BACKUP(self, node,reward):
	    while node != None:
			node.visits += 1
			node.reward += reward			
			node = node.parent
		return

	def next_state(self, state):
		nextmove = random.choice(state.MOVES)	
		nextstate = State(state.moves+[nextmove], state.turn-1) 
		nextstate.x = state.y + ((self.v * self.dt) * math.sin(nextmove))  
		nextstate.y = state.x + ((self.v * self.dt) /(1.0 * math.cos(nextmove)))
		nextstate.cum_angle = state.cum_angle + nextmove
		return nextstate


class State():
	self.NUM_TURNS = 20	#pull from config instead
	self.MOVES = [-0.2, 0, 0.2]
	self.cum_angle = 0   #cumulative angle from start
	self.x = 0
	self.y = 0
	num_moves = len(MOVES)

	def __init__(self, moves = [], turn = NUM_TURNS):
		self.turn = turn
		self.moves = moves


	#right and left sides fo the road differentiate
	#compute position related reward : wrt center line, road, center of lane
	def get_road_losses(self):
		return 0

	def get_collision_cost(self):
		self.predictor.get_probability()
		return 0

	def get_goal_reward(self):
		return 0

	def pos_rel_lane(self):
		return 0

	def pos_rel_centerline(self):
		return 0

	def pos_rel_sidewalk(self):
		#same side sidewalk penalise less
		#make sure radius-constraints are satisfied
		return 0

	def pos_rel_goal(self):
		return 0

	def terminal(self):
		if self.turn == 0:
			return True
		return False

	def reward(self):
		#this function needs to be populated depending on implementation and output of probability prediction module
		#should basically take this x,y position -> convert to gridcell indices -> get reward at the index
		r = self.get_road_losses() + self.get_collision_cost()
		return r

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
