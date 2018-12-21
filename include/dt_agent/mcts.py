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


#todo:
#call P(x,y,t) function to get collsion prob
#gradually decrease exploitation
#decide how to define terminal states

#to make sure:
#	don't surpass goal
#	prefer shorter straighter path
#	keep track of x-closeness to goal as well as y



#pass in goal
#need a way to take in our own position relative to road
# what is vel_resolution
# how to access v

class mctsPanner():

    def __init__(self, predictor, agent_params, sim_params, reward_params):

    	self.predictor = predictor
    	self.dt = sim_params["dt"]
    	self.v = v

		self.time_horizon = agent_params["time_horizon"]
        self.road_width = sim_params["road_width"]
        self.velocity = our_duckie_params["/duckiebots/our_duckie/velocity"]
        self.radius = our_duckie_params["/duckiebots/our_duckie/radius"]
        self.reward_params = reward_params
        self.number_time_steps = int(self.time_horizon/self.dt)

        #self.number_y_steps = int(self.y_horizon/self.y_resolution)
        
        # self.y_resolution = agent_params["y_resolution"]
        # self.vel_resolution = agent_params["vel_resolution"]
        # self.comp_time_mean = agent_params["comp_time_mean"]
        # self.comp_time_std_dev = agent_params["comp_time_std_dev"]

        # self.other_duckie_type = sim_params["other_duckie_type"]
        # self.other_duckie_max_acceleration = sim_params["other_duckie_max_acceleration"]


    # onroad = true/false
    # lane = 1,-1 (1 is for the right(correct) lane)
    # pos_wrt_midlane = +- position depending on which side
    # need more info about lane widths, radius etc
    def computePlan(goal):

    	#might want to call predict function first where it populates the probability DS

    	self.goal = goal
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
		while node is not None:		 
			print("Best Child: %s"%node.state)
			node = self.BESTCHILD(node, 0):


	def UCTSEARCH(self, budget,root):
		for iter in range(int(budget)):
			if iter%10000 == 9999:
				rospy.loginfo("simulation: %d"%iter)
				rospy.loginfo(root)
			front = self.TREEPOLICY(root)
			reward = self.GETREWARD(front.state) 
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

	def get_collision_cost(self):
		collsion_cost = self.predictor.get_probability()
		return collsion_cost

	def GETREWARD(self,state):		
		# while state.terminal() == False:
		# 	state = self.next_state(state)
		ground_type = self.check_ground(state)
		ground_reward = reward_params[ground_type]
		reward = ground_reward + self.get_collision_cost()
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
		nextstate.x = state.y + ((self.v * self.dt) * math.sin(nextmove))  
		nextstate.y = state.x + ((self.v * self.dt) /(1.0 * math.cos(nextmove)))
		nextstate.cum_angle = state.cum_angle + nextmove
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
	self.NUM_TURNS = 20	#pull time horison from config instead
	self.MOVES = [-0.2, 0, 0.2]
	self.cum_angle = 0   #cumulative angle from start
	self.x = 0
	self.y = 0
	num_moves = len(MOVES)

	def __init__(self, moves = [], turn = NUM_TURNS):
		self.turn = turn
		self.moves = moves


	def terminal(self):
		if self.turn == 0:
			return True
		return False

	# def reward(self, ground_type):
	# 	#this function needs to be populated depending on implementation and output of probability prediction module
	# 	#should basically take this x,y position -> convert to gridcell indices -> get reward at the index


	# 	r = self.get_road_losses() + self.get_collision_cost()
	# 	return r

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
