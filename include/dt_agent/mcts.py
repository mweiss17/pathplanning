#!/usr/bin/env python
import math
import hashlib
import logging
import argparse
import numpy as np
import matplotlib.pyplot as plt
import random
import scipy.stats as stats

# truncated gaussian
# lower, upper = -0.2, 0.2
# mu, sigma = 0, 0.2
# X = stats.truncnorm(
#     (lower - mu) / sigma, (upper - mu) / sigma, loc=mu, scale=sigma)

costMat = np.zeros((100,100,100))

#MCTS scalar.  Larger scalar will increase exploitation, smaller will increase exploration. 
SCALAR=1/math.sqrt(2.0)  			

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger('MyLogger')


#todo:
#store accumulated orientations/ global position
#time component, index and check with indexed cells
#gradually decrease exploitation

class State():
	self.NUM_TURNS = 20	#pull from config instead
	self.MOVES=[-0.2, 0, 0.2]
	self.cum_angle = 0   #cumulative angle from start
	num_moves=len(MOVES)

	def __init__(self, moves=[], turn=NUM_TURNS):
		self.turn=turn
		self.moves=moves

	def next_state(self):
		nextmove = random.choice(self.MOVES)
		nextstate = State(self.moves+[nextmove],self.turn-1) 
		return nextstate

	def terminal(self):
		if self.turn == 0:
			return True
		return False

	def reward(self):
		#this function needs to be populated depending on implementation and output of probability prediction module
		#should basically take this x,y position -> convert to gridcell indices -> get reward at the index
		r = 0
		return r

	def snap():
		pass

	def __hash__(self):
		return int(hashlib.md5(str(self.moves).encode('utf-8')).hexdigest(),16)
	def __eq__(self,other):
		if hash(self)==hash(other):
			return True
		return False
	def __repr__(self):
		s="Moves: %s"%(self.moves)
		return s
	

class Node():
	def __init__(self, state, parent=None):
		self.visits=1
		self.reward=0.0	
		self.state=state
		self.children=[]
		self.parent=parent	
	def add_child(self,child_state):
		child=Node(child_state,self)
		self.children.append(child)
	def update(self,reward):
		self.reward+=reward
		self.visits+=1
	def fully_expanded(self):
		if len(self.children)==self.state.num_moves:
			return True
		return False
	def __repr__(self):
		s="Node; children: %d; visits: %d; reward: %f"%(len(self.children),self.visits,self.reward)
		return s
		


def UCTSEARCH(budget,root):
	for iter in range(int(budget)):
		if iter%10000==9999:
			rospy.logger.info("simulation: %d"%iter)
			logger.info(root)
		front=TREEPOLICY(root)
		reward=DEFAULTPOLICY(front.state) 
		BACKUP(front,reward)
	return BESTCHILD(root,0)

def TREEPOLICY(node):
	#a hack to force 'exploitation' in a game where there are many options, and you may never/not want to fully expand first
	while node.state.terminal()==False:
		if len(node.children)==0:
			return EXPAND(node)
		elif random.uniform(0,1)<.5:	
			node=BESTCHILD(node,SCALAR)
		else:
			if node.fully_expanded()==False:	
				return EXPAND(node)
			else:
				node=BESTCHILD(node,SCALAR)
	return node


def EXPAND(node):						
	tried_children=[c.state for c in node.children]
	new_state=node.state.next_state()
	while new_state in tried_children:
		new_state=node.state.next_state()
	node.add_child(new_state)
	return node.children[-1]

def BESTCHILD(node,scalar):
	bestscore=0.0
	bestchildren=[]
	for c in node.children:

		exploit=c.reward/c.visits  
		explore=math.sqrt(2.0*math.log(node.visits)/float(c.visits))	

		score=exploit+scalar*explore
		if score==bestscore:
			bestchildren.append(c)
		if score>bestscore:
			bestchildren=[c]
			bestscore=score
	if len(bestchildren)==0:
		logger.warn("no best child found, probably fatal")
	return random.choice(bestchildren)

def DEFAULTPOLICY(state):
	while state.terminal()==False:
		state=state.next_state()
	return state.reward()  

def BACKUP(node,reward):
	while node!=None:
		node.visits+=1
		node.reward+=reward			
		node=node.parent
	return


parser.add_argument('--num_sims', action="store", required=True, type=int)
parser.add_argument('--levels', action="store", required=True, type=int, choices=range(State.NUM_TURNS))

current_node=Node(State())
for l in range(args.levels):
	current_node=UCTSEARCH(args.num_sims/(l+1),current_node)
	print("level %d"%l)
	print("Num Children: %d"%len(current_node.children))
	for i,c in enumerate(current_node.children):
		print(i,c)
	print("Best Child: %s"%current_node.state)
	
	print("--------------------------------")	
			
	



