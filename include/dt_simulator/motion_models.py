import numpy as np
import math
import rospy

class MotionModel(object):
	def __init__(self, start_pos, dt):
        self.dt = dt
        self.start_pos = start_pos

    def rollout_model(self):
        raise NotImplementedError

class driving_towards_us(MotionModel):

	def __init__(self, start_pos, dt, velx, vely, y_offset):
		MotionModel.__init__(self, start_pos, dt)
		self.vely = vely
		self.velx = velx
		self.y_offset = y_offset

    def rollout_model(self, time_steps):
    	positions = []
    	for t in range(time_steps):
    		positions.append([self.start_pos.x + (self.velx * t * self.dt),self.start_pos.y + (self.vely * t * self.dt)])
    	return positions



class zigzag_towards_us(MotionModel):

	def __init__(self, start_pos, dt, velx, vely, y_offset):
		MotionModel.__init__(self, start_pos, dt)
		self.vely = vely
		self.velx = velx
		self.y_offset = y_offset

    def rollout_model(self, time_steps):
    	positions = []
    	for t in range(time_steps):
    		if( t % 3 ==0):
    			velx = -velx
    		positions.append([self.start_pos.x + (self.velx * t * self.dt),self.start_pos.y + (self.vely * t * self.dt)])
    	return positions


class gaussian_in_place(MotionModel):

	def __init__(self, start_pos, dt, mean, std_dev):
		MotionModel.__init__(self, start_pos, dt)
		self.mean = mean
		self.std_dev = std_dev

    def rollout_model(self, time_steps):							## implement
    	positions = []
    	for t in range(time_steps):
    		positions.append([])
    	return positions