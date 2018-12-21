import logging
import rospy
import numpy as np
import math

import time


class PredictorDiscretePropagation(object):
    def __init__(self, agent_params, sim_params):

        # Parameters
        self.time_horizon = float(agent_params["time_horizon"])
        self.y_resolution = agent_params["y_resolution"]
        self.y_horizon = agent_params["y_horizon"]
        self.vel_resolution = agent_params["vel_resolution"]
 

        self.dt = sim_params["dt"]
        self.road_width = sim_params["road_width"]
        self.other_duckie_type = sim_params["other_duckie_type"]
        self.other_duckie_max_acceleration = sim_params["other_duckie_max_acceleration"]
        self.other_duckie_max_velocity = sim_params["other_duckie_max_velocity"]
        self.other_duckie_min_velocity = sim_params["other_duckie_min_velocity"]

        # Build the velocity changes list
        self.max_vel_change = self.other_duckie_max_acceleration * self.dt          # How much can we change in a time step, in + or -
        self.number_vel_points = int(self.max_vel_change/self.vel_resolution)*2 + 1 # How many discrete points do we model
        self.prob_of_each_vel_change = 1.0/self.number_vel_points                   # Using uniform distribution: probability of each step
        eff_max_vel_change = self.vel_resolution*(self.number_vel_points-1)/2       # Effective maximum velocity change (different from max_vel_change if max_vel_change not multiple of vel_resolution)
        self.vel_changes = []
        for k in range(self.number_vel_points):
            self.vel_changes.append(-eff_max_vel_change + k*self.vel_resolution)    # Start from -eff_max_vel_change and go up to +eff_max_vel_change passing by 0



        self.prediction = {}
        self.time = 0

    def predict(self, obs_msg):
        start = time.time()
        # Reading message
        self.time = round(obs_msg.our_duckie_pose.time, 2)

        other_duckie_x = obs_msg.other_duckie_pose.x
        other_duckie_y = obs_msg.other_duckie_pose.y
        other_duckie_theta = obs_msg.other_duckie_pose.theta
        other_duckie_pose = (other_duckie_x, other_duckie_y, other_duckie_theta)
        other_duckie_velocity = obs_msg.other_duckie_velocity
        other_duckie_radius = obs_msg.other_duckie_radius


        #####
        # Prediction Structure:
        # prediction is a dictionary with int(time*100) as a key (in order to keep it an int)
        # For each time, it contains a list.
        # Each element of the list represents a discretization of y, going from 0 to y_horizon with a resolution of y_resolution
        # It has a value a list of 2-lists corresponding to the different velocities possible at this position
        # Each of duo has as a first element the probability for the other duckie to be at the y value with a velocity given by the second element of the duo.
        #####

        # Reinitializing prediction
        rospy.loginfo("[Agent][Predictor] Predicting...")
        number_time_steps = int(self.time_horizon/self.dt)
        number_y_steps = int(self.y_horizon/self.y_resolution)
        self.prediction = {}
        init_prob = [1, other_duckie_velocity]
        self.add_in_prediction(self.time, other_duckie_y, init_prob)
        prev_time = self.time

        # Propagating prediction (4rth degree for loop...)

        for k_t in range(1, number_time_steps):             # For each time step
            #rospy.loginfo("[Agent][Predictor] Time step: " + str(k_t))
            cur_time = prev_time + self.dt                       # Compute current time
            for k_y in range(0, number_y_steps):                     # For each y step
                y = k_y*self.y_resolution                               # Compute current y
                for y_element in self.get_from_prediction(prev_time, y):     # For each element that existed at y in previous time
                    el_prob = y_element[0]                                   # Probability of being there previously
                    el_vel = y_element[1]                                           # With this velocity
                    for vel_change in self.vel_changes:                                 # For each possible change of velocity
                        next_vel = el_vel+vel_change                                        # Next velocity
                        if next_vel < self.other_duckie_min_velocity:                                                        # Checking limits
                            next_vel = self.other_duckie_min_velocity
                        elif next_vel > self.other_duckie_max_velocity:
                            next_vel = self.other_duckie_max_velocity
                        next_vel = ((1000 * next_vel)//1) /1000                             # Limiting the number of decimals of vel to 3
                        next_y = y + next_vel*self.dt*math.cos(other_duckie_theta)          # Next y
                        next_prob = el_prob*self.prob_of_each_vel_change                    # Next probability
                        prob_vel_duo = [next_prob, next_vel]              
                        self.add_in_prediction(cur_time, next_y, prob_vel_duo)            # Add in the prediction structure
            prev_time = cur_time

        end = time.time()
        rospy.loginfo("[Agent][Predictor] Prediction time: " + str(end - start))

    def add_in_prediction(self, time, y, prob_vel_duo):
        if time >= self.time and time <= self.time + self.time_horizon and y < self.y_horizon and y >= 0: # Only do something if in the time and space limits

            time_key = int(round(time,2)*100)
            if time_key not in self.prediction:                         # If time not in prediction yet, instantiate the new list at this time
                number_y_steps = int(self.y_horizon/self.y_resolution)+1
                self.prediction[time_key] = [[] for x in range(number_y_steps)] 

            y_index = int(round(y/self.y_resolution))                   # Compute index for y 

            values_before = self.prediction[time_key][y_index]          # Get the that was there before

            already_there = False
            for element in values_before:                               # Check for each element if
                if abs(element[1] - prob_vel_duo[1]) <= 0.02 and not already_there:           # there is already a probability at this velocity
                    element[0] += prob_vel_duo[0]                       # If so, add probability
                    already_there = True
            if not already_there:                                       # Otherwise, add probability/velocity duo to the list
                self.prediction[time_key][y_index].append(prob_vel_duo)

    def get_from_prediction(self, time, y):
        time_key = int(round(time,2)*100)
        if time_key in self.prediction and y <= self.y_horizon and y >= 0:
            y_index = int(round(y/self.y_resolution))
            return self.prediction[time_key][y_index]
        else:
            return []

    def get_probability(self, time, y):
        if time < self.time:
            rospy.loginfo("[Agent][Predictor] Trying to get probability for a time that has already passed: " + str(time))
            return 0

        time_key = int(round(time,2)*100)
        time_key_temp = time_key
        while time_key_temp not in self.prediction and time_key - time_key_temp <= 10:
            time_key_temp = time_key_temp - 1
            if time_key_temp in self.prediction:
                time_key = time_key_temp

        if time_key in self.prediction and y <= self.y_horizon and y >= 0:
            y_index = int(round(y/self.y_resolution))
            list_of_prob_vel_duos = self.prediction[time_key][y_index]
            probability = 0
            for prob_vel_duo in list_of_prob_vel_duos:
                probability += prob_vel_duo[0]
            return probability
        else:
            return 0

