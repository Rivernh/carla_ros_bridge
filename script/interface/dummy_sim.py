#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np

class AckermannSteeringModel:
    def __init__(self, config, x0, y0, theta0, v=0, delta=0, dv=0, ddelta=0): # L:wheel base
        self.x = x0 # X
        self.y = y0 # Y
        self.theta = theta0 # headding
        self.v = v # speed
        self.delta = delta # front wheels' angle
        self.dv = dv
        self.ddelta = ddelta
        
        self.config = config
        self.l = config.get('vehicle').get('L')  # wheel base
        self.dt = config.get('dt')  # decision time periodic
        self.min_v = config.get('vehicle').get('min_v')
        self.max_v = config.get('vehicle').get('max_v')
        self.min_delta = config.get('vehicle').get('min_delta')
        self.max_delta = config.get('vehicle').get('max_delta')
        
    def update(self, vt, delta): 
        self.v = vt

        dx = self.v * np.cos(self.theta)
        dy = self.v * np.sin(self.theta)
        dtheta = self.v * np.tan(delta) / self.l

        self.x += dx * self.dt
        self.y += dy * self.dt
        self.theta += dtheta * self.dt
        
    def step(self, action):  # update ugv's state
        self.v = np.clip(action[0], self.min_v, self.max_v)
        self.delta = np.clip(action[1], self.min_delta, self.max_delta)
        
        dx = self.v * np.cos(self.theta)
        dy = self.v * np.sin(self.theta)
        dtheta = self.v * np.tan(self.delta) / self.l

        self.x += dx * self.dt
        self.y += dy * self.dt
        self.theta += dtheta * self.dt
    
    def dstep(self, daction, dv=None):
        self.dv = daction[0]
        self.ddelta = daction[1]
        
        if dv is None:
            action = [self.v, 0.0] + daction
        else:
            action = [self.v+dv, daction[1]]
            
        action = [
            np.clip(action[0], self.min_v, self.max_v),
            np.clip(action[1], self.min_delta, self.max_delta)
        ]
        self.step(action)
        return action
    
    def get_pose(self):
        return np.array([self.x, self.y, self.theta])
    
    def get_dynamic(self):
        return np.array([self.v, self.delta])
    
    def get_control(self):
        """
        get last control (d_action)
        """
        return np.array([self.dv, self.ddelta])