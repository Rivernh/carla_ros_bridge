#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
from os.path import join, dirname
sys.path.insert(0, join(dirname(__file__), '../'))

import math
import numpy as np
from utils.cubic_spline import Spline2D
from utils.coordinate import frenet_to_cartesian1D
from scipy.spatial import KDTree

class Map:
    def __init__(self, center_lines, borders, tx, ty, csp):
        self.center_lines = center_lines
        self.borders = borders
        self.tx = tx
        self.ty = ty
        self.csp = csp

class FreNetMap:
    def __init__(self, rx, ry, ryaw, rk, rdk, csp, rs):
        self.rx = rx
        self.ry = ry
        self.ryaw = ryaw
        self.rk = rk
        self.rdk = rdk
        self.csp = csp
        self.rs = rs
        
        self.kdtree = KDTree(np.array([rx, ry]).T)

def generate_frenet_map(x, y):
    rx, ry, ryaw, rk, rdk, csp, s = generate_target_course2(x, y)
    return FreNetMap(rx, ry, ryaw, rk, rdk, csp, s)
    
def generate_target_course(x, y):
    csp = Spline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, csp

def generate_target_course2(x, y):
    csp = Spline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk, rdk = [], [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))
        rdk.append(csp.calc_d_curvature(i_s))

    return rx, ry, ryaw, rk, rdk, csp, s

def create_lane_border(ref_line, width):
    tx, ty, tyaw, tc, csp = generate_target_course(ref_line[:, 0], ref_line[:, 1])

    border = []

    s = np.arange(0, csp.s[-1], 0.1)
    for i_s in range(len(s)):
        s_condition = [s[i_s]]
        d_condition = [width]
        lx, ly = frenet_to_cartesian1D(s[i_s], tx[i_s], ty[i_s], tyaw[i_s], s_condition, d_condition)
        border.append([lx, ly])

    return np.array(border)

def create_sim_map():
    # center_line = np.array([[0.0, 1.0], [50.0, 10.0], [100, -10.0], [150.0, 15], [200, -15.0]])
    center_lines = []
    borders = []

    center_line = np.array([[0.0, 10.0], [100.0, 0.0], [205, 50.0], [350.0, 65], [705, 0.0]])

    tx, ty, _, _, csp = generate_target_course(center_line[:, 0], center_line[:, 1])
    border_l = [-1.7, 1.7, 5.1, 8.5]
    center_l = [3.4, 6.8]
    for i in range(len(border_l)):
        border = create_lane_border(center_line, border_l[i])
        borders.append(border)
    for i in range(len(center_l)):
        center = create_lane_border(center_line, center_l[i])
        center_lines.append(center)
    
    return Map(center_lines, borders, tx, ty, csp)

class Vehicle():
    def __init__(self, csp, s, l, velocity):
        self._velocity = velocity
        self._csp = csp

        self._s = s
        self._l = l

        self._w = 2.0
        self._h = 4.8

    def update(self, dt):
        self._s += self._velocity * dt

    def length(self):
        return self._h

    def width(self):
        return self._w

    def position(self):
        ix, iy = self._csp.calc_position(self._s)

        return (ix - self._h / 2.0, iy - self._w / 2.0)

    def s(self):
        return self._s

    def s_d(self):
        return self._velocity

    def s_dd(self):
        return 0.0

    def rect(self):

        if self._s < self._csp.s[-1] - 0.1:
            ix, iy = self._csp.calc_position(self._s)
            heading = self._csp.calc_yaw(self._s)

            s_condition = [self._s]
            d_condition = [self._l]
            cx, cy = frenet_to_cartesian1D(self._s, ix, iy, heading, s_condition, d_condition)
        else:
            return None


        cos, sin = np.cos(heading), np.sin(heading)

        vertex = lambda e1, e2: (
            cx + (e1 * self._h * cos + e2 * self._w * sin) / 2.0,
            cy + (e1 * self._h * sin - e2 * self._w * cos) / 2.0
        )

        vertices = [vertex(*e) for e in [(-1, -1), (-1, 1), (1, 1), (1, -1)]]

        vertices.append(vertices[0])

        return np.array(vertices)


    def yaw(self):
        heading = self._csp.calc_yaw(self._s)

        return math.degrees(heading)
    
class EgoState:
    def __init__(self, c_d=0, c_d_d=0, c_d_dd=0, s0=0, s0_d=0, s0_dd=0, c_speed=0):
        self.c_d = c_d # current lateral position [m]
        self.c_d_d = c_d_d # current lateral speed [m/s]
        self.c_d_dd = c_d_dd  # current lateral acceleration [m/ss]
        self.s0 = s0 # current course position
        self.s0_d = s0_d # current course speed [m/s]
        self.s0_dd = s0_dd # current course acceleration [m/ss]
        self.c_speed = c_speed # current speed [m/s]
        
    def sim_update(self, path):
        self.c_d = path.d[1]
        self.c_d_d = path.d_d[1]
        self.c_d_dd = path.d_dd[1]
        self.s0 = path.s[1]
        self.s0_d = path.s_d[1]
        self.s0_dd = path.s_dd[1]
        self.c_speed = path.s_d[1]

    def get_state(self):
        return self.s0, self.s0_d, self.s0_dd, self.c_d, self.c_d_d, self.c_d_dd, self.c_speed