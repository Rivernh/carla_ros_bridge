#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
from os.path import join, dirname
sys.path.insert(0, join(dirname(__file__), '../'))

import copy
import numpy as np
import math
from utils.quintic_poly import QuinticPolynomial
from utils.quartic_poly import QuarticPolynomial
from utils.coordinate import frenet_to_cartesian3D

def my_caculate(a, b, p):  # 判断p在ab上的投影是否在线段上 0不在 1在 2在端点
    a = np.array(a)
    b = np.array(b)
    p = np.array(p)
    v = b - a
    u = p - a
    count = 0
    if ((np.dot(v, u) / np.dot(v, v)) >= 0) and ((np.dot(v, u) / np.dot(v, v)) <= 1):
        return True
    return False

def overlap(box1,box2):#加入已知矩形四个顶点，检测是否碰撞
    crash = True
    #box1
    for i in range(4):
        a = box1[i]
        b = box1[(i + 1) % 4]
        cnt = 0
        for point in box2:
            if my_caculate(a,b,point):
                break
            cnt += 1
        if cnt == 4:
            crash = False
            return crash
    #box2
    for i in range(4):
        a = box2[i]
        b = box2[(i + 1) % 4]
        cnt = 0
        for point in box1:
            if my_caculate(a,b,point):
                break
            cnt += 1
        if cnt == 4:
            crash = False
            return crash
    return crash

def inside_range(p,box):#加入在规划轨迹一定范围内的的检测，便于后续加入steer维度的动作
    #判断当前坐标是否在一个矩形区域内
    cnt = 0
    #box1
    for i in range(4):
        a = box[i]
        b = box[(i + 1) % 4]
        if my_caculate(a, b, p):
            cnt += 1
    if cnt >= 4:
        return True
    return False

# 判断点在矩形内
def MyCross(p1,p2,p):
    return (p2[0] - p1[0]) * (p[1] - p1[1]) -(p[0] - p[0]) * (p2[1] - p1[1])
def inside_rectangle(p,box):
    p1, p2, p3, p4 = box
    return (MyCross(p1, p2, p) * MyCross(p3, p4, p) >= 0) and (MyCross(p2, p3, p) * MyCross(p4, p1, p) >= 0)

class FrenetPath:
    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        self.cd = 0.0
        self.cv = 0.0
        self.cf = 0.0

        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.a = []
        self.c = []

        self.mode = None

class LatticePlanner:
    def __init__(self, config):
        self.config = config
        self.pl_cfg = self.config.lattice_planner

    def calc_following_path(self, c_d, c_d_d, c_d_dd, s0, s0_d, s0_dd, s1, s1_d, s1_dd):
        frenet_paths = []

        # generate path to each offset goal
        for di in np.arange(self.pl_cfg.MIN_ROAD_WIDTH, self.pl_cfg.MAX_ROAD_WIDTH, self.pl_cfg.d_road_w):

            # Lateral motion planning
            for Ti in np.arange(2.0, 5.0, self.pl_cfg.dt):
                fp = FrenetPath()

                lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

                fp.t = [t for t in np.arange(0.0, Ti, self.pl_cfg.dt)]
                fp.d = [lat_qp.calc_point(t) for t in fp.t]
                fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
                fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
                fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

                for delta_s in [s1 - s0]:

                    s_target_dd =  s1_dd
                    s_target_d =  s1_d
                    s_target = s0 + delta_s

                    tfp = copy.deepcopy(fp)
                    lon_qp = QuinticPolynomial(s0, s0_d, s0_dd, s_target, s_target_d, s_target_dd, Ti)

                    tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                    tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                    tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                    tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                    Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                    Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk
                    ds = (tfp.s[-1] - s0) ** 2


                    tfp.cd = self.pl_cfg.K_J * Jp + self.pl_cfg.K_T * 10 * Ti + self.pl_cfg.K_D * tfp.d[-1] ** 2
                    tfp.cv = self.pl_cfg.K_J * Js + self.pl_cfg.K_T * 10 * Ti + self.pl_cfg.K_D * ds
                    tfp.cf = self.pl_cfg.K_LAT * tfp.cd + self.pl_cfg.K_LON * tfp.cv

                    frenet_paths.append(tfp)

        return frenet_paths

    def calc_frenet_paths(self, target_speed, c_speed, c_d, c_d_d, c_d_dd, s0):
        frenet_paths = []

        # generate path to each offset goal
        for di in np.arange(self.pl_cfg.MIN_ROAD_WIDTH, self.pl_cfg.MAX_ROAD_WIDTH, self.pl_cfg.d_road_w):

            # Lateral motion planning
            for Ti in np.arange(self.pl_cfg.min_t, self.pl_cfg.max_t, self.pl_cfg.dt):
                fp = FrenetPath()

                lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

                fp.t = [t for t in np.arange(0.0, Ti, self.pl_cfg.dt)]
                fp.d = [lat_qp.calc_point(t) for t in fp.t]
                fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
                fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
                fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

                # Longitudinal motion planning (Velocity keeping)
                for tv in np.arange(target_speed - self.pl_cfg.d_speed * self.pl_cfg.n_s_sample,
                                    target_speed + self.pl_cfg.d_speed * self.pl_cfg.n_s_sample, self.pl_cfg.d_speed):
                    tfp = copy.deepcopy(fp)
                    lon_qp = QuarticPolynomial(s0, c_speed, 0.0, tv, 0.0, Ti)

                    tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                    tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                    tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                    tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                    Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                    Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                    # square of diff from target speed
                    ds = (target_speed - tfp.s_d[-1]) ** 2

                    tfp.cd = self.pl_cfg.K_J * Jp + self.pl_cfg.K_T * Ti + self.pl_cfg.K_D * tfp.d[-1] ** 2
                    tfp.cv = self.pl_cfg.K_J * Js + self.pl_cfg.K_T * Ti + self.pl_cfg.K_D * ds
                    tfp.cf = self.pl_cfg.K_LAT * tfp.cd + self.pl_cfg.K_LON * tfp.cv

                    frenet_paths.append(tfp)
        #print(len(frenet_paths))
        return frenet_paths

    def calc_global_paths(self, fplist, csp):
        for fp in fplist:
            # calc global positions
            for i in range(len(fp.s)):
                rx, ry = csp.calc_position(fp.s[i])
                if rx is None:
                    break

                rtheta = csp.calc_yaw(fp.s[i])
                rkappa = csp.calc_curvature(fp.s[i])
                rdkappa = csp.calc_d_curvature(fp.s[i])

                s_condition = np.array([fp.s[i], fp.s_d[i], fp.s_dd[i]])

                # di = fp.d[i]
                d_condition = np.array([fp.d[i], fp.d_d[i], fp.d_dd[i]])

                x, y, v, a, theta, kappa = frenet_to_cartesian3D(fp.s[i], rx, ry, rtheta, rkappa, rdkappa, s_condition, d_condition)

                fp.x.append(x)
                fp.y.append(y)
                fp.v.append(v)
                fp.a.append(a)
                fp.yaw.append(theta)
                fp.c.append(kappa)
            # print(len(fp.x))
        return fplist

    def check_collision(self, fp, ob):
        if len(ob) == 0:
            return True
        test = True
        #
        if test:
            for car in ob:
                l = self.pl_cfg.L
                w = self.pl_cfg.W
                left_bottom = [-l * math.cos(car[2]) + w * math.sin(car[2]) + car[0],
                               -l * math.sin(car[2]) - w * math.cos(car[2]) + car[1]]
                right_bottom = [l * math.cos(car[2]) + w * math.sin(car[2]) + car[0],
                                l * math.sin(car[2]) - w * math.cos(car[2]) + car[1]]
                right_up = [l * math.cos(car[2]) - w * math.sin(car[2]) + car[0],
                            l * math.sin(car[2]) + w * math.cos(car[2]) + car[1]]
                left_up = [-l * math.cos(car[2]) - w * math.sin(car[2]) + car[0],
                           -l * math.sin(car[2]) + w * math.cos(car[2]) + car[1]]
                box = [left_bottom, right_bottom, right_up, left_up]
                flag = [inside_range([ix,iy],box)
                     for (ix, iy) in zip(fp.x, fp.y)]
                collision = any(flag)
                if collision:
                    return False
        else:
            for i in range(len(ob)):
                d = [((ix - ob[i, 0]) ** 2 + (iy - ob[i, 1]) ** 2)
                    for (ix, iy) in zip(fp.x, fp.y)]

                collision = any([di <= self.pl_cfg.ROBOT_RADIUS ** 2 for di in d])

                if collision:
                    return False
        return True

    def check_paths(self, fplist, ob):
        ok_ind = []
        N_v = 0
        N_a = 0
        N_c = 0
        N_x = 0
      #  print(f"len ob:{len(ob)}")
        for i, _ in enumerate(fplist):
            if any([v > self.pl_cfg.max_speed for v in fplist[i].s_d]):  # Max speed check
                N_v += 1
                continue
            elif any([abs(a) > self.pl_cfg.max_acc for a in fplist[i].s_dd]):  # Max accel check
                N_a += 1
                continue
            elif any([abs(c) > self.pl_cfg.max_curvature for c in fplist[i].c]):  # Max curvature check
                N_c += 1
                continue
            elif not self.check_collision(fplist[i], ob):
                N_x += 1
                continue

            ok_ind.append(i)
       # print(f"all:{len(fplist)},v:{N_v},a:{N_a},c:{N_c},x:{N_x},left:{len(ok_ind)}")

        return [fplist[i] for i in ok_ind]

    def frenet_optimal_planning(self, target_speed, csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob):
        fplist = self.calc_frenet_paths(target_speed, c_speed, c_d, c_d_d, c_d_dd, s0)
        fplist = self.calc_global_paths(fplist, csp)
        fplist = self.check_paths(fplist, ob)
        # find minimum cost path
        min_cost = float("inf")
        best_path = None
        for fp in fplist:
            if min_cost >= fp.cf:
                min_cost = fp.cf
                best_path = fp

        return best_path, fplist

    def frenet_following_optimal_planning(self, csp, s0, s0_d, s0_dd, c_d, c_d_d, c_d_dd, s1, s1_d, s1_dd, ob):
        fplist = self.calc_following_path(c_d, c_d_d, c_d_dd, s0, s0_d, s0_dd, s1, s1_d, s1_dd)
        fplist = self.calc_global_paths(fplist, csp)
        fplist = self.check_paths(fplist, ob)

        # find minimum cost path
        min_cost = float("inf")
        best_path = None
        for fp in fplist:
            if min_cost >= fp.cf:
                min_cost = fp.cf
                best_path = fp

        return best_path, fplist