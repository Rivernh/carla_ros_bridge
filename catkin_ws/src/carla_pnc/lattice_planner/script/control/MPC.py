#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import math
from scipy import sparse
from scipy.spatial import KDTree
import osqp
# from simple_pid import PID

class Constraint:
    def __init__(self, umin, umax, delta_umin, delta_umax):
        self.umin = umin
        self.umax = umax
        self.delta_umin = delta_umin
        self.delta_umax = delta_umax

class MPCController():
    def __init__(self, config, sim_vehicle, mpc):
        self.config = config
        self.vehicle_cfg = self.config.get('vehicle')
        self.T = config.get('dt')
        self.mpc = mpc

        # self.mpc = MPC(config)
        # self.mpc = CppMPC()
        self.sim_vehicle = sim_vehicle
        # self.pid = PID(1.5, 1, 0.1, sample_time=0.1, setpoint=10)
        # self.pid.output_limits = (self.vehicle_cfg.get('delta_umin_v'), self.vehicle_cfg.get('delta_umax_v'))
    
    def generate_traj(self, ref_traj, duration = None):
        """
        :param ref_traj: reference trajectory, np.array([x, y, phi, v, delta])
        :param duration: duration of the generated trajectory, float
        :returns trajectory: dict
        """
        trajectory = {
            'x':[], 'y':[], 'v':[], 'delta':[], 'ref_v':[], 'e':[]
        }
        tree = KDTree(ref_traj[:, :2])
        duration_step = 999 if duration is None else int(duration/self.T)
        for i in range(duration_step):
            # MPC control
            # u_cur, d_u_cur = self.mpc.solve(self.sim_vehicle.get_pose(), self.sim_vehicle.get_control(), ref_traj)
            x = self.sim_vehicle.get_pose()
            u_pre = self.sim_vehicle.get_control()
            nearest_ref_info = tree.query(x[:2])
            # ref_traj_point = ref_traj[min(len(ref_traj)-1, nearest_ref_info[1]+i//2+1)]
            ref_traj_point = ref_traj[nearest_ref_info[1]]
            # print(nearest_ref_info[1]+i//2+1, ref_traj_point[3])
            res = self.mpc.solve(x, u_pre, ref_traj_point)
            u_cur = np.array([res[0], res[1]])
            # d_u_cur = np.array([res[2], res[3]])
            # PID control
            # self.pid = PID(1.5, 1, 0.1, sample_time=0.1,setpoint=ref_traj_point[3])
            # self.pid.output_limits = (self.vehicle_cfg.get('delta_umin_v'), self.vehicle_cfg.get('delta_umax_v'))

            # dv = self.pid(self.sim_vehicle.v)
            # sim
            # action = self.sim_vehicle.dstep(u_cur, dv)
            action = self.sim_vehicle.dstep(u_cur)

            trajectory['x'].append(self.sim_vehicle.x)
            trajectory['y'].append(self.sim_vehicle.y)
            trajectory['v'].append(action[0])
            trajectory['delta'].append(action[1])
            trajectory['ref_v'].append(ref_traj_point[3])
            e = pow(ref_traj_point[0]-x[0],2)+pow(ref_traj_point[1]-x[1],2)
            trajectory['e'].append(e)
            
            # if self.sim_vehicle.x > ref_traj[:,0][-1]:
            #     break

        return trajectory

class MPC():
    def __init__(self, config):
        self.L = config.get('vehicle').get('L')
        self.T = config.get('dt')

        self.Nx = 3
        self.Nu = 2

        self.Nc = 30
        self.Np = 60

        v = config.get('vehicle')
        constraint = Constraint(
            umin = np.array([[v.get('umin_v')], [v.get('umin_d')]]),
            umax = np.array([[v.get('umax_v')], [v.get('umax_d')]]),
            delta_umin = np.array([[v.get('delta_umin_v')], [v.get('delta_umin_d')]]),
            delta_umax = np.array([[v.get('delta_umax_v')], [v.get('delta_umax_d')]])
        )
        self.constraint = constraint
        
        self.A_t = np.zeros((self.Nc, self.Nc))
        for row in range(self.Nc):
            for col in range(self.Nc):
                if row >= col:
                    self.A_t[row, col] = 1.0
        self.res_x = None
        self.res_y = None

    def solve(self, x, u_pre, ref_traj):
        """
        :param x: vehicle pose [x, y, theta]
        :param u_pre: last d_action [dv, ddelta]
        :param ref_traj: reference trajectory
        :returns: current d_action, d_d_action
        """
        tree = KDTree(ref_traj[:, :2])
        nearest_ref_info = tree.query(x[:2])
        nearest_ref_x = ref_traj[nearest_ref_info[1]]

        a = np.array([
            [1.0,  0,   -nearest_ref_x[3] * math.sin(nearest_ref_x[2]) * self.T],
            [0,   1.0,   nearest_ref_x[3] * math.cos(nearest_ref_x[2]) * self.T],
            [0,   0,     1]
        ])

        b = np.array([
            [math.cos(nearest_ref_x[2]) * self.T,             0],
            [math.sin(nearest_ref_x[2]) * self.T,             0],
            [math.tan(nearest_ref_x[4]) * self.T / self.L,     nearest_ref_x[3] * self.T / (self.L * math.pow(math.cos(nearest_ref_x[4]), 2.0))]
        ])

        A = np.zeros([self.Nx + self.Nu, self.Nx + self.Nu])
        A[0 : self.Nx, 0 : self.Nx] = a
        A[0 : self.Nx, self.Nx : ] =  b
        A[self.Nx :, self.Nx :] = np.eye(self.Nu)

        B = np.zeros([self.Nx + self.Nu, self.Nu])
        B[0 : self.Nx, :] = b
        B[self.Nx :, : ] = np.eye(self.Nu)

        C = np.array([[1, 0, 0, 0, 0], [0, 1, 0 ,0 ,0], [0, 0, 1, 0, 0]])

        theta = np.zeros([self.Np * self.Nx, self.Nc * self.Nu])
        phi = np.zeros([self.Np * self.Nx, self.Nu + self.Nx])
        tmp = C

        for i in range(1, self.Np + 1):
            phi[self.Nx * (i - 1) : self.Nx * i] = np.dot(tmp, A)

            tmp_c = np.zeros([self.Nx, self.Nc * self.Nu])
            tmp_c[ :, 0 : self.Nu] = np.dot(tmp, B)

            if i > 1:
                tmp_c[ :, self.Nu :] = theta[self.Nx * (i - 2) : self.Nx * (i - 1), 0 : -self.Nu]

            theta[self.Nx * (i - 1) : self.Nx * i, :] = tmp_c

            tmp = np.dot(tmp, A)


        Q = np.eye(self.Nx * self.Np)
        R = 5.0 * np.eye(self.Nu * self.Nc)
        rho = 10

        H = np.zeros((self.Nu * self.Nc + 1, self.Nu * self.Nc + 1))
        H[0 : self.Nu * self.Nc, 0 : self.Nu * self.Nc] = np.dot(np.dot(theta.transpose(), Q), theta) + R
        H[-1 : -1] = rho

        kesi = np.zeros((self.Nx + self.Nu, 1))
        diff_x = x - nearest_ref_x[:3]
        diff_x = diff_x.reshape(-1, 1)
        kesi[: self.Nx, :] = diff_x
        diff_u = u_pre.reshape(-1, 1)
        kesi[self.Nx :, :] = diff_u

        F = np.zeros((1, self.Nu * self.Nc + 1))
        F_1 = 2 * np.dot(np.dot(np.dot(phi, kesi).transpose(), Q), theta)
        F[ 0,  0 : self.Nu * self.Nc] = F_1

        # A_t = np.zeros((self.Nc, self.Nc))
        # for row in range(self.Nc):
        #     for col in range(self.Nc):
        #         if row >= col:
        #             A_t[row, col] = 1.0

        A_I = np.kron(self.A_t, np.eye(self.Nu))

        A_cons = np.zeros((self.Nc * self.Nu, self.Nc * self.Nu + 1))
        A_cons[0 : self.Nc * self.Nu, 0 : self.Nc * self.Nu] = A_I

        U_t = np.kron(np.ones((self.Nc, 1)), u_pre.reshape(-1, 1))

        U_min = np.kron(np.ones((self.Nc, 1)), self.constraint.umin)
        U_max = np.kron(np.ones((self.Nc, 1)), self.constraint.umax)

        LB = U_min - U_t
        UB = U_max - U_t

        delta_Umin = np.kron(np.ones((self.Nc, 1)), self.constraint.delta_umin)
        delta_Umax = np.kron(np.ones((self.Nc, 1)), self.constraint.delta_umax)

        delta_Umin = np.vstack((delta_Umin, [0]))
        delta_Umax = np.vstack((delta_Umax, [10]))

        A_1_cons = np.eye(self.Nc * self.Nu + 1, self.Nc * self.Nu + 1)

        A_cons = np.vstack((A_cons, A_1_cons))

        LB = np.vstack((LB, delta_Umin))
        UB = np.vstack((UB, delta_Umax))

        # Create an OSQP object
        prob = osqp.OSQP()

        H = sparse.csc_matrix(H)
        A_cons = sparse.csc_matrix(A_cons)

        # Setup workspace
        prob.setup(H, F.transpose(), A_cons, LB, UB, verbose=True, warm_start=True, time_limit=0.05)
        
        if self.res_x is not None and self.res_y is not None:
            prob.warm_start(x=self.res_x, y=self.res_y)
            
        res = prob.solve()
        
        self.res_x = res.x
        self.res_y = res.y

        # Check solver status
        if res.info.status != 'solved':
            raise ValueError('OSQP did not solve the problem!')

        u_cur = u_pre + res.x[0 : self.Nu]

        return u_cur, res.x[0 : self.Nu]