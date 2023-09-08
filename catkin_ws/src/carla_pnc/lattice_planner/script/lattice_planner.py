#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import carla
from interface.carla_sim import config, add_vehicle
from interface.carla_sim.navigator import get_map, replan, close2dest
from interface.carla_sim.sensor_manager import SensorManager
from utils import add_alpha_channel
from copy import deepcopy as cp
import cv2
import copy
import random
import numpy as np
import threading
import math
import rospy
import time
from transforms3d.euler import quat2euler

from control.rear_wheel_fb import CapacController, getActorState, pi2pi
from interface.map import EgoState, generate_frenet_map
from planning.frenet.lattice_planner import LatticePlanner
#from planning.MPC import MPCController
from interface.dummy_sim import AckermannSteeringModel
from utils import load_config, load_yaml
from utils.coordinate import cartesian_to_frenet3D, cartesian_to_frenet2D, NormalizeAngle
import matplotlib.pyplot as plt
from simple_pid import PID
from tqdm import tqdm

import ros_compatibility as roscomp
from ros_compatibility.exceptions import *
from ros_compatibility.qos import QoSProfile, DurabilityPolicy
from ros_compatibility.node import CompatibleNode

from derived_object_msgs.msg import ObjectArray
from nav_msgs.msg import Odometry,Path
from std_msgs.msg import Float64  # pylint: disable=import-error

from test_carla_planner2 import get_waypoints_xy_ros,generate_frenet_map,get_plan_ros,lattice2vehicle
from control.rear_wheel_fb import State, pi2pi
from carla_msgs.msg import CarlaEgoVehicleControl

class CarlaPlanner(CompatibleNode):
    """
    A planner using refrence line
    """

    def __init__(self):
        """
        Constructor
        """
        super(CarlaPlanner, self).__init__("lattice_planner")

        role_name = self.get_param("role_name", "ego_vehicle")
        self.data_lock = threading.Lock()

        self._ego_vehicle_pose = None
        self._ego_vehicle_twist = None
        self._ego_vehicle_waypoints = None
        self.global_traj = None
        self.vehicle = None
        self._objects = {}

        self.ctrller = CapacController(None, None, 10)

        self._control_cmd_publisher = self.new_publisher(
            CarlaEgoVehicleControl,
            "/carla/{}/vehicle_control_cmd".format(role_name),
            qos_profile=10)

        self._planned_flag_publisher = self.new_publisher(
            Float64,
            "/carla/{}/vehicle_planned_flag".format(role_name),
            qos_profile=10)

        self._odometry_subscriber = self.new_subscription(
            Odometry,
            "/carla/{}/odometry".format(role_name),
            self.odometry_cb,
            qos_profile=10
        )

        self._waypoints_subscriber = self.new_subscription(
            Path,
            "/carla/{}/waypoints".format(role_name),
            self.waypoints_cb,
            qos_profile=10
        )

        #plan_thread = threading.Thread(target = self.make_plan, args=())
        #plan_thread.start()

        #ctl_thread = threading.Thread(target = self.make_ctl, args=())
        #ctl_thread.start()

    def odometry_cb(self, odometry_msg):
        with self.data_lock:
            self._ego_vehicle_pose = odometry_msg.pose.pose
            self._ego_vehicle_twist = odometry_msg.twist.twist

    def waypoints_cb(self, Path_msg):
        with self.data_lock:
            self._ego_vehicle_waypoints = []
            for pose in Path_msg.poses:
                self._ego_vehicle_waypoints.append(pose.pose)
            center_line =  get_waypoints_xy_ros(self._ego_vehicle_waypoints[::2])
            self.frenet_map = generate_frenet_map(center_line[:, 0], center_line[:, 1])

    def make_plan(self):
        with self.data_lock:
            # retrieve relevant elements for safe navigation, i.e.: traffic lights and other vehicles.
            ego_vehicle_pose = copy.deepcopy(self._ego_vehicle_pose)
            ego_vehicle_twist = copy.deepcopy(self._ego_vehicle_twist)
            ego_vehicle_waypoints = copy.deepcopy(self._ego_vehicle_waypoints)

        if ego_vehicle_pose is None:
            self.loginfo("Waiting for ego vehicle pose")
            return

        if ego_vehicle_waypoints is None:
            self.loginfo("Waiting for ego vehicle waypoints")
            return

        plan_time = time.time()
        #plan_time = rospy.Time.now().to_sec()
        state0 = State('odom', plan_time, x=0, y=0, z=0, theta=0, v=0, a=0)
        state0.x = ego_vehicle_pose.position.x
        state0.y = ego_vehicle_pose.position.y
        state0.z = ego_vehicle_pose.position.z
        
        quaternion = (
            ego_vehicle_pose.orientation.w,
            ego_vehicle_pose.orientation.x,
            ego_vehicle_pose.orientation.y,
            ego_vehicle_pose.orientation.z
        )
        _,_,state0.theta = quat2euler(quaternion)
        state0.velocity = np.array([ego_vehicle_twist.linear.x,ego_vehicle_twist.linear.y,0])
        state0.v = np.linalg.norm(state0.velocity)
        # self.loginfo(f"state:x,y,yaw,v:{state0.x,state0.y,state0.theta,state0.v}")

        path = get_plan_ros(None, state0,self.frenet_map)
        if path is not None:
            self._planned_flag_publisher.publish(1)
            global_pose = [state0.x, state0.y, state0.theta]
            self.state = state0
            self.global_traj = lattice2vehicle(path, global_pose, plan_time)
        else:
            pass
            # self.loginfo('No path')

    def make_ctl(self):
        with self.data_lock:
            # retrieve relevant elements for safe navigation, i.e.: traffic lights and other vehicles.
            ego_vehicle_pose = copy.deepcopy(self._ego_vehicle_pose)
            ego_vehicle_twist = copy.deepcopy(self._ego_vehicle_twist)
            global_traj = copy.deepcopy(self.global_traj)

        if ego_vehicle_pose is None:
            self.loginfo("Waiting for ego vehicle pose")
            return
        
        control_command = CarlaEgoVehicleControl()
        control_command.steer = 0.0
        control_command.throttle = 0.0
        control_command.brake = 1.0
        control_command.hand_brake = False
        control_command.manual_gear_shift = False

        state0 = State('odom', time.time(), x=0, y=0, z=0, theta=0, v=0, a=0)
        state0.x = ego_vehicle_pose.position.x
        state0.y = ego_vehicle_pose.position.y
        state0.z = ego_vehicle_pose.position.z
        
        quaternion = (
            ego_vehicle_pose.orientation.w,
            ego_vehicle_pose.orientation.x,
            ego_vehicle_pose.orientation.y,
            ego_vehicle_pose.orientation.z
        )
        _,_,state0.theta = quat2euler(quaternion)
        state0.velocity = np.array([ego_vehicle_twist.linear.x,ego_vehicle_twist.linear.y,0])
        state0.v = np.linalg.norm(state0.velocity)   

        if global_traj is not None:
            # now = rospy.Time.now().to_sec()
            dt =time.time()  - self.state.time_stamp
            index = int(dt/0.2) + 2
            self.loginfo(f"index:{index}")
            control = self.ctrller.run_step_ros(global_traj, index, self.state,state0)
            control_command.steer = control.steer
            control_command.throttle = control.throttle
            control_command.brake = control.brake
        self._control_cmd_publisher.publish(control_command)

def main(args=None):
    try:
        pnc = CarlaPlanner()
        parameters = {}
        roscomp.init("carla_pnc", args=args)
        parameters['role_name'] = pnc.get_param('role_name', "ego_vehicle")

        #plan_thread = threading.Thread(target = pnc.make_plan, args=())
        #plan_thread.start()
        """
        cnt = 0
        while True:
            cnt += 1
            cnt %= 20
            if cnt %10 == 0:
                pnc.make_plan()
            pnc.make_ctl()
        """
        update_timer1 = pnc.new_timer(
            0.1, lambda timer_event=None: pnc.make_plan())
        update_timer2 = pnc.new_timer(
            0.01, lambda timer_event=None: pnc.make_ctl())
        pnc.spin()

    except KeyboardInterrupt:
        pass

    finally:
        roscomp.loginfo('carla_pnc shutting down.')
        roscomp.shutdown()

if __name__ == "__main__":
    main()
