#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
CARLA_PATH = r'/home/ylh/Carla'
#CARLA_PATH = r'/home/ylh/CARLA/CARLA_0.9.13'
from interface import carla_sim
carla_sim.load(CARLA_PATH)
from utils.process import start_process, kill_process
import sys
from os.path import join, dirname
#sys.path.insert(0, join(dirname(__file__), '/home/ylh/CARLA/CARLA_0.9.13/PythonAPI'))
sys.path.insert(0, join(dirname(__file__), '/home/ylh/Carla/PythonAPI'))
import carla
sys.path.append(CARLA_PATH+'/PythonAPI/carla')
from agents.navigation.basic_agent import BasicAgent
from interface.carla_sim import config, add_vehicle
from interface.carla_sim.navigator import get_map, replan, close2dest
from interface.carla_sim.sensor_manager import SensorManager
from utils import add_alpha_channel
from copy import deepcopy as cp
import cv2
import time
import copy
import random
import numpy as np
import threading

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

MAX_SPEED = 10
DRAW_PATH = True

global_view_img = None
global_vel = 0
global_collision = False
global_path = None
global_pose = None
global_state = None
global_traj = None
world = None
vehicle = None
frenet_map = None

cfg = load_config('/home/ylh/carla-ros-bridge/catkin_ws/src/carla_pnc/lattice_planner/script/config.yaml')
planner = LatticePlanner(cfg)
pl_cfg = cfg.lattice_planner
mpc_cfg = load_yaml('/home/ylh/carla-ros-bridge/catkin_ws/src/carla_pnc/lattice_planner/script/config.yaml')

def collision_callback(data):
    global global_collision
    global_collision = True

def view_image_callback(data):
    global global_view_img
    array = np.frombuffer(data.raw_data, dtype=np.dtype("uint8")) 
    array = np.reshape(array, (data.height, data.width, 4)) # RGBA format
    global_view_img = array

def visualize(input_img, nav=None):
    global global_vel
    img = copy.deepcopy(input_img)
    text = "speed: "+str(round(3.6*global_vel, 1))+' km/h'
    cv2.putText(img, text, (20, 30), cv2.FONT_HERSHEY_PLAIN, 2.0, (255, 255, 255), 2)
    if nav is not None:
        new_nav = add_alpha_channel(nav)
        new_nav = cv2.flip(new_nav, 1)
        new_nav = copy.deepcopy(new_nav)
        img[:nav.shape[0],-nav.shape[1]:] = new_nav

    cv2.imshow('Visualization', img)
    cv2.waitKey(5)

def global2local(wpt_list, org_wpt):
    point_list = []
    for item in wpt_list:
        wp = item[0]
        
        _x = wp.transform.location.x-org_wpt.transform.location.x
        _y = wp.transform.location.y-org_wpt.transform.location.y
        
        yaw = np.deg2rad(org_wpt.transform.rotation.yaw)
        c = _x*np.cos(yaw) - _y*np.sin(yaw)
        s = _x*np.sin(yaw) + _y*np.cos(yaw)
        point_list.append([s, c])
    
    return np.array(point_list)

def get_route_trace_xy(route_trace):
    point_list = []
    for item in route_trace:
        wp = item[0]
        x = wp.transform.location.x
        y = wp.transform.location.y
        point_list.append([x, y])
    return np.array(point_list)

def get_waypoints_xy(waypoints):
    point_list = []
    for wp in waypoints:
        x = wp.location.x
        y = wp.location.y
        point_list.append([x, y])
    return np.array(point_list)

def get_waypoints_xy_ros(waypoints):
    point_list = []
    for wp in waypoints:
        x = wp.position.x
        y = wp.position.y
        point_list.append([x, y])
    return np.array(point_list)
        
def global2vehicle(wpt_list, vehicle_tf):
    point_list = []
    for item in wpt_list:
        wp = item[0]
        
        _x = wp.transform.location.x-vehicle_tf.location.x
        _y = wp.transform.location.y-vehicle_tf.location.y
        
        yaw = np.deg2rad(vehicle_tf.rotation.yaw)
        x = _x*np.cos(yaw) - _y*np.sin(yaw)
        y = _x*np.sin(yaw) + _y*np.cos(yaw)
        point_list.append([x, y])
    
    return np.array(point_list)

def draw_point(world, x, y, life_time=0.1, color=None):
    if color is None:
        color = carla.Color(0,0,255)
    else:
        color = carla.Color(color[0],color[1],color[2])
    localtion = carla.Location(x=x, y=y, z=2.5)
    world.debug.draw_point(localtion, size=0.2, color=color, life_time=life_time)

def get_sd(frenet_map, x, y, v, a, theta, kappa):
    nearest_ref_info = frenet_map.kdtree.query([x, y])
    index = nearest_ref_info[1]
    if v > 0.5:
        s, d = cartesian_to_frenet3D(frenet_map.rs[index], frenet_map.rx[index], frenet_map.ry[index], 
                                     frenet_map.ryaw[index], frenet_map.rk[index], frenet_map.rdk[index], x, y, v, a, theta, kappa)
        return s[0], d[0], s[1], d[1], s[2], d[2]
    else:
        s, d = cartesian_to_frenet2D(frenet_map.rs[index], frenet_map.rx[index], frenet_map.ry[index],
                                     frenet_map.ryaw[index], frenet_map.rk[index], x, y, v, theta)
        return s[0], d[0], s[1], d[1], 0, 0


def get_plan(world, vehicle, frenet_map):
    global global_ob
    x = vehicle.get_transform().location.x
    y = vehicle.get_transform().location.y
    theta = np.deg2rad(vehicle.get_transform().rotation.yaw)
    vel = vehicle.get_velocity()
    v = np.sqrt(vel.x**2+vel.y**2)
    acc = vehicle.get_acceleration()
    a = np.sqrt(acc.x**2+acc.y**2)
    #print(f"x:{x},y:{y},theta:(theta)")
    if v < 0.1:
        kappa = 999#np.inf
    else:
        kappa = (vel.x*acc.y-acc.x*vel.y)/np.power(vel.x*vel.x+vel.y*vel.y, 3/2)

    # get vehicle position in frenet
    s, d, s_d, d_d, s_dd, d_dd = get_sd(frenet_map, x, y, v, a, theta, kappa)

    # print('s/d', round(s,2), round(d,2), '\ts_d/d_d', round(s_d,2), round(d_d,2), '\ts_dd/d_dd', round(s_dd,2), round(d_dd,2))
    
    state = EgoState(
        c_d = d,  # current lateral position [m]
        c_d_d = d_d,  # current lateral speed [m/s]
        c_d_dd = d_dd,  # current lateral acceleration [m/ss]
        s0 = s,  # current course position
        s0_d = s_d,  # current speed [m/s]
        s0_dd = s_dd,
        c_speed = v
    )
     
    ob = np.array(global_ob)
    s0, s0_d, s0_dd, c_d, c_d_d, c_d_dd, c_speed = state.get_state()
    
    path, all_paths = planner.frenet_optimal_planning(
                8,frenet_map.csp,
                s0, c_speed, c_d, c_d_d, c_d_dd, ob)
    
    # path, all_paths = planner.frenet_following_optimal_planning(
    #             frenet_map.csp,
    #             s0, s0_d, s0_dd, c_d, c_d_d, c_d_dd,
    #             car_1.s() - pl_cfg.D0 - pl_cfg.tau * car_1.s_d(), car_1.s_d(), car_1.s_dd(),
    #             ob)

    if DRAW_PATH and path is not None:
        for i in range(len(path.x)):
            draw_point(world, path.x[i], path.y[i], 0.2)

    return path

def get_plan_ros(ob,state,frenet_map):
    x = state.x
    y = state.y
    theta =state.theta
    v = state.v
    a = state.a
    vel = state.velocity
    acc = state.acceleration
    #print(f"x:{x},y:{y},theta:(theta)")
    if v < 0.1:
        kappa = 999#np.inf
    else:
        kappa = (vel[0]*acc[1]-acc[0]*vel[1])/np.power(vel[0]*vel[0]+vel[1]*vel[1], 3/2)

    # get vehicle position in frenet
    s, d, s_d, d_d, s_dd, d_dd = get_sd(frenet_map, x, y, v, a, theta, kappa)

    # print('s/d', round(s,2), round(d,2), '\ts_d/d_d', round(s_d,2), round(d_d,2), '\ts_dd/d_dd', round(s_dd,2), round(d_dd,2))
    
    state = EgoState(
        c_d = d,  # current lateral position [m]
        c_d_d = d_d,  # current lateral speed [m/s]
        c_d_dd = d_dd,  # current lateral acceleration [m/ss]
        s0 = s,  # current course position
        s0_d = s_d,  # current speed [m/s]
        s0_dd = s_dd,
        c_speed = v
    )
     
    ob = []
    s0, s0_d, s0_dd, c_d, c_d_d, c_d_dd, c_speed = state.get_state()
    
    path, all_paths = planner.frenet_optimal_planning(
                8,frenet_map.csp,
                s0, c_speed, c_d, c_d_d, c_d_dd, ob)

    return path

def lattice2vehicle(path, pose, t):
    traj = {'time':[], 'x':[], 'y':[], 'yaw':[], 'v':[], 'c':[]}
    x0 = pose[0]
    y0 = pose[1]
    # yaw0 = pose[2]
    # x0 = path.x[0]
    # y0 = path.y[0]
    # yaw0 = -np.arctan2(path.y[-1]-path.y[0], path.x[-1]-path.x[0])
    yaw0 = -pose[2]
    # path.yaw = np.array(path.yaw) - path.yaw[0]
    path.yaw = pi2pi(np.array(path.yaw) - pose[2])

    path.x = np.array(path.x)-x0
    path.y = np.array(path.y)-y0
    # path.yaw = pi2pi(np.array(path.yaw) - yaw0)
    
    for i in range(len(path.x)):
        _x = path.x[i]
        _y = path.y[i]
        x = _x*np.cos(yaw0) - _y*np.sin(yaw0)
        y = _x*np.sin(yaw0) + _y*np.cos(yaw0)
        
        traj['x'].append(x)
        traj['y'].append(y)
        
    traj['time'] = t
    traj['yaw'] = path.yaw
    traj['v'] = path.v
    traj['a'] = path.a
    traj['c'] = path.c

    return traj

def get_ref_pose(vehicle, org_pose):
    x0 = org_pose[0]
    y0 = org_pose[1]
    yaw0 = org_pose[2]
    _x = vehicle.get_transform().location.x - x0
    _y = vehicle.get_transform().location.y - y0
    x = _x*np.cos(-yaw0) - _y*np.sin(-yaw0)
    y = _x*np.sin(-yaw0) + _y*np.cos(-yaw0)
    yaw = NormalizeAngle(np.deg2rad(vehicle.get_transform().rotation.yaw) - yaw0)
    return x, y, yaw

def make_plan():
    global world, vehicle, frenet_map, global_path, global_pose, global_state, global_traj
    while True:
        # import cProfile, pstats, io
        # from pstats import SortKey
        # pr = cProfile.Profile()
        # pr.enable()
        plan_time = time.time()
        state0 = getActorState('odom', plan_time, vehicle)
        state0.x = vehicle.get_transform().location.x
        state0.y = vehicle.get_transform().location.y
        state0.z = vehicle.get_transform().location.z
        state0.theta = np.deg2rad(vehicle.get_transform().rotation.yaw)
     
        path = get_plan(world, vehicle, frenet_map)
        # pr.disable()
        # s = io.StringIO()
        # sortby = SortKey.CUMULATIVE
        # ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
        # ps.print_stats()
        # print(s.getvalue())
        if path is not None:
            # x = path.x[0]
            # y = path.y[0]
            # yaw = path.yaw[0]
            # yaw = np.arctan2(path.y[-1]-path.y[0], path.x[-1]-path.x[0])
            global_path = path
            global_pose = [state0.x, state0.y, state0.theta]
            global_state = state0
            global_traj = lattice2vehicle(path, global_pose, plan_time)
            # plt.cla()
            # plt.plot(global_traj['x'], global_traj['y'])
            # plt.axis('equal')
            # plt.pause(0.01)
        else:
            print('No path')
            global_path = None
            
        # time.sleep(0.5)

def dist(v1, v2):
    x0 = v1.get_location().x
    y0 = v1.get_location().y
    x1 = v2.get_location().x
    y1 = v2.get_location().y
    return np.hypot(x1-x0, y1-y0)
    
def decision(world, vehicle):
    global frenet_map, global_ob
    vehicle_list = world.get_actors().filter("*vehicle*")
    yaw0 = np.deg2rad(vehicle.get_transform().rotation.yaw)
    npc_dict = {}
    ob = []
    for npc in vehicle_list:
        if npc.id == vehicle.id:
            continue

        distance = dist(npc, vehicle)
        if distance > 50: continue
        yaw = NormalizeAngle(np.deg2rad(npc.get_transform().rotation.yaw) - yaw0)
        
        x = npc.get_transform().location.x
        y = npc.get_transform().location.y
        theta = np.deg2rad(npc.get_transform().rotation.yaw)
        vel = npc.get_velocity()
        v = np.sqrt(vel.x**2+vel.y**2)
        acc = npc.get_acceleration()
        a = np.sqrt(acc.x**2+acc.y**2)
        if v < 0.1:
            kappa = 999#np.inf
        else:
            kappa = (vel.x*acc.y-acc.x*vel.y)/np.power(vel.x*vel.x+vel.y*vel.y, 3/2)
        
        s, d, s_d, d_d, s_dd, d_dd = get_sd(frenet_map, x, y, v, a, theta, kappa)
        npc_dict[npc.id] = [npc, distance, yaw, s, d]
        ob.append([x,y,theta])
    global_ob = ob
    # print(npc_dict)
    # print()
    return npc_dict
    
def main():
    global global_path, global_vel, global_view_img, global_collision, global_ob
    global world, vehicle, frenet_map, global_pose, global_state, global_traj
    
    client = carla.Client(config['host'], config['port'])
    client.set_timeout(config['timeout'])
    
    world = client.load_world('Town01')
    world.set_weather(carla.WeatherParameters.ClearNoon)

    blueprint = world.get_blueprint_library()
    world_map = world.get_map()
    
    vehicle = add_vehicle(world, blueprint, vehicle_type='vehicle.audi.a2')

    spawn_points = world_map.get_spawn_points()
    waypoint_tuple_list = world_map.get_topology()

    origin_map = get_map(waypoint_tuple_list)

    agent = BasicAgent(vehicle, target_speed=MAX_SPEED, opt_dict={'ignore_stop_signs':True})
  
    vehicle.set_simulate_physics(True)

    sensor_dict = {
        'camera:view':{
            # 'transform':carla.Transform(carla.Location(x=0.0, y=0.0, z=30.0), carla.Rotation(yaw=-90, pitch=-90)),
            'transform':carla.Transform(carla.Location(x=-3.0, y=0.0, z=6.0), carla.Rotation(pitch=-45)),
            'callback':view_image_callback,
            },
        'collision':{
            'transform':carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0)),
            'callback':collision_callback,
            },
        }

    sm = SensorManager(world, blueprint, vehicle, sensor_dict)
    sm.init_all()

    '''
    settings = world.get_settings()
    settings.fixed_delta_seconds = 0.05
    settings.synchronous_mode = True
    world.apply_settings(settings)
    '''
    
    destination = carla.Transform()
    destination.location = world.get_random_location_from_navigation()
    _, destination = replan(agent, destination, copy.deepcopy(origin_map), spawn_points)
    dest_wpt = world_map.get_waypoint(destination.location)
    org_wpt = world_map.get_waypoint(vehicle.get_location())
    route_trace = agent.trace_route(org_wpt, dest_wpt)
    center_line = get_route_trace_xy(route_trace[::2])
    frenet_map = generate_frenet_map(center_line[:, 0], center_line[:, 1])
    #for i in center_line:
    #    draw_point(world,i[0],i[1],120,(255,0,0))
    global_ob =[]

    vehicle_blueprints = world.get_blueprint_library().filter('*vehicle*')
    for i in range(0,50):
        world.try_spawn_actor(random.choice(vehicle_blueprints), random.choice(spawn_points))

    ctrller = CapacController(world, vehicle, MAX_SPEED)
    
    plan_thread = threading.Thread(target = make_plan, args=())
    plan_thread.start()
    goal_pos_x = []
    goal_pos_y = []
    current_pos_x = []
    current_pos_y = []
    goal_speed = []
    current_speed = []
    global_t = []
    
    # for total_steps in tqdm(range(9999)):
    for total_steps in range(9999):
        global_t.append(total_steps)
        current_pos_x.append(vehicle.get_transform().location.x)
        current_pos_y.append(vehicle.get_transform().location.y)
        current_speed.append(global_vel)

        while global_view_img is None:
            time.sleep(0.001)
        npc_dict = decision(world, vehicle)
        # print(f"npc_dict:{len(npc_dict)}")
        min_d = np.inf
        min_id = None
        for id in npc_dict.keys():
            npc = npc_dict[id]
            if abs(npc[-1]) < min_d:
                min_d = abs(npc[-1])
                min_id = id
        
        if min_id is not None and min_d < 2:
            loc = npc_dict[min_id][0].get_location()
            draw_point(world, loc.x, loc.y, 1.0, (0,255,0))
        
        if total_steps % 2 == 0:
            visualize(global_view_img)
        
        if close2dest(vehicle, destination):
            print('get destination !!!')
            break
            start_point = random.choice(spawn_points)
            vehicle.set_transform(start_point)
            
            destination = carla.Transform()
            destination.location = world.get_random_location_from_navigation()
            _, destination = replan(agent, destination, copy.deepcopy(origin_map), spawn_points)
            dest_wpt = world_map.get_waypoint(destination.location)
            org_wpt = world_map.get_waypoint(vehicle.get_location())
            route_trace = agent.trace_route(org_wpt, dest_wpt)
            center_line = get_route_trace_xy(route_trace[::2])
            frenet_map = generate_frenet_map(center_line[:, 0], center_line[:, 1])
            
            start_point.rotation = route_trace[0][0].transform.rotation
            vehicle.set_transform(start_point)
            time.sleep(0.1)

        if global_collision:
            print('Collision')
            break
            # cv2.imwrite('result/img_log/'+log_name+'/'+str(time.time())+'.png', copy.deepcopy(global_view_img))

            start_point = random.choice(spawn_points)
            vehicle.set_transform(start_point)
            
            destination = carla.Transform()
            destination.location = world.get_random_location_from_navigation()
            _, destination = replan(agent, destination, copy.deepcopy(origin_map), spawn_points)
            dest_wpt = world_map.get_waypoint(destination.location)
            org_wpt = world_map.get_waypoint(vehicle.get_location())
            route_trace = agent.trace_route(org_wpt, dest_wpt)
            center_line = get_route_trace_xy(route_trace[::2])
            frenet_map = generate_frenet_map(center_line[:, 0], center_line[:, 1])
            
            start_point.rotation = route_trace[0][0].transform.rotation
            vehicle.set_transform(start_point)

            time.sleep(0.1)
            global_collision = False

        vel = vehicle.get_velocity()
        global_vel = np.sqrt(vel.x**2+vel.y**2)


        if True:#global_path is not None:
            # ref_traj = lattice2mpc(global_path)
            # x, y, yaw = get_ref_pose(vehicle, global_pose)
            try:
                #print(total_steps)
                dt = time.time() - global_state.time_stamp
                index = int(dt/pl_cfg.dt) + 2
                if global_path is not None:
                    control = ctrller.run_step(global_traj, index, global_state)
                    goal_pos_x.append(global_traj['x'][index] + current_pos_x[-1])
                    goal_pos_y.append(global_traj['y'][index] + current_pos_y[-1])
                    goal_speed.append(global_traj['v'][index])
                else:
                    control = carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0)
                    goal_pos_x.append(current_pos_x[-1])
                    goal_pos_y.append(current_pos_y[-1])
                    goal_speed.append(0)
                vehicle.apply_control(control)
                # world.tick()

                if len(global_t) > 2000:
                    """
                    plt.title('speed curve', fontsize=15)  # 标题，并设定字号大小
                    plt.xlabel(u'time(s)', fontsize=10)  # 设置x轴，并设定字号大小
                    plt.ylabel(u'speed(m/s)', fontsize=10)  # 设置y轴，并设定字号大小

                    plt.plot(np.arange(len(goal_speed)) * 0.2, goal_speed, color='#1E90FF',  label='goal_speed')
                    plt.plot(np.arange(len(current_speed)) * 0.2, current_speed, color='#6A5ACD', label='current_speed')
                    plt.legend(loc='best')  # 图例展示位置，数字代表第几象限3
                    plt.show()
                    """

                    plt.title('pos curve', fontsize=15)  # 标题，并设定字号大小
                    plt.xlabel(u'x(m)', fontsize=10)  # 设置x轴，并设定字号大小
                    plt.ylabel(u'y(m)', fontsize=10)  # 设置y轴，并设定字号大小
                    #plt.scatter(np.arange(10), np.arange(10),color='#1E90FF',  label='goal_pos')

                    plt.plot(np.array(goal_pos_x), np.array(goal_pos_y), color='#1E90FF',  label='goal_pos')
                    plt.plot(np.array(current_pos_x), np.array(current_pos_y), color='#6A5ACD', label='current_pos')
                    plt.legend(loc='best')  # 图例展示位置，数字代表第几象限3
                    plt.show()
            except:
                pass
            # control = agent.run_step()
            # control.manual_gear_shift = False
            # vehicle.apply_control(control)
        
    cv2.destroyAllWindows()
    sm.close_all()
    vehicle.destroy()
        
if __name__ == '__main__':
    try:
        start_process(show=False)
        main()
    finally:
        kill_process()
