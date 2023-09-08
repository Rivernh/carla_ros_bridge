import numpy as np
import carla
from utils.coordinate import cartesian_to_frenet3D, cartesian_to_frenet2D, NormalizeAngle

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
    localtion = carla.Location(x=x, y=y, z=0.5)
    world.debug.draw_point(localtion, size=0.1, color=color, life_time=life_time)
    
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
    
    
def normalize_angle(angle):
    a = np.fmod(angle+np.pi, 2*np.pi)
    if a < 0.0:
        a += (2.0*np.pi)        
    return a - np.pi

def lattice2mpc2(cfg, path):
    traj_x, traj_y, traj_phi = [], [], []

    traj_x = np.array(path.x)
    traj_y = np.array(path.y)
    traj_phi = np.array(path.yaw)

    traj_v = path.v
    traj_delta = np.arctan(cfg.vehicle.L*np.array(path.c))
    
    for i in range(len(traj_phi)-1):
        traj_phi[i] = normalize_angle(np.arctan2(traj_y[i+1] - traj_y[i], traj_x[i+1] - traj_x[i]))
        
    traj_phi[-1] = traj_phi[-2]
    # traj_delta = traj_phi
    print('traj', round(traj_phi[0],2), round(traj_delta[0],2))
    ref_traj = np.array([traj_x, traj_y, traj_phi, traj_v, traj_delta]).T
    return ref_traj

def lattice2mpc(cfg, path, pose=None):
    traj_x, traj_y, traj_phi = [], [], []
    # x0 = pose[0]
    # y0 = pose[1]
    # yaw0 = pose[2]
    path.x = np.array(path.x)-path.x[0]
    path.y = np.array(path.y)-path.y[0]
    path.yaw = np.array(path.yaw) - path.yaw[0]
    yaw0 = -np.arctan2(path.y[-1]-path.y[0], path.x[-1]-path.x[0])

    for i in range(len(path.x)):
        _x = path.x[i]
        _y = path.y[i]
        x = _x*np.cos(yaw0) - _y*np.sin(yaw0)
        y = _x*np.sin(yaw0) + _y*np.cos(yaw0)
        yaw = NormalizeAngle(path.yaw[i] - yaw0)
        traj_x.append(x)
        traj_y.append(y)
        traj_phi.append(yaw)
    
    traj_v = path.v
    traj_delta = np.arctan(cfg.vehicle.L*np.array(path.c))
    
    for i in range(len(traj_phi)-1):
        traj_phi[i] = np.arctan2(traj_y[i+1] - traj_y[i], traj_x[i+1] - traj_x[i])
    traj_phi[-1] = traj_phi[-2]
    # print('traj', round(traj_phi[0],2), round(traj_delta[0],2))
    ref_traj = np.array([traj_x, traj_y, traj_phi, traj_v, traj_delta]).T
    return ref_traj
    
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

def dist(v1, v2):
    x0 = v1.get_location().x
    y0 = v1.get_location().y
    x1 = v2.get_location().x
    y1 = v2.get_location().y
    return np.hypot(x1-x0, y1-y0)

def draw_global_path(world, x_list, y_list, t=5):
    for x, y in zip(x_list, y_list):
        draw_point(world, x, y, t, color=(255, 0, 0))