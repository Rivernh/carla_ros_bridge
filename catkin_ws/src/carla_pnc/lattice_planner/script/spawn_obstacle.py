#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import sys
import random
import time

import carla


def _create_vehicle_bluepprint(world, actor_filter, color=None, number_of_wheels=[4]):
    """Create the blueprint for a specific actor type.
Args:
    actor_filter: a string indicating the actor type, e.g, 'vehicle.lincoln*'.
Returns:
    bp: the blueprint object of carla.
"""
    blueprints = world.get_blueprint_library().filter(actor_filter)
    blueprint_library = []
    for nw in number_of_wheels:
        blueprint_library = blueprint_library + [x for x in blueprints if
                                                    int(x.get_attribute('number_of_wheels')) == nw]
    bp = random.choice(blueprint_library)
    if bp.has_attribute('color'):
        if not color:
            color = random.choice(bp.get_attribute('color').recommended_values)
        bp.set_attribute('color', color)
    return bp

def _try_spawn_other_vehicle_at(world, transform):

    blueprint = _create_vehicle_bluepprint(world,'vehicle.audi.a2')
    # blueprint = _create_vehicle_bluepprint(world,'vehicle.*')
    blueprint.set_attribute('role_name', 'autopilot')
    vehicle = world.try_spawn_actor(blueprint, transform)
    #vehicle.set_autopilot()
    return vehicle

    return True

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    world_map = world.get_map()
    vehicle_list = world.get_actors().filter("vehicle.tesla.model3")
    ego_transform = vehicle_list[0].get_transform()
    sp_transform = world_map.get_waypoint(ego_transform.location).transform
    noise = 40
    yaw = sp_transform.rotation.yaw / 180 * math.pi
    sp_transform.location.x += noise * math.cos(yaw)
    sp_transform.location.y += noise * math.sin(yaw)
    sp_transform.location.z += 0.5
    tm = client.get_trafficmanager()
    tm.set_hybrid_physics_mode(True)
    # tm里面每一辆车都是默认速度的80%
    tm.global_percentage_speed_difference(30)


    if True:
        v = _try_spawn_other_vehicle_at(world,sp_transform)
    else:
        spawn_points = world_map.get_spawn_points()
        for i in range(50):
            _try_spawn_other_vehicle_at(world,random.choice(spawn_points))
    t1 = time.time()
    i = 0
    while True:
        i += 1
        t2 = time.time()
        dt = t2 - t1
        if (i % 100 == 0):
            steer = 0.7 * math.sin((dt / 100) % 6)
            control = carla.VehicleControl(throttle=0.5, brake=0.0, steer=steer)
            v.apply_control(control)
        if i > 2000:
            break


if __name__ == '__main__':
    main()
    time.sleep(1000)

