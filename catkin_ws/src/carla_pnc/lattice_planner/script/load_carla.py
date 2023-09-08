#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
CARLA_PATH = r'/home/ylh/Carla'
#CARLA_PATH = r'/home/ylh/CARLA/CARLA_0.9.13'
from utils.process import start_process, kill_process
import sys
from os.path import join, dirname
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile, DurabilityPolicy
import time


def main(args=None):
    try:
        parameters = {}
        roscomp.init("carla_load", args=args)
        carla_load_node = CompatibleNode("carla_load")
        parameters['show'] = carla_load_node.get_param('show', False)
        start_process(show=parameters['show'])
        carla_load_node.spin()

    except KeyboardInterrupt:
        kill_process()
        pass

    finally:
        roscomp.loginfo('carla_load shutting down.')
        roscomp.shutdown()

if __name__ == "__main__":
    main()
