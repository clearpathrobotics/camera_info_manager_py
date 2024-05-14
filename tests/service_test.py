#!/usr/bin/env python

# This trivial node is useful for debugging set_camera_info service
# request handling problems.

import rclpy
from camera_info_manager import CameraInfoManager

rclpy.init()

node = rclpy.create_node("service_test_node")

cinfo = CameraInfoManager(node)

# spin in the main thread: required for service callbacks
rclpy.spin(node)

node.destroy_node()

rclpy.shutdown()
