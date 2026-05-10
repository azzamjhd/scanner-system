#!/usr/bin/env python3
"""
scanner_bridge.launch.py
========================
Starts robot_state_publisher with the scanner_bridge URDF so the full
TF tree is broadcast:

    base_link → gantry_link → lidar_link
    gantry_link → camera_link → camera_optical_frame

The gantry_joint edge is dynamic: it is driven by /joint_states, which
scan_assembler_node publishes automatically by converting /current_position
(Float32, mm) to metres.  No separate gantry_bridge_node is required.

Typically called from full_system.launch.py, but can be run standalone
to get the TF tree up during development:

    ros2 launch lidar_camera_fusion scanner_bridge.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_camera_fusion')
    urdf_file = os.path.join(pkg_share, 'urdf', 'scanner_bridge.urdf')

    with open(urdf_file, 'r') as fh:
        robot_desc = fh.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            output='screen',
        ),
    ])
