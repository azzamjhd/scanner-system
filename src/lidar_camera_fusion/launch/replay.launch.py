#!/usr/bin/env python3
"""
replay.launch.py
================
Offline rosbag replay launcher for testing and previewing the tablet/web GUI
without physical hardware.

Starts:
  1. ros2 bag play --clock [--loop] <bag_path> (with remapping for webgui compatibility)
  2. rosbridge_server (WebSocket on port 9090 for tablet_gui)
  3. robot_state_publisher (broadcasts scanner_bridge.urdf TF tree with use_sim_time)

Usage:
  ros2 launch lidar_camera_fusion replay.launch.py

  # With custom bag path or rate:
  ros2 launch lidar_camera_fusion replay.launch.py \
      bag_path:=/home/azzam/ros2_bags/scan_session_20260826_175758 \
      rate:=1.0 loop:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fusion_share = get_package_share_directory('lidar_camera_fusion')
    rosbridge_share = get_package_share_directory('rosbridge_server')
    urdf_file = os.path.join(fusion_share, 'urdf', 'scanner_bridge.urdf')

    with open(urdf_file, 'r') as fh:
        robot_desc = fh.read()

    default_bag = '/home/azzam/ros2_bags/scan_session_20260826_175758'

    # Launch arguments
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value=default_bag,
        description='Path to the ROS2 bag directory'
    )
    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Playback rate multiplier (e.g. 1.0, 2.0)'
    )
    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='true',
        description='Whether to loop bag playback continuously'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock from rosbag'
    )
    launch_rosbridge_arg = DeclareLaunchArgument(
        'launch_rosbridge',
        default_value='true',
        description='Whether to start rosbridge_websocket node for tablet GUI'
    )

    # 1. Robot state publisher (URDF TF tree)
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen',
    )

    # 2. Rosbridge WebSocket server for Web GUI
    rosbridge_launch_file = os.path.join(rosbridge_share, 'launch', 'rosbridge_websocket_launch.xml')
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(rosbridge_launch_file),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
        condition=IfCondition(LaunchConfiguration('launch_rosbridge')),
    )

    # 3. Rosbag Play Process with topic remapping for webgui compatibility
    bag_player_process = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'ROS_LOOP_FLAG="" && '
            'if [ "$1" = "true" ]; then ROS_LOOP_FLAG="--loop"; fi && '
            'ros2 bag play "$2" --clock $ROS_LOOP_FLAG --rate "$3" '
            '--remap /scanner/colored_cloud:=/point_cloud/downsampled',
            'bash',
            LaunchConfiguration('loop'),
            LaunchConfiguration('bag_path'),
            LaunchConfiguration('rate')
        ],
        output='screen'
    )

    return LaunchDescription([
        bag_path_arg,
        rate_arg,
        loop_arg,
        use_sim_time_arg,
        launch_rosbridge_arg,
        LogInfo(msg=['Launching rosbag replay mode from: ', LaunchConfiguration('bag_path')]),
        robot_state_pub_node,
        rosbridge_launch,
        bag_player_process,
    ])
