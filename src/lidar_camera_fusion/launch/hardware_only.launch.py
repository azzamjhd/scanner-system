#!/usr/bin/env python3
"""
hardware_only.launch.py
=======================
Headless bringup for Raspberry Pi / Jetson / SSH sessions on ROS 2 Jazzy.

Starts only the hardware-facing scanning path:
  1. micro-ROS agent
  2. RPLiDAR A1
  3. robot_state_publisher for scanner_bridge.urdf
  4. optional v4l2_camera_node
  5. scan_assembler_node
  6. scan_cycle_node

Intentionally does NOT start:
  - fusion_gui / PyQt (requires desktop session)
  - cloud_colorizer_node
  - scan_image_recorder_node
  - body_preprocess_node
  - manual_segmentation_node
  - RViz/Foxglove

Usage:
  ros2 launch lidar_camera_fusion hardware_only.launch.py \
    output_dir:=/home/azzam/ros2_scans \
    max_points:=100000 publish_rate:=0.5 enable_camera:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fusion_share = get_package_share_directory('lidar_camera_fusion')

    args = [
        DeclareLaunchArgument('enable_micro_ros', default_value='true',
                              description='Start micro_ros_agent for ESP32 gantry firmware'),
        DeclareLaunchArgument('enable_lidar', default_value='true',
                              description='Start rplidar_ros A1 driver'),
        DeclareLaunchArgument('enable_camera', default_value='false',
                              description='Start v4l2_camera_node. False saves RAM on headless Pi/Nano.'),
        DeclareLaunchArgument('enable_camera_locks', default_value='false',
                              description='Apply C922 v4l2-ctl focus/exposure locks after camera startup'),
        DeclareLaunchArgument('enable_scan_cycle', default_value='true',
                              description='Start /scanner/start_cycle and /scanner/interrupt service node'),

        DeclareLaunchArgument('mcu_port', default_value='/dev/ttyUSB0',
                              description='Serial port for micro-ROS / ESP32 firmware'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1',
                              description='Serial port for RPLiDAR A1'),
        DeclareLaunchArgument('lidar_frame_id', default_value='lidar_link',
                              description='Frame stamped on /scan; lidar_link matches scanner_bridge.urdf'),
        DeclareLaunchArgument('scan_mode', default_value='Standard',
                              description='RPLiDAR A1 mode. Standard is lighter than Boost.'),

        DeclareLaunchArgument('camera_device', default_value='/dev/video0',
                              description='V4L2 camera device'),
        DeclareLaunchArgument('camera_info_url', default_value='file:///home/azzam/Documents/webcam_calibration.yaml',
                              description='Camera calibration URL'),
        DeclareLaunchArgument('camera_frame_id', default_value='camera_optical_frame',
                              description='Frame stamped on camera images'),

        DeclareLaunchArgument('position_topic', default_value='/current_position',
                              description='Gantry position feedback topic (geometry_msgs/Point; x is scan axis in mm)'),
        DeclareLaunchArgument('target_position_topic', default_value='/target_position',
                              description='Gantry target command topic (geometry_msgs/Point; x is scan axis in mm)'),
        DeclareLaunchArgument('speed_topic', default_value='/speed',
                              description='Gantry speed command topic (std_msgs/Float32, mm/s)'),

        DeclareLaunchArgument('target_frame', default_value='base_link',
                              description='TF frame to accumulate assembled cloud in'),
        DeclareLaunchArgument('scan_topic', default_value='/scan',
                              description='LaserScan topic from RPLiDAR'),
        DeclareLaunchArgument('output_dir', default_value=os.path.expanduser('~/ros2_scans'),
                              description='Directory where /scanner/stop saves timestamped .pcd files'),
        DeclareLaunchArgument('max_points', default_value='100000',
                              description='Circular buffer capacity; lower value reduces RAM'),
        DeclareLaunchArgument('publish_rate', default_value='0.5',
                              description='Assembled-cloud publish rate in Hz; lower value reduces CPU/DDS load'),

        DeclareLaunchArgument('scan_start_mm', default_value='0.0',
                              description='Gantry start position for /scanner/start_cycle'),
        DeclareLaunchArgument('scan_end_mm', default_value='1850.0',
                              description='Gantry end position for /scanner/start_cycle'),
        DeclareLaunchArgument('scan_speed_mm_s', default_value='20.0',
                              description='Gantry speed during scan sweep; slower is safer on headless SBCs'),
        DeclareLaunchArgument('scan_tolerance_mm', default_value='0.5',
                              description='Arrival tolerance for scan cycle'),
        DeclareLaunchArgument('scan_timeout_s', default_value='90.0',
                              description='Per-segment arrival timeout'),
    ]

    micro_ros_agent = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'serial', '--dev', LaunchConfiguration('mcu_port'), '-b', '115200',
        ],
        output='screen',
        name='micro_ros_agent',
        condition=IfCondition(LaunchConfiguration('enable_micro_ros')),
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_a1_launch.py',
            )
        ),
        launch_arguments={
            'serial_port': LaunchConfiguration('lidar_port'),
            'frame_id': LaunchConfiguration('lidar_frame_id'),
            'scan_mode': LaunchConfiguration('scan_mode'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_lidar')),
    )

    scanner_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fusion_share, 'launch', 'scanner_bridge.launch.py')
        ),
    )

    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera_node',
        parameters=[{
            'video_device': LaunchConfiguration('camera_device'),
            'camera_info_url': LaunchConfiguration('camera_info_url'),
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
        }],
        remappings=[
            ('image_raw', '/image_raw'),
            ('camera_info', '/camera_info'),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_camera')),
    )

    camera_locks = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'v4l2-ctl', '-d', LaunchConfiguration('camera_device'),
                    '--set-ctrl=focus_automatic_continuous=0',
                    '--set-ctrl=white_balance_automatic=0',
                    '--set-ctrl=auto_exposure=1',
                    '--set-ctrl=exposure_dynamic_framerate=0',
                    '--set-ctrl=focus_absolute=0',
                    '--set-ctrl=exposure_time_absolute=500',
                    '--set-ctrl=white_balance_temperature=4000',
                    '--set-ctrl=gain=0',
                ],
                output='screen',
                condition=IfCondition(LaunchConfiguration('enable_camera_locks')),
            )
        ],
    )

    scan_assembler = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='lidar_camera_fusion',
                executable='scan_assembler_node',
                name='scan_assembler_node',
                parameters=[{
                    'target_frame': LaunchConfiguration('target_frame'),
                    'scan_topic': LaunchConfiguration('scan_topic'),
                    'position_topic': LaunchConfiguration('position_topic'),
                    'max_points': LaunchConfiguration('max_points'),
                    'publish_rate': LaunchConfiguration('publish_rate'),
                    'output_dir': LaunchConfiguration('output_dir'),
                }],
                output='screen',
            )
        ],
    )

    scan_cycle_node = TimerAction(
        period=3.5,
        actions=[
            Node(
                package='lidar_camera_fusion',
                executable='scan_cycle_node',
                name='scan_cycle_node',
                parameters=[{
                    'start_mm': LaunchConfiguration('scan_start_mm'),
                    'end_mm': LaunchConfiguration('scan_end_mm'),
                    'speed_mm_s': LaunchConfiguration('scan_speed_mm_s'),
                    'tolerance_mm': LaunchConfiguration('scan_tolerance_mm'),
                    'timeout_s': LaunchConfiguration('scan_timeout_s'),
                    'position_topic': LaunchConfiguration('position_topic'),
                    'target_position_topic': LaunchConfiguration('target_position_topic'),
                    'speed_topic': LaunchConfiguration('speed_topic'),
                }],
                output='screen',
                condition=IfCondition(LaunchConfiguration('enable_scan_cycle')),
            )
        ],
    )

    return LaunchDescription(args + [
        micro_ros_agent,
        rplidar_launch,
        scanner_bridge_launch,
        camera_node,
        camera_locks,
        scan_assembler,
        scan_cycle_node,
    ])
