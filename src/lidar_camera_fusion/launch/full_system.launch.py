#!/usr/bin/env python3
"""
full_system.launch.py
=====================
Single-command bringup for the lidar_camera_fusion pipeline.

Starts, in order:
  1. micro-ROS agent          – ESP32 firmware ↔ ROS 2 bridge
  2. RPLiDAR A1               – publishes /scan  (frame_id=lidar_link)
  3. robot_state_publisher    – broadcasts TF tree from scanner_bridge.urdf
  4. v4l2_camera_node         – publishes /image_raw + /camera_info
  5. fusion.launch.py         – scan_assembler_node + cloud_colorizer_node
                                (static TF lidar_link→laser is skipped because
                                 lidar_link is already in the URDF TF tree)

Note: gantry_bridge_node is NOT a separate process — its mm→m conversion is
built directly into scan_assembler_node (position_callback → /joint_states).

Usage
-----
# All defaults match the hardware setup
ros2 launch lidar_camera_fusion full_system.launch.py

# Override anything
ros2 launch lidar_camera_fusion full_system.launch.py \\
    output_dir:=/home/azzam/scans \\
    max_points:=1000000

Scan workflow (CLI)
-------------------
ros2 service call /scanner/start std_srvs/srv/Trigger
ros2 topic pub /position std_msgs/msg/Float32 "{data: 100.0}" -1
ros2 service call /scanner/stop  std_srvs/srv/Trigger
# → saves  <output_dir>/scan_YYYYMMDD_HHMMSS.pcd
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node





def generate_launch_description():

    # ── Package share paths ────────────────────────────────────────────────
    fusion_share = get_package_share_directory('lidar_camera_fusion')

    # ── Launch arguments ───────────────────────────────────────────────────
    args = [
        # ── Hardware ports ─────────────────────────────────────────────────
        DeclareLaunchArgument(
            'mcu_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for the micro-ROS / ESP32 firmware'),
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/ttyUSB1',
            description='Serial port for RPLiDAR A1'),
        DeclareLaunchArgument(
            'camera_device',
            default_value='/dev/video0',
            description='V4L2 device node for the USB camera'),

        # ── LiDAR ──────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            'lidar_frame_id',
            default_value='lidar_link',
            description='TF frame_id that rplidar_ros stamps on /scan. '
                        'Must match a frame in the URDF TF tree or have a '
                        'static TF bridging it (see add_lidar_static_tf).'),
        DeclareLaunchArgument(
            'scan_mode',
            default_value='Boost',
            description='RPLiDAR A1 scan mode: Standard (2K pts), '
                        'Express (4K pts), or Boost (8K pts). '
                        'Sensitivity is NOT supported by the A1.'),

        # ── Camera ─────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='file:///home/azzam/Documents/webcam_calibration.yaml',
            description='URL to the camera calibration YAML '
                        '(file:///absolute/path.yaml or package://...)'),
        DeclareLaunchArgument(
            'camera_frame_id',
            default_value='camera_optical_frame',
            description='TF frame stamped on camera images — must match the '
                        'optical frame in scanner_bridge.urdf'),
        # NOTE: V4L2 hardware controls (focus_absolute, exposure_time_absolute)
        # are NOT passed as launch params. The C922 exposes them as UVC
        # extension controls that v4l2_camera cannot enumerate at startup
        # ("Available controls:" shows empty). Applying them from a params
        # file before streaming begins corrupts VIDIOC_REQBUFS (MMAP setup).
        # Set them AFTER the node is running:
        #   ros2 param set /v4l2_camera_node focus_absolute 0
        #   ros2 param set /v4l2_camera_node exposure_time_absolute 512

        # ── scan_assembler_node ────────────────────────────────────────────
        DeclareLaunchArgument(
            'output_dir',
            default_value=os.path.expanduser('~/ros2_scans'),
            description='Directory where /scanner/stop saves timestamped .pcd files'),
        DeclareLaunchArgument(
            'max_points',
            default_value='500000',
            description='Circular buffer capacity in scan_assembler_node'),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='2.0',
            description='Assembled-cloud publish rate in Hz'),

        # ── fusion.launch.py passthrough ───────────────────────────────────
        DeclareLaunchArgument(
            'scan_source_frame',
            default_value='lidar_link',
            description='frame_id that rplidar_ros publishes /scan in. '
                        'Passed to fusion.launch.py so the static TF bridge '
                        'is only added when the frame is NOT already in the '
                        'URDF TF tree (e.g. use "laser" for the rplidar '
                        'default, "lidar_link" for this hardware setup).'),
    ]

    # ── 1. micro-ROS agent ─────────────────────────────────────────────────
    # Bridges the ESP32 firmware to ROS 2 over serial.
    # Publishes:   /current_position  (std_msgs/Float32, mm)
    #              /motor_speed       (std_msgs/Float32)
    # Subscribes:  /position /speed /acceleration  (std_msgs/Float32)
    micro_ros_agent = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'serial',
            '--dev', LaunchConfiguration('mcu_port'),
            '-b', '115200',
        ],
        output='screen',
        name='micro_ros_agent',
    )

    # ── 2. RPLiDAR A1 ─────────────────────────────────────────────────────
    # Publishes /scan (sensor_msgs/LaserScan).
    # frame_id is set to lidar_link so it maps directly onto the URDF link —
    # no separate static TF bridge is needed.
    # scan_mode must be Standard/Express/Boost — the A1 does NOT support
    # the rplidar_ros default "Sensitivity" mode (causes exit code 255).
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
            'frame_id':    LaunchConfiguration('lidar_frame_id'),
            'scan_mode':   LaunchConfiguration('scan_mode'),
        }.items(),
    )

    # ── 3. robot_state_publisher ───────────────────────────────────────────
    # Reads urdf/scanner_bridge.urdf and broadcasts:
    #   base_link → gantry_link → lidar_link
    #   gantry_link → camera_link → camera_optical_frame
    # The gantry_joint edge is driven by /joint_states, published by
    # scan_assembler_node's built-in position_callback.
    scanner_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fusion_share, 'launch', 'scanner_bridge.launch.py')
        ),
    )

    # ── 4. USB camera ──────────────────────────────────────────────────────
    # Only camera_info_url and camera_frame_id are passed at startup.
    # V4L2 hardware controls (focus, exposure) must be set post-startup via
    # ros2 param set — see the launch arg comments above.
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera_node',
        parameters=[{
            'video_device':    LaunchConfiguration('camera_device'),
            'camera_info_url': LaunchConfiguration('camera_info_url'),
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
        }],
        remappings=[
            ('image_raw',   '/image_raw'),
            ('camera_info', '/camera_info'),
        ],
        output='screen',
    )

    # ── 5. Fusion pipeline ─────────────────────────────────────────────────
    # Delayed 2 s to give robot_state_publisher time to fill the TF buffer
    # before scan_assembler_node starts calling lookupTransform.
    #
    # add_lidar_static_tf=false: lidar_link is already in the URDF TF tree,
    # so fusion.launch.py must NOT publish lidar_link→lidar_link (self-transform
    # which tf2 rejects).  Only needed when scan_source_frame="laser" (rplidar
    # default) which is NOT in the URDF.
    fusion_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(fusion_share, 'launch', 'fusion.launch.py')
                ),
                launch_arguments={
                    'output_dir':           LaunchConfiguration('output_dir'),
                    'max_points':           LaunchConfiguration('max_points'),
                    'publish_rate':         LaunchConfiguration('publish_rate'),
                    'scan_source_frame':    LaunchConfiguration('scan_source_frame'),
                    'add_lidar_static_tf':  'false',
                }.items(),
            )
        ],
    )

    return LaunchDescription(args + [
        micro_ros_agent,
        rplidar_launch,
        scanner_bridge_launch,
        camera_node,
        fusion_launch,
    ])
