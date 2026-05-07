#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Declare launch arguments ────────────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            'target_frame', default_value='base_link',
            description='TF frame to accumulate the assembled point cloud in'),
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan',
            description='LaserScan topic from the RPLiDAR A1'),
        DeclareLaunchArgument(
            'max_points', default_value='500000',
            description='Circular buffer capacity; oldest points are evicted first'),
        DeclareLaunchArgument(
            'publish_rate', default_value='2.0',
            description='Assembled-cloud publish rate in Hz'),
        DeclareLaunchArgument(
            'optical_frame', default_value='camera_optical_frame',
            description='Camera optical frame used for pinhole projection'),
        DeclareLaunchArgument(
            'queue_size', default_value='10',
            description='ApproximateTime synchronizer queue depth'),
        DeclareLaunchArgument(
            'approx_time_slop', default_value='0.1',
            description='Maximum time difference (s) for ApproximateTime sync'),
        # The rplidar_ros default launch sets frame_id:=laser.
        # If you change rplidar to publish in lidar_link directly,
        # pass scan_source_frame:=lidar_link here and remove the static TF.
        DeclareLaunchArgument(
            'scan_source_frame', default_value='laser',
            description='frame_id that rplidar publishes LaserScan in'),
    ]

    scan_source_frame = LaunchConfiguration('scan_source_frame')

    # ── Static TF: lidar_link → <scan_source_frame> ─────────────────────────
    # scanner_bridge.urdf defines lidar_link fixed to gantry_link, but
    # rplidar_ros publishes /scan with frame_id="laser" (its own default).
    # This static identity transform bridges the gap so laser_geometry can
    # resolve: base_link → gantry_link → lidar_link → laser
    #
    # NOTE: base_link and the rest of the TF tree come from robot_state_publisher
    # in scanner_bridge.launch.py — that launch file MUST be running before
    # or alongside this one, otherwise scan_assembler_node will warn
    # "'base_link' does not exist".
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_link_to_scan_frame_tf',
        arguments=[
            '--frame-id', 'lidar_link',
            '--child-frame-id', scan_source_frame,
        ],
        output='screen',
    )

    # ── scan_assembler_node ─────────────────────────────────────────────────
    scan_assembler = Node(
        package='lidar_camera_fusion',
        executable='scan_assembler_node',
        name='scan_assembler_node',
        parameters=[{
            'target_frame': LaunchConfiguration('target_frame'),
            'scan_topic':   LaunchConfiguration('scan_topic'),
            'max_points':   LaunchConfiguration('max_points'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }],
        output='screen',
    )

    # ── cloud_colorizer_node ────────────────────────────────────────────────
    cloud_colorizer = Node(
        package='lidar_camera_fusion',
        executable='cloud_colorizer_node',
        name='cloud_colorizer_node',
        parameters=[{
            'optical_frame':    LaunchConfiguration('optical_frame'),
            'queue_size':       LaunchConfiguration('queue_size'),
            'approx_time_slop': LaunchConfiguration('approx_time_slop'),
        }],
        output='screen',
    )

    return LaunchDescription(args + [static_tf_node, scan_assembler, cloud_colorizer])
