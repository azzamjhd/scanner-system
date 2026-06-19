#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
            'rectify', default_value='true',
            description='Option A: run image_proc/rectify_node to undistort the raw '
                        'camera image, and colorize from the rectified image using '
                        'CameraInfo.P. Set false to fall back to raw-image colorizing '
                        '(legacy behaviour, distorted at image edges).'),
        DeclareLaunchArgument(
            'raw_image_topic', default_value='/image_raw',
            description='Raw (distorted) camera image topic from the driver. Input to '
                        'rectify_node.'),
        DeclareLaunchArgument(
            'rect_image_topic', default_value='/image_rect_color',
            description='Rectified image topic produced by rectify_node.'),
        DeclareLaunchArgument(
            'colorizer_image_topic', default_value='/image_rect_color',
            description='Image topic consumed by cloud_colorizer_node. Default is the '
                        'rectified image for Option A.'),
        DeclareLaunchArgument(
            'colorizer_camera_info_topic', default_value='/camera_info',
            description='CameraInfo topic the colorizer reads intrinsics from. With '
                        'Option A, the colorizer uses the P (rectified projection) '
                        'matrix from this message.'),
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
        DeclareLaunchArgument(
            'output_dir', default_value='.',
            description='Directory where timestamped .pcd files are saved on /scanner/stop'),
        DeclareLaunchArgument(
            'add_lidar_static_tf', default_value='true',
            description='Publish a static identity TF from lidar_link to scan_source_frame. '
                        'Set to false when scan_source_frame is already a link in the '
                        'URDF TF tree (e.g. lidar_link) to avoid a self-transform error.'),
        DeclareLaunchArgument(
            'image_topic', default_value='/image_raw',
            description='Camera image topic consumed by scan_image_recorder_node'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/camera_info',
            description='Camera intrinsics topic. scan_image_recorder_node reads fx '
                        'from this to auto-derive pixels_per_mm.'),
        DeclareLaunchArgument(
            'use_lidar_distance', default_value='true',
            description='scan_image_recorder_node: use live LiDAR distance from /scan '
                        'as the camera-to-subject distance Z. When false, '
                        'working_distance_mm is used as a fixed estimate.'),
        DeclareLaunchArgument(
            'min_capture_spacing_mm', default_value='10.0',
            description='scan_image_recorder_node: minimum gantry travel (mm) between '
                        'captured frames. Decouples capture density from gantry speed; '
                        'set to 0 to capture on every colored-cloud message.'),
        DeclareLaunchArgument(
            'undistort_images', default_value='true',
            description='scan_image_recorder_node: rectify each frame using K and D '
                        'from /camera_info before stitching. Eliminates lens-distortion '
                        'side-to-side jitter on cameras with significant tangential '
                        'distortion (e.g. C922 webcam).'),
        DeclareLaunchArgument(
            'lidar_distance_quantile', default_value='0.25',
            description='scan_image_recorder_node: take the median of the closest '
                        'fraction of valid LiDAR ranges as the subject distance. '
                        '0.25 = closest 25 %% of beams.'),
        DeclareLaunchArgument(
            'working_distance_mm', default_value='500.0',
            description='scan_image_recorder_node: fallback camera → subject distance '
                        'in mm when use_lidar_distance is false or no /scan data yet.'),
        DeclareLaunchArgument(
            'pixels_per_mm', default_value='0.0',
            description='scan_image_recorder_node: manual override for canvas scale. '
                        '0 = auto-compute from camera intrinsics + working_distance_mm. '
                        'Set non-zero only to force a specific value.'),
    ]

    scan_source_frame = LaunchConfiguration('scan_source_frame')

    # ── image_proc rectify_node (Option A) ──────────────────────────────────
    # Undistorts /image_raw -> /image_rect_color using K and D from /camera_info.
    # CameraSubscriber resolves 'camera_info' as a sibling of the 'image' topic,
    # so remapping image -> /image_raw makes it read /camera_info automatically.
    # The colorizer then projects with CameraInfo.P (distortion already removed),
    # eliminating the raw-image + pinhole-only side misalignment.
    rectify_node = Node(
        package='image_proc',
        executable='rectify_node',
        name='rectify_node',
        output='screen',
        remappings=[
            ('image',      LaunchConfiguration('raw_image_topic')),
            ('image_rect', LaunchConfiguration('rect_image_topic')),
        ],
        condition=IfCondition(LaunchConfiguration('rectify')),
    )

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
        condition=IfCondition(LaunchConfiguration('add_lidar_static_tf')),
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
            'output_dir':   LaunchConfiguration('output_dir'),
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
            'image_topic':      LaunchConfiguration('colorizer_image_topic'),
            'camera_info_topic': LaunchConfiguration('colorizer_camera_info_topic'),
            'queue_size':       LaunchConfiguration('queue_size'),
            'approx_time_slop': LaunchConfiguration('approx_time_slop'),
            'output_dir':       LaunchConfiguration('output_dir'),
        }],
        output='screen',
    )

    # ── scan_image_recorder_node ─────────────────────────────────────────────────────────
    scan_image_recorder = Node(
        package='lidar_camera_fusion',
        executable='scan_image_recorder_node',
        name='scan_image_recorder_node',
        parameters=[{
            'output_dir':              LaunchConfiguration('output_dir'),
            'image_topic':             LaunchConfiguration('image_topic'),
            'camera_info_topic':       LaunchConfiguration('camera_info_topic'),
            'scan_topic':              LaunchConfiguration('scan_topic'),
            'min_capture_spacing_mm':  LaunchConfiguration('min_capture_spacing_mm'),
            'undistort_images':        LaunchConfiguration('undistort_images'),
            'use_lidar_distance':      LaunchConfiguration('use_lidar_distance'),
            'lidar_distance_quantile': LaunchConfiguration('lidar_distance_quantile'),
            'working_distance_mm':     LaunchConfiguration('working_distance_mm'),
            'pixels_per_mm':           LaunchConfiguration('pixels_per_mm'),
        }],
        output='screen',
    )

    return LaunchDescription(args + [rectify_node, static_tf_node, scan_assembler, cloud_colorizer, scan_image_recorder])
