#!/usr/bin/env python3
"""
segmentation.launch.py
======================
Chains the segmentation pipeline on top of the colorizer's output:

    /scanner/colored_cloud  ──►  body_preprocess_node  ──►  /scanner/body_cloud
                                                              │
                                                              ▼
                                              manual_segmentation_node
                                                              │
                                                              ▼
                                                /scanner/segmented_regions

This launch file is meant to be composed with full_system.launch.py — it
does NOT bring up the LiDAR / camera / micro-ROS / state-publisher.
Run it AFTER full_system.launch.py (or alongside it):

    # one terminal: hardware + fusion + colorizer
    ros2 launch lidar_camera_fusion full_system.launch.py
    # another terminal: just the segmentation chain
    ros2 launch lidar_camera_fusion segmentation.launch.py

Or include it from full_system.launch.py with `enable_segmentation:=true`.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "input_topic", default_value="/scanner/colored_cloud",
            description="PointCloud2 topic feeding body_preprocess_node "
                        "(must be in base_link)."),
        DeclareLaunchArgument(
            "body_topic", default_value="/scanner/body_cloud",
            description="Output of body_preprocess_node, input of "
                        "manual_segmentation_node."),
        DeclareLaunchArgument(
            "regions_topic", default_value="/scanner/segmented_regions",
            description="Final labeled-cloud topic published by "
                        "manual_segmentation_node."),
        DeclareLaunchArgument(
            "output_dir",
            default_value=os.path.expanduser("~/ros2_scans"),
            description="Where body_<ts>.pcd and regions_<ts>.pcd are saved."),

        # ── RANSAC bed-plane removal tunables ───────────────────────────────
        DeclareLaunchArgument(
            "z_min", default_value="-0.50",
            description="body_preprocess: lower bound of the Z slab cropped "
                        "before RANSAC, in metres in base_link. Anything "
                        "below z_min is dropped."),
        DeclareLaunchArgument(
            "z_max", default_value="0.80",
            description="body_preprocess: upper bound of the Z slab. Tighten "
                        "this if the ceiling/walls leak in."),
        DeclareLaunchArgument(
            "plane_distance_thresh", default_value="0.005",
            description="body_preprocess: RANSAC inlier threshold in metres "
                        "(5 mm matches the spec)."),
        DeclareLaunchArgument(
            "plane_max_iter", default_value="200",
            description="body_preprocess: RANSAC iteration cap."),
        DeclareLaunchArgument(
            "plane_eps_angle_deg", default_value="15.0",
            description="body_preprocess: max angle (deg) between the fitted "
                        "plane normal and +Z. Forces a horizontal bed."),

        # ── Cluster extraction ─────────────────────────────────────────────
        DeclareLaunchArgument(
            "enable_cluster", default_value="true",
            description="body_preprocess: keep only the largest Euclidean "
                        "cluster after plane removal."),
        DeclareLaunchArgument(
            "cluster_tolerance", default_value="0.02",
            description="body_preprocess: Euclidean cluster radius in metres."),
        DeclareLaunchArgument(
            "cluster_min_size", default_value="500",
            description="body_preprocess: smallest acceptable cluster."),
        DeclareLaunchArgument(
            "cluster_max_size", default_value="2000000",
            description="body_preprocess: safety cap on cluster size."),

        # ── Manual segmentation GUI ────────────────────────────────────────
        DeclareLaunchArgument(
            "auto_open_on_stop", default_value="true",
            description="manual_segmentation: pop the polygon GUI "
                        "automatically when /scanner/is_scanning flips false."),
    ]

    body_preprocess = Node(
        package="lidar_camera_fusion",
        executable="body_preprocess_node",
        name="body_preprocess_node",
        output="screen",
        parameters=[{
            "input_topic":           LaunchConfiguration("input_topic"),
            "output_topic":          LaunchConfiguration("body_topic"),
            "output_dir":            LaunchConfiguration("output_dir"),
            "z_min":                 LaunchConfiguration("z_min"),
            "z_max":                 LaunchConfiguration("z_max"),
            "plane_distance_thresh": LaunchConfiguration("plane_distance_thresh"),
            "plane_max_iter":        LaunchConfiguration("plane_max_iter"),
            "plane_eps_angle_deg":   LaunchConfiguration("plane_eps_angle_deg"),
            "enable_cluster":        LaunchConfiguration("enable_cluster"),
            "cluster_tolerance":     LaunchConfiguration("cluster_tolerance"),
            "cluster_min_size":      LaunchConfiguration("cluster_min_size"),
            "cluster_max_size":      LaunchConfiguration("cluster_max_size"),
        }],
    )

    manual_segmentation = Node(
        package="lidar_camera_fusion",
        executable="manual_segmentation_node",
        name="manual_segmentation_node",
        output="screen",
        parameters=[{
            "input_topic":       LaunchConfiguration("body_topic"),
            "output_topic":      LaunchConfiguration("regions_topic"),
            "output_dir":        LaunchConfiguration("output_dir"),
            "auto_open_on_stop": LaunchConfiguration("auto_open_on_stop"),
        }],
    )

    return LaunchDescription(args + [body_preprocess, manual_segmentation])
