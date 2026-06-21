#!/usr/bin/env python3
"""marker_publisher_node — publish trajectories_3d.json as RViz MarkerArray.

Usage:
    ros2 run massage_path_tool marker_publisher_node --ros-args \\
        -p trajectories_file:=/path/to/trajectories_3d.json

Publishes one latched MarkerArray to /massage_trajectories_3d then exits.
Discrete points → orange spheres, paths → cyan line strips.
"""
from __future__ import annotations
import json
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

COLOUR_POINT = (1.0, 0.5, 0.0, 0.9)   # R,G,B,A — orange
COLOUR_PATH  = (0.0, 0.8, 0.8, 0.7)   # cyan


class MarkerPublisherNode(Node):
    """Read trajectories_3d.json and publish a MarkerArray once, then exit."""

    def __init__(self) -> None:
        super().__init__('marker_publisher_node')

        self.declare_parameter('trajectories_file', 'trajectories_3d.json')
        path = self.get_parameter('trajectories_file').value

        if not path or not Path(path).exists():
            self.get_logger().error(f"trajectories_file not found: '{path}'")
            raise SystemExit(1)

        with open(path) as f:
            data = json.load(f)

        pub = self.create_publisher(MarkerArray, '/massage_trajectories_3d', 1)
        ma = self._build_marker_array(data, now=self.get_clock().now().to_msg())
        pub.publish(ma)
        self.get_logger().info(
            f"Published {len(ma.markers)} markers to /massage_trajectories_3d — exiting"
        )
        raise SystemExit(0)

    def _build_marker_array(self, data: dict, now) -> MarkerArray:
        frame = data.get('frame', 'base_link')
        ma = MarkerArray()

        # Discrete points → SPHERE_LIST (or individual POINT markers)
        points = data.get('discrete_points', [])
        for i, p in enumerate(points):
            m = Marker()
            m.header.frame_id = frame
            m.header.stamp = now
            m.ns = 'massage_points'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = p['x']
            m.pose.position.y = p['y']
            m.pose.position.z = p['z']
            m.pose.orientation.w = 1.0
            m.scale.x = 0.015
            m.scale.y = 0.015
            m.scale.z = 0.015
            r, g, b, a = COLOUR_POINT
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a
            m.lifetime.sec = 0  # forever
            ma.markers.append(m)

        # Paths → LINE_STRIP
        for j, path in enumerate(data.get('paths', [])):
            waypoints = path.get('waypoints', [])
            if len(waypoints) < 2:
                continue
            m = Marker()
            m.header.frame_id = frame
            m.header.stamp = now
            m.ns = 'massage_paths'
            m.id = j + 10000
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = 0.005
            r, g, b, a = COLOUR_PATH
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a
            for wp in waypoints:
                pt = Point()
                pt.x, pt.y, pt.z = wp['x'], wp['y'], wp['z']
                m.points.append(pt)
            ma.markers.append(m)

        return ma


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        MarkerPublisherNode()
    except SystemExit:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
