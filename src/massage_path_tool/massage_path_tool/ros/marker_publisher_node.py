#!/usr/bin/env python3
"""marker_publisher_node — publish trajectories_3d.json as RViz/Foxglove MarkerArray.

Usage:
    ros2 run massage_path_tool marker_publisher_node --ros-args \
        -p trajectories_file:=/path/to/trajectories_3d.json

Publishes one latched MarkerArray to /massage_trajectories_3d then exits:

  - DELETEALL marker first (no ghosts on re-publish)
  - SPHERE_LIST for all discrete points (single marker, per-point colors)
  - TEXT_VIEW_FACING per point (sequence labels)
  - LINE_STRIP per path (green→red colour gradient shows direction)
"""
from __future__ import annotations
import json
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


def _rgb(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = r, g, b, a
    return c


# Saturated palette for good visibility on dark RViz background
ORANGE = _rgb(1.0, 0.55, 0.0, 0.9)
GREEN  = _rgb(0.0, 0.9, 0.2, 0.85)
RED    = _rgb(0.95, 0.1, 0.1, 0.85)
WHITE  = _rgb(1.0, 1.0, 1.0, 0.85)
GREY   = _rgb(0.5, 0.5, 0.5, 0.5)
CYAN   = _rgb(0.0, 0.8, 0.8, 0.7)


def _lerp_color(c1: ColorRGBA, c2: ColorRGBA, t: float) -> ColorRGBA:
    """Linear interpolation between two RGBA colours."""
    t = max(0.0, min(1.0, t))
    return _rgb(
        c1.r + t * (c2.r - c1.r),
        c1.g + t * (c2.g - c1.g),
        c1.b + t * (c2.b - c1.b),
        c1.a + t * (c2.a - c1.a),
    )


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

        # Transient-local so late RViz/Foxglove subscribers still get markers
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pub = self.create_publisher(MarkerArray, '/massage_trajectories_3d', qos)
        ma = self._build_marker_array(data, now=self.get_clock().now().to_msg())
        pub.publish(ma)
        self.get_logger().info(
            f"Published {len(ma.markers)} markers to /massage_trajectories_3d — exiting"
        )
        raise SystemExit(0)

    # ── helpers ────────────────────────────────────────────────────

    def _make_marker(
        self, ns: str, id_: int, type_: int, now, frame: str
    ) -> Marker:
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = now
        m.ns = ns
        m.id = id_
        m.type = type_
        m.action = Marker.ADD
        m.lifetime.sec = 0  # persist
        return m

    def _point(self, x: float, y: float, z: float) -> Point:
        p = Point()
        p.x, p.y, p.z = x, y, z
        return p

    # ── build marker array ─────────────────────────────────────────

    def _build_marker_array(self, data: dict, now) -> MarkerArray:
        frame = data.get('frame', 'base_link')
        points = data.get('discrete_points', [])
        paths = data.get('paths', [])
        ma = MarkerArray()

        # 1. DELETEALL — clean slate so no ghosts from previous publish
        clear = Marker()
        clear.header.frame_id = frame
        clear.header.stamp = now
        clear.ns = 'clear'
        clear.id = 0
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        # 2. SPHERE_LIST — all discrete points in a single marker
        if points:
            sm = self._make_marker('points', 0, Marker.SPHERE_LIST, now, frame)
            sm.scale.x = 0.018
            sm.scale.y = 0.018
            sm.scale.z = 0.018
            sm.pose.orientation.w = 1.0
            for p in points:
                sm.points.append(self._point(p['x'], p['y'], p['z']))
                sm.colors.append(ORANGE)
            ma.markers.append(sm)

        # 3. TEXT_VIEW_FACING — label each discrete point with its ID
        for i, p in enumerate(points):
            tm = self._make_marker('labels', i, Marker.TEXT_VIEW_FACING, now, frame)
            tm.pose.position.x = p['x']
            tm.pose.position.y = p['y']
            tm.pose.position.z = p['z'] + 0.025  # float slightly above sphere
            tm.pose.orientation.w = 1.0
            tm.scale.z = 0.018  # text height
            tm.text = p.get('label', p.get('id', f'p{i}'))
            tm.color = WHITE
            ma.markers.append(tm)

        # 4. LINE_STRIP per path with green→red colour gradient
        for j, path_obj in enumerate(paths):
            wpts = path_obj.get('waypoints', [])
            if len(wpts) < 2:
                continue
            lm = self._make_marker(
                'paths', j + 10000, Marker.LINE_STRIP, now, frame,
            )
            lm.scale.x = 0.006
            lm.pose.orientation.w = 1.0
            n = len(wpts)
            for k, wp in enumerate(wpts):
                t = k / (n - 1) if n > 1 else 0.0
                lm.points.append(self._point(wp['x'], wp['y'], wp['z']))
                lm.colors.append(_lerp_color(GREEN, RED, t))
            ma.markers.append(lm)

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
