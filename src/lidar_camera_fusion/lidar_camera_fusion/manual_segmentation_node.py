#!/usr/bin/env python3
"""
manual_segmentation_node
========================
Interactive 2D polygon-selection GUI over a top-down (Z-projected) view
of /scanner/body_cloud. The user draws N labeled polygons; every body
point that falls inside a polygon (in X-Y) is tagged with the polygon's
label and republished on /scanner/segmented_regions as a PointCloud2 with
an extra `label` (uint8) field.

Workflow
--------
1. Wait for /scanner/body_cloud (latched snapshot from body_preprocess_node).
2. On user request (button / startup), grab the *latest* body cloud and
   open a pyqtgraph window with a top-down scatter (X horizontal, Y
   vertical, optional RGB if present).
3. User picks a label from a side panel, then draws a polygon. Repeat.
   Polygon overlap is resolved by *last-wins* — the most recently drawn
   polygon claims the point. This matches user expectation (draw → adjust).
4. "Publish" -> publish PointCloud2 with labels and save labeled PCD to
   <output_dir>/regions_<YYYYMMDD_HHMMSS>.pcd.

GUI design choices
------------------
- pyqtgraph + PyQt5 for fast scatter rendering on large clouds (10k+ pts
  stay fluid; matplotlib was the previous backend and got sluggish).
  pyqtgraph is the `python3-pyqtgraph` apt package.
- Qt input dialogs for new-label entry (no Tkinter dependency).
- The Qt event loop runs on the main thread; ROS spinning runs in a
  background thread. Cross-thread state is guarded by a single lock and the
  editor is opened from a QTimer poll on the main thread.
- We project on (x, y) with z mapped to a color ramp when no RGB is
  available, so the user can still see body contours.

Caveats
-------
- The view is a vertical (top-down) projection. Anything occluded along
  Z (e.g. arm-over-torso) gets the same label as whatever is below it.
  That is acceptable for the massage workflow per the supervisor.
- Label IDs are 1..N. 0 is reserved for "unlabeled" and is included in
  the output cloud (so downstream consumers can see every body point).
"""

import os
import struct
import threading
import time
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Bool, Header
from std_srvs.srv import Trigger


# PyQtGraph/Qt are imported lazily so headless unit tests can import the
# module without a display.
def _import_gui():
    # Import the editor module FIRST: it pins the PyQt5 binding (sets
    # PYQTGRAPH_QT_LIB and imports PyQt5 before pyqtgraph), matching the rest
    # of this workspace. Importing pyqtgraph.Qt before this would let it grab
    # the apt-pulled PyQt6 instead.
    try:
        from .segmentation_editor_qt import SegmentationEditor
    except ImportError:  # direct script execution / non-package import
        from segmentation_editor_qt import SegmentationEditor
    from pyqtgraph.Qt import QtWidgets
    return QtWidgets, SegmentationEditor


# ── Default region presets (the "vocabulary" the user will use) ────────────
DEFAULT_REGIONS = [
    "head",
    "shoulders",
    "upper_back",
    "lower_back",
    "left_arm",
    "right_arm",
    "left_leg",
    "right_leg",
    "other",
]


def _make_color(idx: int) -> Tuple[float, float, float]:
    """Stable distinct color per label index (1..N) using an HSV hash."""
    import colorsys
    h = (idx * 0.6180339887) % 1.0  # golden-ratio sweep -> well separated hues
    r, g, b = colorsys.hsv_to_rgb(h, 0.65, 0.95)
    return r, g, b


# ── Helpers: PointCloud2 <-> numpy ─────────────────────────────────────────

def _xyzrgb_from_msg(msg: PointCloud2) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """Returns (XYZ Nx3 float32, optional RGB Nx3 uint8) from a PointCloud2.

    Uses sensor_msgs_py.point_cloud2.read_points to get a structured array,
    then splits it cleanly.

    NOTE: we do NOT pass skip_nans=True. The PCL `rgb` field is a float32
    whose 32-bit pattern is 0x00RRGGBB; for many ordinary colors that bit
    pattern is itself a float NaN. read_points(skip_nans=True) would then
    drop every point whose *color* happens to look like NaN — silently
    deleting valid geometry (this caused body_*.pcd N pts -> regions_*.pcd
    fewer pts). We read everything and filter only on XYZ being finite, so
    color never removes a point.
    """
    field_names = {f.name for f in msg.fields}
    has_rgb = "rgb" in field_names

    fields = ("x", "y", "z", "rgb") if has_rgb else ("x", "y", "z")
    arr = pc2.read_points(msg, field_names=fields, skip_nans=False)
    if arr.size == 0:
        return np.empty((0, 3), dtype=np.float32), None

    x = np.asarray(arr["x"], dtype=np.float32)
    y = np.asarray(arr["y"], dtype=np.float32)
    z = np.asarray(arr["z"], dtype=np.float32)

    # Filter on geometry only — keep a point iff its x/y/z are all finite.
    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)

    xyz = np.stack([x[finite], y[finite], z[finite]], axis=-1).astype(np.float32)
    rgb_u8: Optional[np.ndarray] = None

    if has_rgb:
        # rgb is packed float32 whose 32-bit pattern is 0x00RRGGBB. Reinterpret
        # the raw bits as uint32 WITHOUT going through any float arithmetic
        # (which could normalize a NaN bit pattern and corrupt the color).
        rgb_raw = np.asarray(arr["rgb"])
        rgb_int = rgb_raw.view(np.uint32)[finite]
        r = ((rgb_int >> 16) & 0xFF).astype(np.uint8)
        g = ((rgb_int >>  8) & 0xFF).astype(np.uint8)
        b = ( rgb_int        & 0xFF).astype(np.uint8)
        rgb_u8 = np.stack([r, g, b], axis=-1)

    return xyz, rgb_u8


def _build_labeled_cloud(
    header: Header,
    xyz: np.ndarray,
    rgb_u8: Optional[np.ndarray],
    labels: np.ndarray,
) -> PointCloud2:
    """Pack XYZ(+RGB)+label into a PointCloud2.

    Layout is float32 x, float32 y, float32 z, [uint32 rgb,] uint8 label,
    aligned to a fixed point_step. We write raw bytes to keep dependencies
    minimal and to control padding.
    """
    n = xyz.shape[0]
    if n == 0:
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = 0
        msg.is_dense = False
        return msg

    has_rgb = rgb_u8 is not None
    if has_rgb:
        # x:0 y:4 z:8 rgb:12 label:16  -> point_step 20
        point_step = 20
        fields = [
            PointField(name="x",     offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y",     offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z",     offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb",   offset=12, datatype=PointField.UINT32,  count=1),
            PointField(name="label", offset=16, datatype=PointField.UINT8,   count=1),
        ]
        buf = np.zeros((n, point_step), dtype=np.uint8)
        # XYZ
        buf[:,  0: 4] = np.frombuffer(xyz[:, 0].astype(np.float32).tobytes(),
                                      dtype=np.uint8).reshape(n, 4)
        buf[:,  4: 8] = np.frombuffer(xyz[:, 1].astype(np.float32).tobytes(),
                                      dtype=np.uint8).reshape(n, 4)
        buf[:,  8:12] = np.frombuffer(xyz[:, 2].astype(np.float32).tobytes(),
                                      dtype=np.uint8).reshape(n, 4)
        # RGB packed as 0x00RRGGBB uint32 (matches PCL XYZRGB convention)
        rgb_packed = (
            (rgb_u8[:, 0].astype(np.uint32) << 16) |
            (rgb_u8[:, 1].astype(np.uint32) <<  8) |
            (rgb_u8[:, 2].astype(np.uint32))
        )
        buf[:, 12:16] = np.frombuffer(rgb_packed.tobytes(),
                                      dtype=np.uint8).reshape(n, 4)
        # label
        buf[:, 16] = labels.astype(np.uint8)
    else:
        # x:0 y:4 z:8 label:12  -> point_step 13 (we round to 16 for alignment)
        point_step = 16
        fields = [
            PointField(name="x",     offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y",     offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z",     offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name="label", offset=12, datatype=PointField.UINT8,   count=1),
        ]
        buf = np.zeros((n, point_step), dtype=np.uint8)
        buf[:,  0: 4] = np.frombuffer(xyz[:, 0].astype(np.float32).tobytes(),
                                      dtype=np.uint8).reshape(n, 4)
        buf[:,  4: 8] = np.frombuffer(xyz[:, 1].astype(np.float32).tobytes(),
                                      dtype=np.uint8).reshape(n, 4)
        buf[:,  8:12] = np.frombuffer(xyz[:, 2].astype(np.float32).tobytes(),
                                      dtype=np.uint8).reshape(n, 4)
        buf[:, 12] = labels.astype(np.uint8)

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = point_step
    msg.row_step = point_step * n
    msg.is_dense = False
    msg.data = buf.tobytes()
    return msg


def _save_pcd_ascii_with_label(
    path: str,
    xyz: np.ndarray,
    rgb_u8: Optional[np.ndarray],
    labels: np.ndarray,
) -> None:
    """Write a PCL-compatible ASCII .pcd with x y z [rgb] label.

    Matches PCL's XYZRGBL/XYZL conventions enough for CloudCompare to load
    the file and color points by label. The rgb column is packed as the
    standard PCL float (cast of 0x00RRGGBB).
    """
    n = xyz.shape[0]
    has_rgb = rgb_u8 is not None
    fields = ["x", "y", "z"]
    sizes  = [4, 4, 4]
    types_ = ["F", "F", "F"]
    counts = [1, 1, 1]
    if has_rgb:
        fields.append("rgb"); sizes.append(4); types_.append("F"); counts.append(1)
    fields.append("label"); sizes.append(4); types_.append("U"); counts.append(1)

    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        f"FIELDS {' '.join(fields)}\n"
        f"SIZE {' '.join(str(s) for s in sizes)}\n"
        f"TYPE {' '.join(types_)}\n"
        f"COUNT {' '.join(str(c) for c in counts)}\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA ascii\n"
    )

    with open(path, "w") as f:
        f.write(header)
        if has_rgb:
            rgb_packed = (
                (rgb_u8[:, 0].astype(np.uint32) << 16) |
                (rgb_u8[:, 1].astype(np.uint32) <<  8) |
                (rgb_u8[:, 2].astype(np.uint32))
            )
            # cast packed uint32 to float32 bit-pattern (PCL convention)
            rgb_float = rgb_packed.view(np.float32)
            for i in range(n):
                f.write(
                    f"{xyz[i,0]:.6f} {xyz[i,1]:.6f} {xyz[i,2]:.6f} "
                    f"{rgb_float[i]:.6e} {int(labels[i])}\n"
                )
        else:
            for i in range(n):
                f.write(
                    f"{xyz[i,0]:.6f} {xyz[i,1]:.6f} {xyz[i,2]:.6f} "
                    f"{int(labels[i])}\n"
                )


# ── Node ───────────────────────────────────────────────────────────────────

class ManualSegmentationNode(Node):
    def __init__(self) -> None:
        super().__init__("manual_segmentation_node")

        # Parameters
        self.declare_parameter("input_topic",  "/scanner/body_cloud")
        self.declare_parameter("output_topic", "/scanner/segmented_regions")
        self.declare_parameter("output_dir",   os.path.expanduser("~/ros2_scans"))
        self.declare_parameter("region_labels", DEFAULT_REGIONS)
        self.declare_parameter("auto_open_on_stop", True)
        self.declare_parameter("frame_id",     "base_link")

        self.input_topic_   = self.get_parameter("input_topic").get_parameter_value().string_value
        self.output_topic_  = self.get_parameter("output_topic").get_parameter_value().string_value
        self.output_dir_    = self.get_parameter("output_dir").get_parameter_value().string_value
        self.region_labels_ = list(
            self.get_parameter("region_labels").get_parameter_value().string_array_value
        ) or DEFAULT_REGIONS
        self.auto_open_on_stop_ = self.get_parameter("auto_open_on_stop").get_parameter_value().bool_value
        self.fallback_frame_id_ = self.get_parameter("frame_id").get_parameter_value().string_value

        # State (guarded by lock)
        self._lock = threading.Lock()
        self._latest_msg: Optional[PointCloud2] = None
        self._open_request = threading.Event()
        self._open_pending = False  # armed on scan-stop, fired when cloud arrives
        self._scanning = False

        # Subscribers
        # body_preprocess_node publishes /scanner/body_cloud exactly once per
        # scan (on stop) as RELIABLE + TRANSIENT_LOCAL (latched). Match that QoS
        # so we still receive the latched cloud even if this node connects after
        # the publish, and so the reliability policies are compatible.
        body_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PointCloud2, self.input_topic_, self._cloud_cb, qos_profile=body_qos)
        self.create_subscription(
            Bool, "/scanner/is_scanning", self._is_scanning_cb,
            QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        # Publisher
        self.pub_ = self.create_publisher(PointCloud2, self.output_topic_, 5)

        # Service: lets the user pop the GUI from CLI
        # ros2 service call /scanner/open_segmenter std_srvs/srv/Trigger
        self.create_service(Trigger, "/scanner/open_segmenter",
                            self._open_segmenter_srv)

        os.makedirs(self.output_dir_, exist_ok=True)

        self.get_logger().info(
            f"manual_segmentation_node: subscribing {self.input_topic_} -> "
            f"publishing {self.output_topic_}; "
            f"output_dir={self.output_dir_}; "
            f"labels={self.region_labels_}; "
            f"auto_open_on_stop={self.auto_open_on_stop_}")

    # ── Subscribers ────────────────────────────────────────────────────────

    def _cloud_cb(self, msg: PointCloud2) -> None:
        with self._lock:
            self._latest_msg = msg
            pending = self._open_pending
            self._open_pending = False
        # If a scan just stopped and we were waiting for the cleaned body
        # cloud, this is it — open the editor now that we actually have data.
        if pending and self.auto_open_on_stop_:
            self.get_logger().info(
                "Body cloud received after scan stop — opening segmenter window")
            self._open_request.set()

    def _is_scanning_cb(self, msg: Bool) -> None:
        was_scanning = self._scanning
        self._scanning = bool(msg.data)
        # Edge: scanning just went False -> True  =>  new scan started.
        # Clear any latched/stale body cloud from a PREVIOUS scan NOW, while
        # there is plenty of time before the next stop. body_preprocess_node
        # only publishes its body cloud at scan STOP, so between here and the
        # next stop _latest_msg legitimately stays None. This is what prevents
        # the stop handler from ever opening with last scan's data.
        if not was_scanning and self._scanning:
            with self._lock:
                self._latest_msg = None
                self._open_pending = False
            return
        # Edge: scanning just went True -> False  =>  scan finished.
        # Because we cleared at scan-start, _latest_msg is non-None here ONLY
        # if THIS scan's fresh body cloud already arrived (body_preprocess can
        # publish before our stop message lands — DDS isn't cross-topic
        # ordered). In that case open immediately; otherwise arm pending and
        # let _cloud_cb open the window once the fresh cloud arrives.
        if was_scanning and not self._scanning and self.auto_open_on_stop_:
            with self._lock:
                have = self._latest_msg is not None
                self._open_pending = not have
            if have:
                self.get_logger().info(
                    "Scan finished — fresh body cloud already in; opening window")
                self._open_request.set()
            else:
                self.get_logger().info(
                    "Scan finished — waiting for body_preprocess_node cloud …")

    def _open_segmenter_srv(self, request, response):
        with self._lock:
            have = self._latest_msg is not None
        if not have:
            response.success = False
            response.message = (
                f"No body cloud received yet on {self.input_topic_}; "
                "is body_preprocess_node running?")
            return response
        self._open_request.set()
        response.success = True
        response.message = "Segmenter window requested"
        return response

    # ── GUI loop (runs on the main thread) ────────────────────────────────

    def gui_loop_blocking(self) -> None:
        """Blocking Qt loop on the main thread. ROS spinning runs on a worker
        thread; Qt/pyqtgraph stay on the main thread."""
        QtWidgets, SegmentationEditor = _import_gui()
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        # Do NOT quit when the editor window is closed — this node lives for
        # the whole session and must survive between scans so the next scan can
        # pop a fresh window. Without this, closing the first editor returns
        # from app.exec_() and the node shuts down ("finished cleanly").
        app.setQuitOnLastWindowClosed(False)
        self._qapp = app
        # Strong reference to the live editor so it isn't garbage-collected the
        # moment _run_editor returns (show() is non-blocking).
        self._editor = None

        def _poll_open_request():
            if not self._open_request.is_set():
                return
            self._open_request.clear()
            with self._lock:
                msg = self._latest_msg
            if msg is None:
                self.get_logger().warn("Open requested but no cloud cached yet")
                return
            try:
                self._run_editor(msg, SegmentationEditor)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"Segmentation editor crashed: {exc}")

        # Quit the Qt loop when rclpy shuts down (Ctrl-C on the launch).
        def _check_ros():
            if not rclpy.ok():
                self._qapp.quit()

        from pyqtgraph.Qt import QtCore
        qt_timer = QtCore.QTimer()
        qt_timer.timeout.connect(_poll_open_request)
        qt_timer.start(200)
        ros_timer = QtCore.QTimer()
        ros_timer.timeout.connect(_check_ros)
        ros_timer.start(500)
        # exec() works on both PyQt5 and PyQt6 (exec_() is PyQt5-only).
        app.exec()
        qt_timer.stop()
        ros_timer.stop()

    def _run_editor(self, msg: PointCloud2, SegmentationEditor) -> None:
        xyz, rgb = _xyzrgb_from_msg(msg)
        if xyz.shape[0] == 0:
            self.get_logger().warn("Body cloud is empty — nothing to segment")
            return
        frame_id = msg.header.frame_id or self.fallback_frame_id_
        labels = np.zeros(xyz.shape[0], dtype=np.uint8)
        if rgb is not None:
            base_colors = rgb.astype(np.float32) / 255.0
        else:
            z = xyz[:, 2]
            zmin, zmax = float(z.min()), float(z.max())
            denom = max(zmax - zmin, 1e-6)
            v = (z - zmin) / denom
            base_colors = np.stack([v, 1.0 - v, 0.5 * np.ones_like(v)], axis=1).astype(np.float32)

        def _publish(polygons):
            self._publish_and_save(msg, xyz, rgb, labels, polygons, frame_id)

        editor = SegmentationEditor(
            xyz=xyz,
            base_colors=base_colors,
            labels=labels,
            region_labels=self.region_labels_,
            make_color=_make_color,
            on_publish=_publish,
            title=f"Manual Segmentation — frame={frame_id}, points={xyz.shape[0]}",
            log=lambda m: self.get_logger().info(str(m)),
        )
        # Keep a strong reference so the window isn't GC'd when this method
        # returns (show() is non-blocking). Replaces any previous editor.
        self._editor = editor
        editor.run()

    def _publish_and_save(
        self,
        msg: PointCloud2,
        xyz: np.ndarray,
        rgb: Optional[np.ndarray],
        labels: np.ndarray,
        polygons: List[Tuple[int, str, np.ndarray]],
        frame_id: str,
    ) -> None:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id

        out_msg = _build_labeled_cloud(header, xyz, rgb, labels)
        self.pub_.publish(out_msg)

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        os.makedirs(self.output_dir_, exist_ok=True)
        pcd_path = os.path.join(self.output_dir_, f"regions_{ts}.pcd")
        _save_pcd_ascii_with_label(pcd_path, xyz, rgb, labels)

        # Sidecar JSON with polygon definitions, so the user (and future
        # reviewers) can replay the exact selection later.
        import json
        sidecar = {
            "frame_id": frame_id,
            "timestamp": ts,
            "labels": [
                {"id": int(lid), "name": name,
                 "vertices": np.asarray(verts).tolist()}
                for (lid, name, verts) in polygons
            ],
            "label_counts": {
                str(int(li)): int(np.sum(labels == li))
                for li in np.unique(labels)
            },
            "total_points": int(xyz.shape[0]),
        }
        with open(pcd_path.replace(".pcd", ".json"), "w") as f:
            json.dump(sidecar, f, indent=2)

        self.get_logger().info(
            f"Published {xyz.shape[0]} labeled pts to {self.output_topic_} | "
            f"saved -> {pcd_path}")


# ── Entry point ────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManualSegmentationNode()

    # rclpy spinning runs on a worker thread; Qt/pyqtgraph need the main thread.
    spinner = threading.Thread(
        target=rclpy.spin, args=(node,), name="rclpy_spin", daemon=True)
    spinner.start()

    try:
        node.gui_loop_blocking()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spinner.join(timeout=1.0)


if __name__ == "__main__":
    main()
