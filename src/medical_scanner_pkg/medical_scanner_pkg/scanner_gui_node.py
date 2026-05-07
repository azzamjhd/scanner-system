#!/usr/bin/env python3

import json
import os
import subprocess
import sys
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtGui import QImage
from PyQt5.QtWidgets import QApplication
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger

from medical_scanner_pkg.scanner_gui_widget import ScannerGUI


class _ROSBridge(QObject):
    """Thread-safe bridge: emits Qt signals from ROS callbacks into the GUI main thread."""

    position_updated = pyqtSignal(float)
    rpm_updated = pyqtSignal(float)
    points_updated = pyqtSignal(int)
    image_updated = pyqtSignal(QImage)
    connected_updated = pyqtSignal(bool)
    stitcher_log = pyqtSignal(str)  # stitcher service feedback → GUI log
    stitcher_status_updated = pyqtSignal(str, bool)  # (label_text, ok) → status label


class ScannerROSNode(Node):
    def __init__(self, bridge: _ROSBridge):
        super().__init__("scanner_gui_ros_node")
        self._bridge = bridge

        # Publishers
        self.position_pub = self.create_publisher(Float32, "/position", 10)
        self.speed_pub = self.create_publisher(Float32, "/speed", 10)
        self.accel_pub = self.create_publisher(Float32, "/acceleration", 10)
        self.gantry_pub = self.create_publisher(Vector3, "/scanner/cmd", 10)

        # Subscribers
        self.create_subscription(Float32, "/current_position", self._on_position, 10)
        self.create_subscription(Float32, "/motor_speed", self._on_motor_speed, 10)
        self.create_subscription(
            PointCloud2, "/scanner/pointcloud", self._on_pointcloud, 10
        )

        self.declare_parameter("image_topic", "/camera/image_raw")
        self._image_topic = self.get_parameter("image_topic").value
        self._image_sub = self.create_subscription(
            Image, self._image_topic, self._on_image, 10
        )

        # Stitcher integration (enabled by default; set to false to disable)
        self.declare_parameter("enable_stitcher_integration", True)
        self.declare_parameter("stitcher_start_service", "/stitcher/start_session")
        self.declare_parameter("stitcher_stop_service", "/stitcher/stop_session")
        self._stitcher_enabled = bool(
            self.get_parameter("enable_stitcher_integration").value
        )

        # Service clients
        self.start_scan_client = self.create_client(Trigger, "/start_scan")
        self.stop_scan_client = self.create_client(Trigger, "/stop_scan")
        self.clear_viz_client = self.create_client(Trigger, "/clear_visualization")
        self.stitcher_start_client = self.create_client(
            Trigger, self.get_parameter("stitcher_start_service").value
        )
        self.stitcher_stop_client = self.create_client(
            Trigger, self.get_parameter("stitcher_stop_service").value
        )

        self.create_subscription(String, '/stitcher/status', self._on_stitcher_status, 10)

        # State
        self.current_position_mm = 0.0
        self.total_points_received = 0
        self._last_msg_stamp = self.get_clock().now()

        self.create_timer(2.0, self._check_connection)

    # ── ROS callbacks ──────────────────────────────────────────────────────

    def _check_connection(self):
        elapsed = (self.get_clock().now() - self._last_msg_stamp).nanoseconds / 1e9
        self._bridge.connected_updated.emit(elapsed < 5.0)

    def _on_position(self, msg):
        self.current_position_mm = msg.data
        self._last_msg_stamp = self.get_clock().now()
        self._bridge.position_updated.emit(msg.data)

    def _on_motor_speed(self, msg):
        self._bridge.rpm_updated.emit(msg.data)

    def _on_pointcloud(self, msg):
        self.total_points_received += msg.width * msg.height
        self._bridge.points_updated.emit(self.total_points_received)

    def _on_image(self, msg):
        frame = self._image_msg_to_bgr(msg)
        if frame is None:
            return
        rgb = np.ascontiguousarray(frame[:, :, ::-1])
        h, w, ch = rgb.shape
        # .copy() keeps a valid buffer after the numpy array is GC'd
        qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888).copy()
        self._bridge.image_updated.emit(qimg)

    def _image_msg_to_bgr(self, msg):
        encoding = msg.encoding.lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)

        if encoding in ("bgr8", "rgb8"):
            channels = 3
            if msg.step < msg.width * channels:
                self.get_logger().error(
                    f"Invalid step for {encoding}: step={msg.step}, width={msg.width}"
                )
                return None
            row_data = data.reshape((msg.height, msg.step))
            img = row_data[:, : msg.width * channels].reshape(
                (msg.height, msg.width, channels)
            )
            if encoding == "rgb8":
                img = img[:, :, ::-1]
            return np.ascontiguousarray(img)

        if encoding == "mono8":
            if msg.step < msg.width:
                self.get_logger().error(
                    f"Invalid step for mono8: step={msg.step}, width={msg.width}"
                )
                return None
            row_data = data.reshape((msg.height, msg.step))
            gray = row_data[:, : msg.width].reshape((msg.height, msg.width))
            return np.ascontiguousarray(np.stack((gray, gray, gray), axis=2))

        self.get_logger().warn(
            f"Unsupported image encoding on {self._image_topic}: {msg.encoding}. "
            "Use bgr8/rgb8/mono8."
        )
        return None

    # ── Commands ───────────────────────────────────────────────────────────

    def send_gantry_command(self, target_mm, speed_mm_s, accel_mm_s2):
        self.position_pub.publish(Float32(data=float(target_mm)))
        self.speed_pub.publish(Float32(data=float(speed_mm_s)))
        self.accel_pub.publish(Float32(data=float(accel_mm_s2)))
        self.gantry_pub.publish(
            Vector3(x=float(target_mm), y=float(speed_mm_s), z=float(accel_mm_s2))
        )
        self.get_logger().info(
            f"Gantry: target={target_mm:.2f} mm, "
            f"speed={speed_mm_s:.2f} mm/s, accel={accel_mm_s2:.2f} mm/s²"
        )

    def set_scanner_parameters(self, angle_min, angle_max, range_max, simulate_encoder):
        try:
            from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
            from rcl_interfaces.srv import SetParameters

            client = self.create_client(
                SetParameters, "/scanner_3d_node/set_parameters"
            )
            if not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error(
                    "scanner_3d_node parameter service not available"
                )
                return
            req = SetParameters.Request()
            entries = [
                ("angle_min", ParameterType.PARAMETER_DOUBLE, float(angle_min)),
                ("angle_max", ParameterType.PARAMETER_DOUBLE, float(angle_max)),
                ("range_max", ParameterType.PARAMETER_DOUBLE, float(range_max)),
                (
                    "simulate_encoder",
                    ParameterType.PARAMETER_BOOL,
                    bool(simulate_encoder),
                ),
            ]
            for name, ptype, val in entries:
                p = Parameter()
                p.name = name
                if ptype == ParameterType.PARAMETER_DOUBLE:
                    p.value = ParameterValue(type=ptype, double_value=val)
                else:
                    p.value = ParameterValue(type=ptype, bool_value=val)
                req.parameters.append(p)
            future = client.call_async(req)
            future.add_done_callback(
                lambda f: self.get_logger().info("Scanner parameters applied")
            )
        except Exception as e:
            self.get_logger().error(f"Failed to set scanner parameters: {e}")

    def _on_stitcher_status(self, msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
            if parsed.get('state') == 'finalizing':
                p = parsed.get('progress', 0)
                t = parsed.get('total', 1)
                self._bridge.stitcher_status_updated.emit(f'Finalizing {p}/{t}', True)
                return
        except (json.JSONDecodeError, TypeError):
            pass
        plain = {
            'idle': ('Idle', True),
            'capturing': ('Capturing', True),
            'error': ('Error', False),
        }
        if msg.data in plain:
            self._bridge.stitcher_status_updated.emit(*plain[msg.data])

    def set_stitcher_capture_spacing(self, spacing_mm: float) -> None:
        if not self._stitcher_enabled:
            return
        try:
            from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
            from rcl_interfaces.srv import SetParameters

            client = self.create_client(SetParameters, '/stitcher_node/set_parameters')
            if not client.wait_for_service(timeout_sec=2.0):
                self._bridge.stitcher_log.emit(
                    '[STITCHER] Parameter service not available — is stitcher_node running?'
                )
                return
            req = SetParameters.Request()
            p = Parameter()
            p.name = 'capture_spacing_mm'
            p.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=float(spacing_mm),
            )
            req.parameters.append(p)
            future = client.call_async(req)
            future.add_done_callback(self._on_stitcher_param_result)
        except Exception as e:
            self._bridge.stitcher_log.emit(f'[STITCHER] Failed to set capture_spacing_mm: {e}')

    def _on_stitcher_param_result(self, future) -> None:
        try:
            result = future.result()
            if result.results and result.results[0].successful:
                self._bridge.stitcher_log.emit('[STITCHER] capture_spacing_mm updated successfully')
            else:
                reason = result.results[0].reason if result.results else 'unknown'
                self._bridge.stitcher_log.emit(f'[STITCHER] capture_spacing_mm rejected: {reason}')
        except Exception as e:
            self._bridge.stitcher_log.emit(f'[STITCHER] Param set callback error: {e}')

    def change_image_topic(self, topic: str):
        """Destroy old image subscription and subscribe to a new topic."""
        if topic == self._image_topic:
            return
        try:
            self.destroy_subscription(self._image_sub)
        except Exception:
            pass
        self._image_topic = topic
        self._image_sub = self.create_subscription(
            Image, self._image_topic, self._on_image, 10
        )
        self.get_logger().info(f"Image topic changed to {topic}")

    def call_service_async(self, client, tag):
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"{tag} service not available")
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda f: self._log_service_result(f, tag))

    def _log_service_result(self, future, tag):
        try:
            r = future.result()
            (self.get_logger().info if r.success else self.get_logger().warn)(
                f"{tag}: {r.message}"
            )
        except Exception as e:
            self.get_logger().error(f"{tag} call failed: {e}")

    def call_stitcher_start(self):
        if not self._stitcher_enabled:
            return
        if not self.stitcher_start_client.wait_for_service(timeout_sec=1.0):
            self._bridge.stitcher_log.emit(
                "[STITCHER] /stitcher/start_session not available — is stitcher_node running?"
            )
            self._bridge.stitcher_status_updated.emit("Not Available", False)
            return
        future = self.stitcher_start_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_stitcher_start_result)

    def call_stitcher_stop(self):
        if not self._stitcher_enabled:
            return
        if not self.stitcher_stop_client.wait_for_service(timeout_sec=1.0):
            self._bridge.stitcher_log.emit(
                "[STITCHER] /stitcher/stop_session not available — is stitcher_node running?"
            )
            self._bridge.stitcher_status_updated.emit("Not Available", False)
            return
        future = self.stitcher_stop_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_stitcher_stop_result)

    def _on_stitcher_start_result(self, future):
        try:
            r = future.result()
            icon = "✓" if r.success else "✗"
            self._bridge.stitcher_log.emit(f"[STITCHER] Start {icon}: {r.message}")
            if r.success:
                self._bridge.stitcher_status_updated.emit("Capturing", True)
            else:
                self._bridge.stitcher_status_updated.emit(f"Error: {r.message}", False)
        except Exception as e:
            self._bridge.stitcher_log.emit(f"[STITCHER] Start call error: {e}")
            self._bridge.stitcher_status_updated.emit("Error", False)

    def _on_stitcher_stop_result(self, future):
        try:
            r = future.result()
            icon = "✓" if r.success else "✗"
            self._bridge.stitcher_log.emit(f"[STITCHER] Stop {icon}: {r.message}")
            if r.success:
                self._bridge.stitcher_status_updated.emit("Finalizing…", True)
            else:
                self._bridge.stitcher_status_updated.emit(f"Error: {r.message}", False)
        except Exception as e:
            self._bridge.stitcher_log.emit(f"[STITCHER] Stop call error: {e}")
            self._bridge.stitcher_status_updated.emit("Error", False)


class ScannerControlWindow(ScannerGUI):
    """Subclasses ScannerGUI (QMainWindow) to add ROS wiring and lifecycle."""

    def __init__(self, node: ScannerROSNode, bridge: _ROSBridge):
        super().__init__()
        self._node = node
        self._rviz_process = None
        self._scan_start_mm = 0.0
        self._scan_end_mm = 1000.0

        self._wire(bridge)

    def _wire(self, bridge: _ROSBridge):
        # Bridge → GUI (automatically queued across threads)
        bridge.position_updated.connect(self.set_position)
        bridge.rpm_updated.connect(self.set_motor_speed)
        bridge.points_updated.connect(self.set_points_published)
        bridge.image_updated.connect(self.set_camera_image)
        bridge.connected_updated.connect(self.set_connected)
        bridge.stitcher_log.connect(self.append_log)
        bridge.stitcher_status_updated.connect(self.set_stitcher_status)

        # GUI signals → ROS actions
        self.start_scan_clicked.connect(self._on_start_scan)
        self.stop_scan_clicked.connect(self._on_stop_scan)
        self.move_command_sent.connect(self._node.send_gantry_command)
        self.home_clicked.connect(
            lambda: self._node.send_gantry_command(-5.0, 40.0, 100.0)
        )
        self.go_to_start_clicked.connect(self._on_go_to_start)
        self.go_to_end_clicked.connect(self._on_go_to_end)
        self.scanner_settings_applied.connect(self._node.set_scanner_parameters)
        self.scanner_settings_refresh.connect(self._on_scanner_refresh)
        self.scan_range_changed.connect(self._on_scan_range_changed)
        self.camera_topic_changed.connect(self._on_camera_topic_changed)
        self.launch_rviz_clicked.connect(self._launch_rviz)
        self.clear_rviz_clicked.connect(self._clear_rviz)
        self.capture_spacing_changed.connect(self._on_capture_spacing_changed)

    # ── Scan service wrappers ──────────────────────────────────────────────

    def _on_start_scan(self):
        def callback(future):
            try:
                r = future.result()
                if r.success:
                    pos = self._node.current_position_mm
                    start = self._scan_start_mm
                    end = self._scan_end_mm
                    target = start if abs(pos - start) >= abs(pos - end) else end
                    self._node.send_gantry_command(target, 50.0, 120.0)
                    self._node.call_stitcher_start()
                    self.set_status("Scanning")
                    self.append_log(f"Scan started — auto traverse to {target:.2f} mm")
                else:
                    self.set_status("Idle")
                    self.append_log(f"[ERROR] Start scan failed: {r.message}")
            except Exception as e:
                self.append_log(f"[ERROR] Start scan error: {e}")

        if not self._node.start_scan_client.wait_for_service(timeout_sec=1.0):
            self.append_log("[WARN] /start_scan service not available")
            return
        self._node.start_scan_client.call_async(Trigger.Request()).add_done_callback(
            callback
        )

    def _on_stop_scan(self):
        def callback(future):
            try:
                r = future.result()
                if r.success:
                    self._node.call_stitcher_stop()
                    self.set_status("Idle")
                    self.append_log(f"Scan stopped: {r.message}")
                else:
                    self.append_log(f"[ERROR] Stop scan failed: {r.message}")
            except Exception as e:
                self.append_log(f"[ERROR] Stop scan error: {e}")

        if not self._node.stop_scan_client.wait_for_service(timeout_sec=1.0):
            self.append_log("[WARN] /stop_scan service not available")
            return
        self._node.stop_scan_client.call_async(Trigger.Request()).add_done_callback(
            callback
        )

    # ── Gantry shortcuts ──────────────────────────────────────────────────

    def _on_go_to_start(self):
        self._node.send_gantry_command(self._scan_start_mm, 50.0, 120.0)

    def _on_go_to_end(self):
        self._node.send_gantry_command(self._scan_end_mm, 50.0, 120.0)

    def _on_scan_range_changed(self, start_mm, end_mm):
        self._scan_start_mm = start_mm
        self._scan_end_mm = end_mm

    # ── Stitcher capture spacing ──────────────────────────────────────────

    def _on_capture_spacing_changed(self, spacing_mm: float):
        self._node.set_stitcher_capture_spacing(spacing_mm)
        self.append_log(f'[STITCHER] capture_spacing_mm → {spacing_mm:.2f} mm')

    # ── Scanner settings refresh ──────────────────────────────────────────

    def _on_scanner_refresh(self):
        self.append_log("[INFO] Parameter refresh from scanner_3d_node not implemented")

    # ── Camera topic ──────────────────────────────────────────────────────

    def _on_camera_topic_changed(self, topic: str):
        self._node.change_image_topic(topic)
        self.append_log(f"[INFO] Image topic changed to {topic}")

    # ── RViz ──────────────────────────────────────────────────────────────

    def _launch_rviz(self):
        if self._rviz_process is not None:
            return
        try:
            from ament_index_python.packages import get_package_share_directory

            pkg_share = get_package_share_directory("medical_scanner_pkg")
            config_file = os.path.join(pkg_share, "config", "scanner_rviz.rviz")
            args = (
                ["rviz2", "-d", config_file]
                if os.path.exists(config_file)
                else ["rviz2"]
            )
            self._rviz_process = subprocess.Popen(args)
            self.append_log("[INFO] RViz launched")
        except Exception as e:
            self.set_rviz_running(False)
            self.append_log(f"[ERROR] Failed to launch RViz: {e}")

    def _clear_rviz(self):
        if self._rviz_process is not None:
            try:
                self._rviz_process.terminate()
                self._rviz_process.wait(timeout=5)
            except Exception:
                pass
            self._rviz_process = None
        self._node.call_service_async(
            self._node.clear_viz_client, "clear_visualization"
        )
        self.append_log("[INFO] RViz display cleared")

    # ── Lifecycle ─────────────────────────────────────────────────────────

    def closeEvent(self, event):
        if self._rviz_process is not None:
            try:
                self._rviz_process.terminate()
                self._rviz_process.wait(timeout=5)
            except Exception:
                pass
        if self._node:
            self._node.destroy_node()
        rclpy.shutdown()
        event.accept()


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    bridge = _ROSBridge()
    node = ScannerROSNode(bridge)
    window = ScannerControlWindow(node, bridge)
    window.show()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
