#!/usr/bin/env python3

import os
import sys
import subprocess
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from std_srvs.srv import Trigger

from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtGui import QImage

from medical_scanner_pkg.mission_control_gui import MissionControlGUI


class _ROSBridge(QObject):
    """Thread-safe bridge: emits Qt signals from ROS callbacks into the GUI main thread."""
    position_updated  = pyqtSignal(float)
    rpm_updated       = pyqtSignal(float)
    points_updated    = pyqtSignal(int)
    image_updated     = pyqtSignal(QImage)
    connected_updated = pyqtSignal(bool)


class ScannerROSNode(Node):
    def __init__(self, bridge: _ROSBridge):
        super().__init__('scanner_gui_ros_node')
        self._bridge = bridge

        # Publishers
        self.position_pub = self.create_publisher(Float32, '/position', 10)
        self.speed_pub    = self.create_publisher(Float32, '/speed', 10)
        self.accel_pub    = self.create_publisher(Float32, '/acceleration', 10)
        self.gantry_pub   = self.create_publisher(Vector3, '/scanner/cmd', 10)

        # Subscribers
        self.create_subscription(Float32,      '/current_position',    self._on_position,    10)
        self.create_subscription(Float32,      '/motor_speed',         self._on_motor_speed, 10)
        self.create_subscription(PointCloud2,  '/scanner/pointcloud',  self._on_pointcloud,  10)

        self.declare_parameter('image_topic', '/camera/image_raw')
        self._image_topic = self.get_parameter('image_topic').value
        self._image_sub = self.create_subscription(
            Image, self._image_topic, self._on_image, 10)

        # Stitcher integration (optional)
        self.declare_parameter('enable_stitcher_integration', False)
        self.declare_parameter('stitcher_start_service', '/stitcher/start_session')
        self.declare_parameter('stitcher_stop_service',  '/stitcher/stop_session')
        self._stitcher_enabled = bool(self.get_parameter('enable_stitcher_integration').value)

        # Service clients
        self.start_scan_client    = self.create_client(Trigger, '/start_scan')
        self.stop_scan_client     = self.create_client(Trigger, '/stop_scan')
        self.clear_viz_client     = self.create_client(Trigger, '/clear_visualization')
        self.stitcher_start_client = self.create_client(
            Trigger, self.get_parameter('stitcher_start_service').value)
        self.stitcher_stop_client  = self.create_client(
            Trigger, self.get_parameter('stitcher_stop_service').value)

        # State
        self.current_position_mm   = 0.0
        self.total_points_received = 0
        self._last_msg_stamp       = self.get_clock().now()

        # Periodic connection health check
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

        if encoding in ('bgr8', 'rgb8'):
            channels = 3
            if msg.step < msg.width * channels:
                self.get_logger().error(
                    f'Invalid step for {encoding}: step={msg.step}, width={msg.width}')
                return None
            row_data = data.reshape((msg.height, msg.step))
            img = row_data[:, :msg.width * channels].reshape(
                (msg.height, msg.width, channels))
            if encoding == 'rgb8':
                img = img[:, :, ::-1]
            return np.ascontiguousarray(img)

        if encoding == 'mono8':
            if msg.step < msg.width:
                self.get_logger().error(
                    f'Invalid step for mono8: step={msg.step}, width={msg.width}')
                return None
            row_data = data.reshape((msg.height, msg.step))
            gray = row_data[:, :msg.width].reshape((msg.height, msg.width))
            return np.ascontiguousarray(np.stack((gray, gray, gray), axis=2))

        self.get_logger().warn(
            f'Unsupported image encoding on {self._image_topic}: {msg.encoding}. '
            'Use bgr8/rgb8/mono8.')
        return None

    # ── Commands ───────────────────────────────────────────────────────────

    def send_gantry_command(self, target_mm, speed_mm_s, accel_mm_s2):
        self.position_pub.publish(Float32(data=float(target_mm)))
        self.speed_pub.publish(Float32(data=float(speed_mm_s)))
        self.accel_pub.publish(Float32(data=float(accel_mm_s2)))
        self.gantry_pub.publish(
            Vector3(x=float(target_mm), y=float(speed_mm_s), z=float(accel_mm_s2)))
        self.get_logger().info(
            f'Gantry: target={target_mm:.2f} mm, '
            f'speed={speed_mm_s:.2f} mm/s, accel={accel_mm_s2:.2f} mm/s²')

    def set_scanner_parameters(self, angle_min, angle_max, range_max, simulate_encoder):
        try:
            from rcl_interfaces.srv import SetParameters
            from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
            client = self.create_client(SetParameters, '/scanner_3d_node/set_parameters')
            if not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error('scanner_3d_node parameter service not available')
                return
            req = SetParameters.Request()
            entries = [
                ('angle_min',        ParameterType.PARAMETER_DOUBLE, float(angle_min)),
                ('angle_max',        ParameterType.PARAMETER_DOUBLE, float(angle_max)),
                ('range_max',        ParameterType.PARAMETER_DOUBLE, float(range_max)),
                ('simulate_encoder', ParameterType.PARAMETER_BOOL,   bool(simulate_encoder)),
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
                lambda f: self.get_logger().info('Scanner parameters applied'))
        except Exception as e:
            self.get_logger().error(f'Failed to set scanner parameters: {e}')

    def call_service_async(self, client, tag):
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{tag} service not available')
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda f: self._log_service_result(f, tag))

    def _log_service_result(self, future, tag):
        try:
            r = future.result()
            (self.get_logger().info if r.success else self.get_logger().warn)(
                f'{tag}: {r.message}')
        except Exception as e:
            self.get_logger().error(f'{tag} call failed: {e}')

    def call_stitcher_start(self):
        if self._stitcher_enabled:
            self.call_service_async(self.stitcher_start_client, 'stitcher_start')

    def call_stitcher_stop(self):
        if self._stitcher_enabled:
            self.call_service_async(self.stitcher_stop_client, 'stitcher_stop')


class MissionControlWindow(QMainWindow):
    def __init__(self, node: ScannerROSNode, bridge: _ROSBridge):
        super().__init__()
        self._node = node
        self._rviz_process = None
        self._scan_start_mm = 0.0
        self._scan_end_mm   = 1000.0

        self._gui = MissionControlGUI(theme='dark', density='comfortable', demo_mode=False)
        self.setCentralWidget(self._gui)
        self.setWindowTitle('Scanner Control · Mission Control')
        self.resize(1440, 900)

        self._wire(bridge)

    def _wire(self, bridge: _ROSBridge):
        g = self._gui

        # Bridge → GUI (automatically queued across threads)
        bridge.position_updated.connect(g.set_position)
        bridge.rpm_updated.connect(g.set_rpm)
        bridge.points_updated.connect(g.set_points)
        bridge.image_updated.connect(g.set_camera_image)
        bridge.connected_updated.connect(g.set_connected)

        # GUI signals → ROS actions
        g.start_scan_requested.connect(self._on_start_scan)
        g.stop_scan_requested.connect(self._on_stop_scan)
        g.estop_requested.connect(self._on_estop)
        g.move_requested.connect(self._node.send_gantry_command)
        g.home_requested.connect(lambda: self._node.send_gantry_command(-5.0, 40.0, 100.0))
        g.lidar_settings_changed.connect(self._node.set_scanner_parameters)
        g.scan_settings_changed.connect(self._on_scan_settings_changed)
        g.rviz_launch_requested.connect(self._launch_rviz)
        g.rviz_clear_requested.connect(self._clear_rviz)
        g.preset_loaded.connect(self._on_preset_loaded)

    # ── Scan service wrappers ──────────────────────────────────────────────

    def _on_start_scan(self):
        def callback(future):
            try:
                r = future.result()
                if r.success:
                    pos   = self._node.current_position_mm
                    start = self._scan_start_mm
                    end   = self._scan_end_mm
                    target = start if abs(pos - start) >= abs(pos - end) else end
                    self._node.send_gantry_command(target, 50.0, 120.0)
                    self._node.call_stitcher_start()
                else:
                    self._gui.append_log('err', f'Start scan failed: {r.message}')
            except Exception as e:
                self._gui.append_log('err', f'Start scan error: {e}')

        if not self._node.start_scan_client.wait_for_service(timeout_sec=1.0):
            self._gui.append_log('warn', '/start_scan service not available')
            return
        self._node.start_scan_client.call_async(Trigger.Request()).add_done_callback(callback)

    def _on_stop_scan(self):
        def callback(future):
            try:
                r = future.result()
                if r.success:
                    self._node.call_stitcher_stop()
                else:
                    self._gui.append_log('err', f'Stop scan failed: {r.message}')
            except Exception as e:
                self._gui.append_log('err', f'Stop scan error: {e}')

        if not self._node.stop_scan_client.wait_for_service(timeout_sec=1.0):
            self._gui.append_log('warn', '/stop_scan service not available')
            return
        self._node.stop_scan_client.call_async(Trigger.Request()).add_done_callback(callback)

    def _on_estop(self):
        self._node.send_gantry_command(self._node.current_position_mm, 0.0, 500.0)
        self._node.get_logger().error('E-STOP activated — gantry halt sent')

    def _on_scan_settings_changed(self, start_mm, end_mm):
        self._scan_start_mm = start_mm
        self._scan_end_mm   = end_mm

    # ── RViz ──────────────────────────────────────────────────────────────

    def _launch_rviz(self):
        if self._rviz_process is not None:
            return
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('medical_scanner_pkg')
            config_file = os.path.join(pkg_share, 'config', 'scanner_rviz.rviz')
            args = ['rviz2', '-d', config_file] if os.path.exists(config_file) else ['rviz2']
            self._rviz_process = subprocess.Popen(args)
            self._gui.rviz_status_lbl.setText('RVIZ RUNNING')
        except Exception as e:
            self._gui.append_log('err', f'Failed to launch RViz: {e}')

    def _clear_rviz(self):
        if self._rviz_process is not None:
            try:
                self._rviz_process.terminate()
                self._rviz_process.wait(timeout=5)
            except Exception:
                pass
            self._rviz_process = None
        self._node.call_service_async(self._node.clear_viz_client, 'clear_visualization')
        self._gui.rviz_status_lbl.setText('RVIZ NOT RUNNING — START A SCAN')

    # ── Preset ────────────────────────────────────────────────────────────

    def _on_preset_loaded(self, p):
        self._node.set_scanner_parameters(
            p['angle_min'], p['angle_max'], p['range_max'], False)

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
    app.setStyle('Fusion')

    bridge = _ROSBridge()
    node   = ScannerROSNode(bridge)
    window = MissionControlWindow(node, bridge)
    window.show()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
