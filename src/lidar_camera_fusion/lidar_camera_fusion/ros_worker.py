"""
ros_worker.py
=============
rclpy worker for fusion_gui. Owns all ROS 2 entities (subs, pubs, clients,
TF listener). Runs rclpy.spin in a background thread. Communicates with the
Qt GUI exclusively through pyqtSignals on the ROSBridge so the Qt event loop
auto-marshals events from the ROS thread to the GUI thread.

State machine (sweep):
    IDLE → MOVE_TO_START → SCANNING → MOVE_TO_END → STOPPING → IDLE
    Stop button at any state -> STOPPING -> IDLE.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, List, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from rcl_interfaces.msg import Log, ParameterDescriptor, ParameterType
from geometry_msgs.msg import Point
from rcl_interfaces.srv import (
    DescribeParameters,
    GetParameters,
    ListParameters,
    SetParameters,
)
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger
import tf2_ros

try:
    import os
    if os.environ.get("FORCE_HEADLESS") == "1":
        raise ImportError("Forcing headless mode (bypassing PyQt5)")
    from PyQt5.QtCore import QObject, pyqtSignal
    HAS_PYQT = True
except ImportError:
    HAS_PYQT = False
    class QObject:
        def __init__(self, *args, **kwargs) -> None:
            pass

    class pyqtSignal:
        def __init__(self, *types) -> None:
            self.types = types

        def __get__(self, instance, owner):
            if instance is None:
                return self
            name = f"_signal_{id(self)}"
            if not hasattr(instance, name):
                setattr(instance, name, BoundSignal(self.types))
            return getattr(instance, name)

    class BoundSignal:
        def __init__(self, types) -> None:
            self.types = types
            self._callbacks = []

        def connect(self, callback) -> None:
            self._callbacks.append(callback)

        def emit(self, *args) -> None:
            for cb in self._callbacks:
                try:
                    cb(*args)
                except Exception as ex:
                    try:
                        with open('/tmp/tui_exceptions.log', 'a') as f:
                            f.write(f"Exception in callback {cb}: {ex}\n")
                    except Exception:
                        pass



# ── Bridge ──────────────────────────────────────────────────────────────────


class ROSBridge(QObject):
    """All ROS-thread → Qt-thread events flow through these signals."""

    # heartbeats — emitted on every received message (rate-limited internally)
    scan_heartbeat        = pyqtSignal()
    position_heartbeat    = pyqtSignal(float)   # mm
    image_heartbeat       = pyqtSignal()
    is_scanning_changed   = pyqtSignal(bool)

    # service-call results (success, message)
    service_result        = pyqtSignal(str, bool, str)   # service_name, ok, msg

    # rosout passthrough (severity, name, msg)
    rosout                = pyqtSignal(int, str, str)

    # state-machine progress
    sweep_state_changed   = pyqtSignal(str, str)   # state_name, human_msg

    # parameter discovery completed: per-node list of (name, type, value, descriptor_dict)
    params_discovered     = pyqtSignal(str, list)  # node_name, params

    # general log
    log                   = pyqtSignal(int, str)   # level (Log.*), message


# ── Sweep state machine ─────────────────────────────────────────────────────


class SweepState(Enum):
    IDLE          = auto()
    CLEARING      = auto()
    MOVE_TO_START = auto()
    STARTING_SCAN = auto()
    SCANNING      = auto()
    MOVE_TO_END   = auto()
    STOPPING      = auto()


@dataclass
class SweepConfig:
    start_mm:    float
    end_mm:      float
    speed_mm_s:  float
    tolerance_mm: float = 0.5
    arrival_timeout_s: float = 60.0


# ── Heartbeat tracker ───────────────────────────────────────────────────────
# Topic-name → last-received monotonic timestamp. Single writer (ROS thread),
# single reader (Qt thread via QTimer poll). Plain dict + atomic float assign
# is safe in CPython for this access pattern.

@dataclass
class _Heartbeats:
    last: Dict[str, float] = field(default_factory=dict)

    def touch(self, key: str) -> None:
        self.last[key] = time.monotonic()

    def age(self, key: str) -> Optional[float]:
        t = self.last.get(key)
        return None if t is None else time.monotonic() - t


# ── ROS worker node ─────────────────────────────────────────────────────────


class FusionGuiNode(Node):
    """All ROS 2 entities live here. Spins in a background thread."""

    SCAN_TOPIC             = "/scan"
    DEFAULT_POSITION_TOPIC = "/current_position"
    IMAGE_TOPIC            = "/image_raw"
    IS_SCANNING_TOPIC      = "/scanner/is_scanning"
    ASSEMBLED_CLOUD_TOPIC  = "/scanner/assembled_cloud"
    COLORED_CLOUD_TOPIC    = "/scanner/colored_cloud"
    ROSOUT_TOPIC           = "/rosout"

    DEFAULT_TARGET_POSITION_TOPIC = "/target_position"
    SPEED_PUB_TOPIC     = "/speed"

    SVC_START   = "/scanner/start"
    SVC_STOP    = "/scanner/stop"
    SVC_CLEAR   = "/scanner/clear_cloud"

    TF_TARGET   = "base_link"
    TF_SOURCE   = "lidar_link"

    # nodes whose health we monitor in the status panel and param auto-discover
    MONITORED_NODES = (
        "scan_assembler_node",
        "cloud_colorizer_node",
        "scan_image_recorder_node",
        "robot_state_publisher",
        "v4l2_camera_node",
    )

    def __init__(self, bridge: ROSBridge):
        super().__init__("fusion_gui")
        self.bridge = bridge
        self.cb_group = ReentrantCallbackGroup()
        self.heartbeats = _Heartbeats()

        self.declare_parameter("position_topic", self.DEFAULT_POSITION_TOPIC)
        self.declare_parameter("target_position_topic", self.DEFAULT_TARGET_POSITION_TOPIC)
        self.declare_parameter("speed_topic", self.SPEED_PUB_TOPIC)
        self.position_topic = str(self.get_parameter("position_topic").value)
        self.target_position_topic = str(self.get_parameter("target_position_topic").value)
        self.speed_topic = str(self.get_parameter("speed_topic").value)

        # latest known gantry position (mm); None until first message
        self._current_pos_mm: Optional[float] = None
        self._pos_lock = threading.Lock()

        # sweep state-machine
        self._sweep_state = SweepState.IDLE
        self._sweep_cfg: Optional[SweepConfig] = None
        self._sweep_thread: Optional[threading.Thread] = None
        self._sweep_cancel = threading.Event()

        # ── Subscriptions ────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(
            LaserScan, self.SCAN_TOPIC,
            self._on_scan, sensor_qos, callback_group=self.cb_group)
        self.create_subscription(
            Point, self.position_topic,
            self._on_position, 10, callback_group=self.cb_group)
        self.create_subscription(
            Image, self.IMAGE_TOPIC,
            self._on_image, sensor_qos, callback_group=self.cb_group)
        self.create_subscription(
            PointCloud2, self.ASSEMBLED_CLOUD_TOPIC,
            self._on_assembled_cloud, sensor_qos, callback_group=self.cb_group)
        self.create_subscription(
            PointCloud2, self.COLORED_CLOUD_TOPIC,
            self._on_colored_cloud, sensor_qos, callback_group=self.cb_group)

        # transient_local matches scan_assembler's publisher
        latched_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Bool, self.IS_SCANNING_TOPIC,
            self._on_is_scanning, latched_qos, callback_group=self.cb_group)

        # /rosout — rcl_interfaces/Log; reliable, depth 100
        self.create_subscription(
            Log, self.ROSOUT_TOPIC,
            self._on_rosout, 100, callback_group=self.cb_group)

        # ── Publishers ───────────────────────────────────────────────────────
        self.pub_position = self.create_publisher(Point, self.target_position_topic, 10)
        self.pub_speed    = self.create_publisher(Float32, self.speed_topic, 10)

        # ── Service clients ──────────────────────────────────────────────────
        self.cli_start = self.create_client(
            Trigger, self.SVC_START, callback_group=self.cb_group)
        self.cli_stop  = self.create_client(
            Trigger, self.SVC_STOP, callback_group=self.cb_group)
        self.cli_clear = self.create_client(
            Trigger, self.SVC_CLEAR, callback_group=self.cb_group)

        # ── TF listener ──────────────────────────────────────────────────────
        # spin_thread=True so /tf keeps draining while we block on can_transform
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self, spin_thread=True)

        self.get_logger().info("fusion_gui ROS worker initialised")

    # ── Subscription callbacks ──────────────────────────────────────────────

    def _on_scan(self, _msg: LaserScan) -> None:
        self.heartbeats.touch(self.SCAN_TOPIC)
        self.bridge.scan_heartbeat.emit()

    def _on_position(self, msg: Point) -> None:
        # Two-axis gantry reports position as geometry_msgs/Point; scanner axis uses x (mm).
        x_mm = float(msg.x)
        self.heartbeats.touch(self.position_topic)
        with self._pos_lock:
            self._current_pos_mm = x_mm
        self.bridge.position_heartbeat.emit(x_mm)

    def _on_image(self, _msg: Image) -> None:
        self.heartbeats.touch(self.IMAGE_TOPIC)
        self.bridge.image_heartbeat.emit()

    def _on_assembled_cloud(self, _msg: PointCloud2) -> None:
        self.heartbeats.touch(self.ASSEMBLED_CLOUD_TOPIC)

    def _on_colored_cloud(self, _msg: PointCloud2) -> None:
        self.heartbeats.touch(self.COLORED_CLOUD_TOPIC)

    def _on_is_scanning(self, msg: Bool) -> None:
        self.bridge.is_scanning_changed.emit(bool(msg.data))

    def _on_rosout(self, msg: Log) -> None:
        # forward all rosout — the GUI does its own filtering / display
        self.bridge.rosout.emit(int(msg.level), str(msg.name), str(msg.msg))

    # ── Public helpers (called from Qt thread) ──────────────────────────────

    def get_current_position_mm(self) -> Optional[float]:
        with self._pos_lock:
            return self._current_pos_mm

    def heartbeat_age(self, topic: str) -> Optional[float]:
        return self.heartbeats.age(topic)

    def tf_base_to_lidar_ok(self) -> bool:
        return self.tf_buffer.can_transform(
            self.TF_TARGET, self.TF_SOURCE, rclpy.time.Time())

    def list_running_nodes(self) -> List[str]:
        try:
            return [n for n, _ns in self.get_node_names_and_namespaces()]
        except Exception:   # noqa: BLE001
            return []

    # ── Service calls (fire-and-forget; result emitted on bridge) ───────────

    def call_trigger(self, service_name: str) -> None:
        cli = {
            self.SVC_START: self.cli_start,
            self.SVC_STOP:  self.cli_stop,
            self.SVC_CLEAR: self.cli_clear,
        }.get(service_name)
        if cli is None:
            self.bridge.service_result.emit(service_name, False, "unknown service")
            return
        if not cli.service_is_ready():
            self.bridge.service_result.emit(
                service_name, False, "service not available")
            return
        future = cli.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f, n=service_name: self._on_trigger_done(n, f))

    def _on_trigger_done(self, name: str, future) -> None:
        try:
            res = future.result()
            self.bridge.service_result.emit(
                name, bool(res.success), str(res.message))
        except Exception as ex:   # noqa: BLE001
            self.bridge.service_result.emit(name, False, f"exception: {ex}")

    # ── Position publishing ─────────────────────────────────────────────────

    def publish_position(self, mm: float) -> None:
        msg = Point()
        msg.x = float(mm)
        msg.y = 0.0
        msg.z = 0.0
        self.pub_position.publish(msg)

    def publish_speed(self, mm_s: float) -> None:
        msg = Float32()
        msg.data = float(mm_s)
        self.pub_speed.publish(msg)

    # ── Sweep state machine ─────────────────────────────────────────────────

    def sweep_state_name(self) -> str:
        return self._sweep_state.name

    def sweep_busy(self) -> bool:
        return self._sweep_state != SweepState.IDLE

    def start_sweep(self, cfg: SweepConfig) -> bool:
        if self.sweep_busy():
            self.bridge.log.emit(
                Log.WARN, "Sweep already in progress; ignoring Start")
            return False
        self._sweep_cfg = cfg
        self._sweep_cancel.clear()
        self._sweep_thread = threading.Thread(
            target=self._sweep_run, name="sweep_fsm", daemon=True)
        self._sweep_thread.start()
        return True

    def cancel_sweep(self) -> None:
        # signal cancel — even if state is IDLE, this is a no-op cleanly
        self._sweep_cancel.set()
        # also halt gantry in place and stop the scan no matter what
        cur = self.get_current_position_mm()
        if cur is not None:
            self.publish_position(cur)
        self.call_trigger(self.SVC_STOP)
        self._set_state(SweepState.STOPPING, "Stop requested — halting gantry")

    # ── Internal: state-machine implementation ──────────────────────────────

    def _set_state(self, state: SweepState, human: str) -> None:
        self._sweep_state = state
        self.bridge.sweep_state_changed.emit(state.name, human)

    def _wait_arrival(self, target_mm: float, timeout_s: float) -> bool:
        """Poll configured position_topic until within tolerance or timeout/cancel."""
        cfg = self._sweep_cfg
        if cfg is None:
            return False
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._sweep_cancel.is_set():
                return False
            cur = self.get_current_position_mm()
            if cur is not None and abs(cur - target_mm) <= cfg.tolerance_mm:
                return True
            time.sleep(0.05)
        return False

    def _call_trigger_blocking(self, cli, label: str, timeout_s: float = 5.0) -> bool:
        if not cli.wait_for_service(timeout_sec=timeout_s):
            self.bridge.log.emit(Log.ERROR, f"{label}: service not available")
            return False
        future = cli.call_async(Trigger.Request())
        # spin externally is happening in the executor; just poll the future.
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            if future.done():
                break
            time.sleep(0.02)
        if not future.done():
            self.bridge.log.emit(Log.ERROR, f"{label}: timeout")
            return False
        try:
            res = future.result()
            self.bridge.service_result.emit(label, bool(res.success), str(res.message))
            return bool(res.success)
        except Exception as ex:   # noqa: BLE001
            self.bridge.log.emit(Log.ERROR, f"{label}: exception {ex}")
            return False

    def _sweep_run(self) -> None:
        cfg = self._sweep_cfg
        assert cfg is not None
        try:
            # Optional: push speed first
            if cfg.speed_mm_s > 0:
                self.publish_speed(cfg.speed_mm_s)

            # 1. clear buffer
            self._set_state(SweepState.CLEARING, "Clearing previous cloud")
            if not self._call_trigger_blocking(self.cli_clear, self.SVC_CLEAR):
                self._set_state(SweepState.IDLE, "Clear failed; aborting")
                return
            if self._sweep_cancel.is_set():
                self._set_state(SweepState.IDLE, "Cancelled before move-to-start")
                return

            # 2. move to start
            self._set_state(
                SweepState.MOVE_TO_START,
                f"Moving to start ({cfg.start_mm:.1f} mm)")
            self.publish_position(cfg.start_mm)
            if not self._wait_arrival(cfg.start_mm, cfg.arrival_timeout_s):
                if self._sweep_cancel.is_set():
                    self._set_state(SweepState.IDLE, "Cancelled during move-to-start")
                else:
                    self._set_state(
                        SweepState.IDLE,
                        f"Timed out waiting for start ({cfg.start_mm:.1f} mm)")
                return

            # 3. start scan
            self._set_state(SweepState.STARTING_SCAN, "Starting scan")
            if not self._call_trigger_blocking(self.cli_start, self.SVC_START):
                self._set_state(SweepState.IDLE, "Scanner start failed; aborting")
                return
            if self._sweep_cancel.is_set():
                self._call_trigger_blocking(self.cli_stop, self.SVC_STOP)
                self._set_state(SweepState.IDLE, "Cancelled right after start")
                return

            # 4. move to end (scanning is active during this move)
            self._set_state(
                SweepState.SCANNING,
                f"Scanning — sweeping to end ({cfg.end_mm:.1f} mm)")
            self.publish_position(cfg.end_mm)
            arrived = self._wait_arrival(cfg.end_mm, cfg.arrival_timeout_s)

            # 5. stop scan (always, even on timeout/cancel — saves whatever we got)
            self._set_state(SweepState.STOPPING, "Stopping scan and saving")
            self._call_trigger_blocking(self.cli_stop, self.SVC_STOP)

            if arrived:
                self._set_state(SweepState.IDLE, "Sweep complete")
            elif self._sweep_cancel.is_set():
                self._set_state(SweepState.IDLE, "Sweep cancelled")
            else:
                self._set_state(SweepState.IDLE, "Sweep timed out before reaching end")
        except Exception as ex:   # noqa: BLE001
            self.bridge.log.emit(Log.ERROR, f"Sweep FSM crashed: {ex}")
            self._set_state(SweepState.IDLE, f"Crashed: {ex}")

    # ── Parameter discovery ─────────────────────────────────────────────────

    def discover_params(self, node_name: str) -> None:
        """Async — list, describe, then read values for one node.
        Emits bridge.params_discovered when done."""
        threading.Thread(
            target=self._discover_params_thread,
            args=(node_name,),
            daemon=True,
            name=f"discover_{node_name}",
        ).start()

    def _discover_params_thread(self, node_name: str) -> None:
        ns = node_name if node_name.startswith("/") else "/" + node_name
        list_cli = self.create_client(ListParameters, f"{ns}/list_parameters")
        desc_cli = self.create_client(DescribeParameters, f"{ns}/describe_parameters")
        get_cli  = self.create_client(GetParameters, f"{ns}/get_parameters")

        try:
            if not list_cli.wait_for_service(timeout_sec=2.0):
                self.bridge.log.emit(
                    Log.WARN, f"params: {node_name} list_parameters not available")
                self.bridge.params_discovered.emit(node_name, [])
                return

            list_req = ListParameters.Request()
            list_req.depth = 0  # all
            f = list_cli.call_async(list_req)
            self._wait_future(f, 3.0)
            if not f.done():
                self.bridge.params_discovered.emit(node_name, [])
                return
            names = list(f.result().result.names)
            # filter out qos overrides etc — they spam the panel
            names = [
                n for n in names
                if not n.startswith("qos_overrides.")
                and n not in ("use_sim_time",)
            ]
            if not names:
                self.bridge.params_discovered.emit(node_name, [])
                return

            desc_req = DescribeParameters.Request()
            desc_req.names = names
            fd = desc_cli.call_async(desc_req)
            self._wait_future(fd, 3.0)
            descriptors: List[ParameterDescriptor] = (
                list(fd.result().descriptors) if fd.done() else
                [ParameterDescriptor() for _ in names])

            get_req = GetParameters.Request()
            get_req.names = names
            fg = get_cli.call_async(get_req)
            self._wait_future(fg, 3.0)
            values = list(fg.result().values) if fg.done() else []

            params = []
            for name, desc, val in zip(names, descriptors, values):
                py_val = _param_value_to_python(val)
                params.append({
                    "name": name,
                    "type": int(val.type),
                    "value": py_val,
                    "read_only": bool(getattr(desc, "read_only", False)),
                    "description": str(getattr(desc, "description", "") or ""),
                })
            self.bridge.params_discovered.emit(node_name, params)
        except Exception as ex:   # noqa: BLE001
            self.bridge.log.emit(
                Log.ERROR, f"params: discover {node_name} failed: {ex}")
            self.bridge.params_discovered.emit(node_name, [])
        finally:
            self.destroy_client(list_cli)
            self.destroy_client(desc_cli)
            self.destroy_client(get_cli)

    def set_param(self, node_name: str, name: str, py_value) -> None:
        threading.Thread(
            target=self._set_param_thread,
            args=(node_name, name, py_value),
            daemon=True,
            name=f"setparam_{node_name}_{name}",
        ).start()

    def _set_param_thread(self, node_name: str, name: str, py_value) -> None:
        ns = node_name if node_name.startswith("/") else "/" + node_name
        cli = self.create_client(SetParameters, f"{ns}/set_parameters")
        try:
            if not cli.wait_for_service(timeout_sec=2.0):
                self.bridge.log.emit(
                    Log.ERROR, f"set_param: {node_name}/{name} no service")
                return
            param = Parameter(name=name, value=py_value)
            req = SetParameters.Request()
            req.parameters = [param.to_parameter_msg()]
            f = cli.call_async(req)
            self._wait_future(f, 3.0)
            if not f.done():
                self.bridge.log.emit(
                    Log.ERROR, f"set_param {node_name}/{name}: timeout")
                return
            results = list(f.result().results)
            if results and results[0].successful:
                self.bridge.log.emit(
                    Log.INFO, f"set_param {node_name}/{name} = {py_value}")
            else:
                reason = results[0].reason if results else "no result"
                self.bridge.log.emit(
                    Log.ERROR,
                    f"set_param {node_name}/{name} rejected: {reason}")
        except Exception as ex:   # noqa: BLE001
            self.bridge.log.emit(
                Log.ERROR, f"set_param {node_name}/{name} crashed: {ex}")
        finally:
            self.destroy_client(cli)

    @staticmethod
    def _wait_future(future, timeout_s: float) -> None:
        end = time.monotonic() + timeout_s
        while time.monotonic() < end and not future.done():
            time.sleep(0.02)


# ── Helpers ─────────────────────────────────────────────────────────────────

def _param_value_to_python(val):
    """Convert rcl_interfaces/ParameterValue → native Python."""
    t = val.type
    if t == ParameterType.PARAMETER_BOOL:
        return bool(val.bool_value)
    if t == ParameterType.PARAMETER_INTEGER:
        return int(val.integer_value)
    if t == ParameterType.PARAMETER_DOUBLE:
        return float(val.double_value)
    if t == ParameterType.PARAMETER_STRING:
        return str(val.string_value)
    if t == ParameterType.PARAMETER_BYTE_ARRAY:
        return list(val.byte_array_value)
    if t == ParameterType.PARAMETER_BOOL_ARRAY:
        return list(val.bool_array_value)
    if t == ParameterType.PARAMETER_INTEGER_ARRAY:
        return list(val.integer_array_value)
    if t == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return list(val.double_array_value)
    if t == ParameterType.PARAMETER_STRING_ARRAY:
        return list(val.string_array_value)
    return None


# ── Spin thread ─────────────────────────────────────────────────────────────


class WorkerThread(threading.Thread):
    """Owns the executor and the node lifecycle."""

    def __init__(self, bridge: ROSBridge):
        super().__init__(name="rclpy_spin", daemon=True)
        self.bridge = bridge
        self.node: Optional[FusionGuiNode] = None
        self._executor: Optional[MultiThreadedExecutor] = None
        self._ready = threading.Event()

    def run(self) -> None:
        rclpy.init()
        self.node = FusionGuiNode(self.bridge)
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self.node)
        self._ready.set()
        try:
            self._executor.spin()
        finally:
            self._executor.remove_node(self.node)
            self.node.destroy_node()
            rclpy.shutdown()

    def wait_ready(self, timeout: float = 5.0) -> bool:
        return self._ready.wait(timeout)

    def shutdown(self) -> None:
        if self._executor is not None:
            self._executor.shutdown()
