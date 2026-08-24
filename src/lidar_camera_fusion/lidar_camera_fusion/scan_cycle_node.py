"""
scan_cycle_node.py
==================
Headless orchestrator that sequences a full scan sweep:

    1. Move gantry to start position
    2. Wait for arrival
    3. Call /scanner/start  (clears buffer, begins accumulation)
    4. Move gantry to end position
    5. Wait for arrival
    6. Call /scanner/stop   (stops accumulation, saves .pcd)

Services (designed for Foxglove Call Service panels):
    /scanner/start_cycle  (std_srvs/Trigger) — run the sweep
    /scanner/interrupt    (std_srvs/Trigger) — cancel in-progress cycle

Subscribes:  configurable position_topic (geometry_msgs/Point, x mm)
Publishes:   configurable target_position_topic (geometry_msgs/Point, x mm)
             /speed             (std_msgs/Float32, mm/s)

Parameters (set in launch file or via ros2 param set):
    start_mm      (float, default 0.0)   — scan start position
    end_mm        (float, default 300.0) — scan end position
    speed_mm_s    (float, default 20.0)  — gantry speed (0 = don't set)
    tolerance_mm  (float, default 0.5)   — arrival tolerance
    timeout_s     (float, default 60.0)  — per-segment arrival timeout
"""
from __future__ import annotations

import threading
import time
from typing import Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from std_srvs.srv import Trigger


class ScanCycleNode(Node):
    """Orchestrates position → start → sweep → stop for a single scan pass."""

    def __init__(self) -> None:
        super().__init__("scan_cycle_node")

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter("start_mm", 0.0)
        self.declare_parameter("end_mm", 300.0)
        self.declare_parameter("speed_mm_s", 20.0)
        self.declare_parameter("tolerance_mm", 0.5)
        self.declare_parameter("timeout_s", 60.0)
        self.declare_parameter("position_topic", "/current_position")
        self.declare_parameter("target_position_topic", "/target_position")
        self.declare_parameter("speed_topic", "/speed")

        # Read once at construction — use ros2 param set to change between cycles
        self._position_topic = str(self.get_parameter("position_topic").value)
        self._target_position_topic = str(self.get_parameter("target_position_topic").value)
        self._speed_topic = str(self.get_parameter("speed_topic").value)
        self._start_mm = float(self.get_parameter("start_mm").value)
        self._end_mm = float(self.get_parameter("end_mm").value)
        self._speed_mm_s = float(self.get_parameter("speed_mm_s").value)
        self._tolerance_mm = float(self.get_parameter("tolerance_mm").value)
        self._timeout_s = float(self.get_parameter("timeout_s").value)

        # ── Cached gantry position (updated by subscription) ─────────────
        self._current_pos_mm: Optional[float] = None
        self._pos_lock = threading.Lock()

        # ── Sweep state ──────────────────────────────────────────────────
        self._sweep_thread: Optional[threading.Thread] = None
        self._cancel = threading.Event()

        # ── Callback group (reentrant so services and subs coexist) ──────
        self._cb_group = ReentrantCallbackGroup()

        # ── Subscription: gantry position ─────────────────────────────────
        self._pos_sub = self.create_subscription(
            Point,
            self._position_topic,
            self._on_position,
            10,
            callback_group=self._cb_group,
        )

        # ── Publishers ───────────────────────────────────────────────────
        self._pub_position = self.create_publisher(Point, self._target_position_topic, 10)
        self._pub_speed = self.create_publisher(Float32, self._speed_topic, 10)

        # ── Service clients (scanner) ────────────────────────────────────
        self._cli_start = self.create_client(
            Trigger, "/scanner/start", callback_group=self._cb_group
        )
        self._cli_stop = self.create_client(
            Trigger, "/scanner/stop", callback_group=self._cb_group
        )

        # ── Service servers (exposed to Foxglove) ────────────────────────
        self._srv_start_cycle = self.create_service(
            Trigger,
            "/scanner/start_cycle",
            self._on_start_cycle,
            callback_group=self._cb_group,
        )
        self._srv_interrupt = self.create_service(
            Trigger,
            "/scanner/interrupt",
            self._on_interrupt,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            f"scan_cycle_node ready: start={self._start_mm:.1f} mm, "
            f"end={self._end_mm:.1f} mm, speed={self._speed_mm_s:.1f} mm/s, "
            f"tolerance={self._tolerance_mm:.1f} mm, timeout={self._timeout_s:.1f} s"
        )

    # ── Subscription callback ────────────────────────────────────────────────

    def _on_position(self, msg: Point) -> None:
        with self._pos_lock:
            self._current_pos_mm = float(msg.x)

    # ── Public helpers ───────────────────────────────────────────────────────

    def _get_position_mm(self) -> Optional[float]:
        with self._pos_lock:
            return self._current_pos_mm

    def _busy(self) -> bool:
        t = self._sweep_thread
        return t is not None and t.is_alive()

    # ── Refresh parameters (call before each cycle if using ros2 param set) ──

    def _refresh_params(self) -> None:
        self._start_mm = float(self.get_parameter("start_mm").value)
        self._end_mm = float(self.get_parameter("end_mm").value)
        self._speed_mm_s = float(self.get_parameter("speed_mm_s").value)
        self._tolerance_mm = float(self.get_parameter("tolerance_mm").value)
        self._timeout_s = float(self.get_parameter("timeout_s").value)

    # ── Service callbacks ────────────────────────────────────────────────────

    def _on_start_cycle(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if self._busy():
            response.success = False
            response.message = (
                "Scan cycle already in progress — call /scanner/interrupt first"
            )
            return response

        # Re-read params so ros2 param set between cycles takes effect
        self._refresh_params()

        if abs(self._end_mm - self._start_mm) < 0.1:
            response.success = False
            response.message = (
                f"start_mm ({self._start_mm:.1f}) and end_mm ({self._end_mm:.1f}) "
                "are too close — nothing to sweep"
            )
            return response

        self._cancel.clear()
        self._sweep_thread = threading.Thread(
            target=self._sweep_run, name="sweep_fsm", daemon=True
        )
        self._sweep_thread.start()

        response.success = True
        response.message = (
            f"Scan cycle started: {self._start_mm:.1f} → {self._end_mm:.1f} mm"
        )
        self.get_logger().info(response.message)
        return response

    def _on_interrupt(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if not self._busy():
            response.success = True
            response.message = "No scan cycle in progress — nothing to interrupt"
            return response

        self._cancel.set()
        # Halt the gantry immediately at its current position
        cur = self._get_position_mm()
        if cur is not None:
            msg = Point()
            msg.x = float(cur)
            msg.y = 0.0
            msg.z = 0.0
            self._pub_position.publish(msg)

        # Stop the scanner so whatever was accumulated gets saved
        self._call_trigger_async(self._cli_stop, "/scanner/stop")

        response.success = True
        response.message = "Interrupt signaled — gantry halted, scanning stopped"
        self.get_logger().warn(response.message)
        return response

    # ── Sweep state machine (runs in daemon thread) ──────────────────────────

    def _sweep_run(self) -> None:
        try:
            # 0. Optionally set speed
            if self._speed_mm_s > 0:
                msg = Float32()
                msg.data = self._speed_mm_s
                self._pub_speed.publish(msg)
                self.get_logger().info(
                    f"Gantry speed set to {self._speed_mm_s:.1f} mm/s"
                )

            # 1. Move to start
            self.get_logger().info(
                f"Moving to start: {self._start_mm:.1f} mm"
            )
            msg = Point()
            msg.x = float(self._start_mm)
            msg.y = 0.0
            msg.z = 0.0
            self._pub_position.publish(msg)

            if not self._wait_arrival(self._start_mm):
                if self._cancel.is_set():
                    self.get_logger().info("Cancelled during move-to-start")
                else:
                    self.get_logger().error(
                        f"Timed out waiting for start position {self._start_mm:.1f} mm"
                    )
                return

            # 2. Start scan
            self.get_logger().info("Starting scanner accumulation")
            if not self._call_trigger_blocking(self._cli_start, "/scanner/start"):
                self.get_logger().error("Failed to start scanner")
                return
            if self._cancel.is_set():
                self._call_trigger_blocking(self._cli_stop, "/scanner/stop")
                self.get_logger().info("Cancelled right after scan start")
                return

            # 3. Sweep to end (scanning is active during this move)
            self.get_logger().info(
                f"Scanning — sweeping to end: {self._end_mm:.1f} mm"
            )
            msg.data = self._end_mm
            self._pub_position.publish(msg)

            arrived = self._wait_arrival(self._end_mm)

            # 4. Stop scan (always — saves whatever was accumulated)
            self.get_logger().info("Stopping scanner")
            self._call_trigger_blocking(self._cli_stop, "/scanner/stop")

            if arrived:
                self.get_logger().info("Scan cycle complete — PCD saved")
            elif self._cancel.is_set():
                self.get_logger().info("Scan cycle cancelled — partial PCD saved")
            else:
                self.get_logger().error("Scan cycle timed out — partial PCD saved")

        except Exception as exc:
            self.get_logger().error(f"Sweep FSM crashed: {exc}")
            # Best-effort: stop the scanner
            try:
                self._call_trigger_blocking(self._cli_stop, "/scanner/stop")
            except Exception:
                pass

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _wait_arrival(self, target_mm: float) -> bool:
        """Poll configured position_topic until within tolerance, timeout, or cancel."""
        deadline = time.monotonic() + self._timeout_s
        while time.monotonic() < deadline:
            if self._cancel.is_set():
                return False
            cur = self._get_position_mm()
            if cur is not None and abs(cur - target_mm) <= self._tolerance_mm:
                return True
            time.sleep(0.05)
        return False

    def _call_trigger_blocking(
        self,
        cli,
        label: str,
        timeout_s: float = 5.0,
    ) -> bool:
        """Blocking Trigger service call (called from sweep thread, not executor)."""
        if not cli.wait_for_service(timeout_sec=timeout_s):
            self.get_logger().error(f"{label}: service not available")
            return False

        future = cli.call_async(Trigger.Request())
        # Spin the future manually since we're in a non-executor thread
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if future.done():
                break
            time.sleep(0.02)

        if not future.done():
            self.get_logger().error(f"{label}: timeout")
            return False

        try:
            result = future.result()
            if result.success:
                self.get_logger().info(f"{label}: ok — {result.message}")
            else:
                self.get_logger().error(f"{label}: failed — {result.message}")
            return result.success
        except Exception as exc:
            self.get_logger().error(f"{label}: exception {exc}")
            return False

    def _call_trigger_async(self, cli, label: str) -> None:
        """Fire-and-forget Trigger call (used from interrupt callback in executor)."""
        if not cli.service_is_ready():
            self.get_logger().error(f"{label}: service not available for interrupt")
            return
        future = cli.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f, n=label: self._on_async_trigger_done(n, f)
        )

    def _on_async_trigger_done(self, name: str, future) -> None:
        try:
            res = future.result()
            if res.success:
                self.get_logger().info(f"{name}: ok — {res.message}")
            else:
                self.get_logger().error(f"{name}: failed — {res.message}")
        except Exception as exc:
            self.get_logger().error(f"{name}: exception {exc}")


# ── Entry point ──────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ScanCycleNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
