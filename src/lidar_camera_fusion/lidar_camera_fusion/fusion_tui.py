"""
fusion_tui.py
=============
Headless curses TUI for the lidar_camera_fusion package.

Enables full gantry sweep control, node health monitoring, and log viewing
over SSH sessions on headless systems (e.g. Raspberry Pi) without PyQt5.
"""
from __future__ import annotations

import os
# Force headless mode to bypass PyQt5 signals (which require a running Qt Event Loop)
os.environ["FORCE_HEADLESS"] = "1"

import curses
import datetime as dt
import sys
import time
import threading
from typing import List, Optional

from rcl_interfaces.msg import Log

from lidar_camera_fusion.ros_worker import (
    ROSBridge,
    SweepConfig,
    WorkerThread,
)

class FusionTUI:
    FRESH_S = 2.0
    STALE_S = 5.0

    def __init__(self, bridge: ROSBridge, worker: WorkerThread) -> None:
        self.bridge = bridge
        self.worker = worker
        self.node = worker.node

        self._is_scanning = False
        self._latest_pos_mm: Optional[float] = None
        self._sweep_state = "IDLE"
        self._sweep_human = "Ready"

        # Default/Current Gantry Sweep Parameters
        self.start_mm = 0.0
        self.end_mm = 100.0
        self.speed_mm_s = 20.0
        self.tolerance_mm = 0.5
        self.arrival_timeout_s = 60.0

        # Thread-safe log buffer (store last 50 messages)
        self._logs: List[tuple[int, str, str, str]] = []  # (level, name, msg, time_str)
        self._log_lock = threading.Lock()

        # Wire signals from ROS Bridge
        self.bridge.is_scanning_changed.connect(self._on_is_scanning)
        self.bridge.position_heartbeat.connect(self._on_position)
        self.bridge.service_result.connect(self._on_service_result)
        self.bridge.sweep_state_changed.connect(self._on_sweep_state)
        self.bridge.rosout.connect(self._on_rosout)
        self.bridge.log.connect(self._on_internal_log)

    def _on_is_scanning(self, scanning: bool) -> None:
        self._is_scanning = scanning

    def _on_position(self, mm: float) -> None:
        self._latest_pos_mm = mm

    def _on_service_result(self, name: str, ok: bool, msg: str) -> None:
        level = Log.INFO if ok else Log.ERROR
        self._append_log(level, "ROSBridge", f"{name}: {msg}")

    def _on_sweep_state(self, state_name: str, human: str) -> None:
        self._sweep_state = state_name
        self._sweep_human = human
        self._append_log(Log.INFO, "FSM", f"[{state_name}] {human}")

    def _on_rosout(self, level: int, name: str, msg: str) -> None:
        self._append_log(level, name, msg)

    def _on_internal_log(self, level: int, msg: str) -> None:
        self._append_log(level, "Internal", msg)

    def _append_log(self, level: int, name: str, msg: str) -> None:
        ts = dt.datetime.now().strftime("%H:%M:%S")
        with self._log_lock:
            self._logs.append((level, name, msg, ts))
            if len(self._logs) > 100:
                self._logs.pop(0)

    def _classify(self, age: Optional[float]) -> str:
        if age is None:
            return "DEAD"
        if age < self.FRESH_S:
            return "OK"
        if age < self.STALE_S:
            return "STALE"
        return "DEAD"

    def draw_status(self, win) -> None:
        win.erase()
        win.box()
        win.addstr(0, 2, " Headless Node Status (Heartbeat / TF) ", curses.A_BOLD)

        if not self.node:
            win.addstr(2, 2, "ROS node not initialized")
            return

        # Fetch status values
        pos_age = self.node.heartbeat_age(self.node.position_topic)
        scan_age = self.node.heartbeat_age(self.node.SCAN_TOPIC)
        img_age = self.node.heartbeat_age(self.node.IMAGE_TOPIC)

        try:
            tf_ok = self.node.tf_base_to_lidar_ok()
        except Exception:
            tf_ok = False

        running = set(self.node.list_running_nodes())

        # Render rows
        def add_row(r, label, status, detail, color_pair):
            win.addstr(r, 2, f"{label:26s} : [")
            win.addstr(r, 31, f"{status:5s}", color_pair)
            win.addstr(r, 36, f"] {detail}")

        # Colors for status
        c_green = curses.color_pair(2) | curses.A_BOLD
        c_yellow = curses.color_pair(3) | curses.A_BOLD
        c_red = curses.color_pair(4) | curses.A_BOLD

        def get_pair(status):
            return {"OK": c_green, "STALE": c_yellow, "DEAD": c_red}.get(status, c_red)

        # 1. micro_ros_agent
        st = self._classify(pos_age)
        detail = f"{pos_age:.1f}s ago" if pos_age is not None else "no messages"
        add_row(1, "micro_ros_agent (Gantry)", st, detail, get_pair(st))

        # 2. rplidar_ros
        st = self._classify(scan_age)
        detail = f"{scan_age:.1f}s ago" if scan_age is not None else "no messages"
        add_row(2, "rplidar_ros (/scan)", st, detail, get_pair(st))

        # 3. robot_state_publisher
        st = "OK" if tf_ok else "DEAD"
        detail = "base_link -> lidar_link OK" if tf_ok else "TF not available"
        add_row(3, "robot_state_publisher", st, detail, get_pair(st))

        # 4. v4l2_camera
        st = self._classify(img_age)
        detail = f"{img_age:.1f}s ago" if img_age is not None else "no messages"
        add_row(4, "v4l2_camera (/image_raw)", st, detail, get_pair(st))

        # 5. scan_assembler_node
        has_assembler = "scan_assembler_node" in running
        st = "OK" if has_assembler else "DEAD"
        if has_assembler:
            cloud_age = self.node.heartbeat_age(self.node.ASSEMBLED_CLOUD_TOPIC)
            detail = f"running — cloud: {cloud_age:.1f}s ago" if cloud_age is not None else "running — no msgs"
        else:
            detail = "not running"
        add_row(5, "scan_assembler_node", st, detail, get_pair(st))

        # 6. cloud_colorizer_node
        has_colorizer = "cloud_colorizer_node" in running
        st = "OK" if has_colorizer else "DEAD"
        if has_colorizer:
            col_age = self.node.heartbeat_age(self.node.COLORED_CLOUD_TOPIC)
            detail = f"running — colored: {col_age:.1f}s" if col_age is not None else "running — no msgs"
        else:
            detail = "not running"
        add_row(6, "cloud_colorizer_node", st, detail, get_pair(st))

        # 7. scan_image_recorder_node
        has_recorder = "scan_image_recorder_node" in running
        st = "OK" if has_recorder else "DEAD"
        detail = "running" if has_recorder else "not running"
        add_row(7, "scan_image_recorder_node", st, detail, get_pair(st))

        win.noutrefresh()

    def draw_control(self, win) -> None:
        win.erase()
        win.box()
        win.addstr(0, 2, " Gantry Control ", curses.A_BOLD)

        # Render current position
        pos_str = f"{self._latest_pos_mm:8.2f} mm" if self._latest_pos_mm is not None else "— mm"
        win.addstr(2, 4, f"Current Position : ", curses.A_BOLD)
        win.addstr(2, 23, pos_str, curses.color_pair(5) | curses.A_BOLD)

        # Render sweep state
        state_color = curses.color_pair(2) if self._sweep_state == "IDLE" else curses.color_pair(3)
        win.addstr(3, 4, f"Sweep FSM State  : ", curses.A_BOLD)
        win.addstr(3, 23, f"{self._sweep_state} ({self._sweep_human})", state_color | curses.A_BOLD)

        # Scanning status
        scan_color = curses.color_pair(2) if self._is_scanning else curses.color_pair(1)
        win.addstr(4, 4, f"Scanner Status   : ", curses.A_BOLD)
        win.addstr(4, 23, "ACTIVE SCANNING" if self._is_scanning else "INACTIVE", scan_color | curses.A_BOLD)

        # Active settings display
        cfg_str = f"Start: {self.start_mm:.1f} mm | End: {self.end_mm:.1f} mm | Speed: {self.speed_mm_s:.1f} mm/s | Tol: {self.tolerance_mm:.2f} mm | Timeout: {self.arrival_timeout_s:.0f} s"
        win.addstr(5, 4, f"Active Settings  : {cfg_str}", curses.color_pair(6))

        # Keybind instructions
        win.addstr(7, 4, "[S] Start Sweep (Instant)    [P] Edit Parameters    [C] Halt/Cancel Sweep", curses.A_BOLD)
        win.addstr(8, 4, "[X] Clear Cloud              [Q] Quit Controller", curses.A_BOLD)

        win.noutrefresh()

    def draw_logs(self, win, width, height) -> None:
        win.erase()
        win.box()
        win.addstr(0, 2, " System Logs (/rosout) ", curses.A_BOLD)

        max_lines = height - 2
        with self._log_lock:
            # Take only the last fits
            visible_logs = self._logs[-max_lines:]

        for i, (level, name, msg, ts) in enumerate(visible_logs):
            color = curses.color_pair(1)
            lvl_name = "INFO"
            if level == Log.DEBUG:
                color = curses.color_pair(6)
                lvl_name = "DBUG"
            elif level == Log.WARN:
                color = curses.color_pair(3)
                lvl_name = "WARN"
            elif level == Log.ERROR:
                color = curses.color_pair(4) | curses.A_BOLD
                lvl_name = "ERRO"
            elif level == Log.FATAL:
                color = curses.color_pair(4) | curses.A_BOLD | curses.A_BLINK
                lvl_name = "FATL"

            prefix = f"[{ts}] {lvl_name} {name}: "
            max_msg_len = width - len(prefix) - 5
            truncated_msg = msg[:max_msg_len] + "..." if len(msg) > max_msg_len else msg
            
            line_str = f" {prefix}{truncated_msg}"
            # Protect line output size limit
            try:
                win.addstr(i + 1, 1, line_str[:width-2], color)
            except curses.error:
                pass

        win.noutrefresh()

    def get_input_params(self, stdscr, height, width) -> bool:
        """Draw a modal parameter entry box at the bottom.
        Updates self attributes in place. Returns True if parameters updated successfully."""
        # Save screen before draw
        curses.echo()
        curses.curs_set(1)

        input_win = curses.newwin(9, 60, (height - 10) // 2, (width - 60) // 2)
        input_win.box()
        input_win.addstr(0, 2, " Enter Sweep Parameters ", curses.A_BOLD)
        
        def read_val(r, prompt, default):
            input_win.addstr(r, 2, f"{prompt} [{default}]: ")
            input_win.refresh()
            val_str = input_win.getstr().decode('utf-8').strip()
            if not val_str:
                return default
            try:
                return float(val_str)
            except ValueError:
                return default

        start = read_val(2, "Start Position (mm)", self.start_mm)
        end = read_val(3, "End Position (mm)", self.end_mm)
        speed = read_val(4, "Speed (mm/s)", self.speed_mm_s)
        tol = read_val(5, "Arrival Tolerance (mm)", self.tolerance_mm)
        timeout = read_val(6, "Move Timeout (s)", self.arrival_timeout_s)

        curses.noecho()
        curses.curs_set(0)

        # Validate
        if abs(end - start) < tol:
            self._append_log(Log.WARN, "TUI", "Start and End positions within tolerance; parameters not saved.")
            return False

        self.start_mm = start
        self.end_mm = end
        self.speed_mm_s = speed
        self.tolerance_mm = tol
        self.arrival_timeout_s = timeout
        return True

    def run(self, stdscr) -> None:
        # Initialise color schemes
        curses.start_color()
        curses.use_default_colors()
        # Pair 1: Normal
        curses.init_pair(1, -1, -1)
        # Pair 2: Green
        curses.init_pair(2, curses.COLOR_GREEN, -1)
        # Pair 3: Yellow
        curses.init_pair(3, curses.COLOR_YELLOW, -1)
        # Pair 4: Red
        curses.init_pair(4, curses.COLOR_RED, -1)
        # Pair 5: Cyan / Gantry mm
        curses.init_pair(5, curses.COLOR_CYAN, -1)
        # Pair 6: Grey/Dim
        curses.init_pair(6, 8 if curses.COLORS >= 16 else curses.COLOR_BLUE, -1)

        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.keypad(True)

        self._append_log(Log.INFO, "TUI", "Headless control panel started.")

        while True:
            # Dimensions
            h, w = stdscr.getmaxyx()
            if h < 24 or w < 80:
                stdscr.clear()
                stdscr.addstr(0, 0, "Terminal window too small! Require at least 80x24.", curses.A_BOLD | curses.color_pair(4))
                stdscr.refresh()
                time.sleep(0.5)
                # Check for quit key even when small
                ch = stdscr.getch()
                if ch in (ord('q'), ord('Q')):
                    break
                continue

            # Layout split
            # status panel: 10 rows
            # control panel: 9 rows
            # log panel: rest of height
            status_w = curses.newwin(9, w, 0, 0)
            ctrl_w = curses.newwin(9, w, 9, 0)
            log_h = h - 18
            log_w = curses.newwin(log_h, w, 18, 0)

            # Draw components
            self.draw_status(status_w)
            self.draw_control(ctrl_w)
            self.draw_logs(log_w, w, log_h)

            curses.doupdate()

            # Handle keystroke
            ch = stdscr.getch()
            if ch != -1:
                if ch in (ord('q'), ord('Q')):
                    if self.node and self.node.sweep_busy():
                        self._append_log(Log.WARN, "TUI", "Stopping running sweep before exiting...")
                        self.node.cancel_sweep()
                        time.sleep(1.0)
                    break
                elif ch in (ord('s'), ord('S')):
                    if self.node and self.node.sweep_busy():
                        self._append_log(Log.WARN, "TUI", "Sweep already in progress!")
                    else:
                        cfg = SweepConfig(
                            start_mm=self.start_mm,
                            end_mm=self.end_mm,
                            speed_mm_s=self.speed_mm_s,
                            tolerance_mm=self.tolerance_mm,
                            arrival_timeout_s=self.arrival_timeout_s
                        )
                        ok = self.node.start_sweep(cfg)
                        if ok:
                            self._append_log(Log.INFO, "TUI", f"Started sweep instantly: {self.start_mm} -> {self.end_mm} mm @ {self.speed_mm_s} mm/s")
                        else:
                            self._append_log(Log.WARN, "TUI", "Sweep start rejected.")
                elif ch in (ord('p'), ord('P')):
                    if self.node and self.node.sweep_busy():
                        self._append_log(Log.WARN, "TUI", "Cannot edit parameters while sweep is running.")
                    else:
                        if self.get_input_params(stdscr, h, w):
                            self._append_log(Log.INFO, "TUI", f"Parameters updated: {self.start_mm} -> {self.end_mm} mm @ {self.speed_mm_s} mm/s")
                elif ch in (ord('c'), ord('C')):
                    if self.node:
                        self.node.cancel_sweep()
                        self._append_log(Log.INFO, "TUI", "Halt command issued.")
                elif ch in (ord('x'), ord('X')):
                    if self.node:
                        if self.node.sweep_busy():
                            self._append_log(Log.WARN, "TUI", "Cannot clear cloud while sweep is running.")
                        else:
                            self.node.call_trigger(self.node.SVC_CLEAR)
                            self._append_log(Log.INFO, "TUI", "Clear cloud command sent.")

            time.sleep(0.05)


def main() -> None:
    bridge = ROSBridge()
    worker = WorkerThread(bridge)
    worker.start()
    if not worker.wait_ready(5.0):
        print("ERROR: ROS worker failed to come up within 5 s", file=sys.stderr)
        sys.exit(1)

    tui = FusionTUI(bridge, worker)
    try:
        curses.wrapper(tui.run)
    finally:
        worker.shutdown()
        worker.join(timeout=2.0)


if __name__ == "__main__":
    main()
