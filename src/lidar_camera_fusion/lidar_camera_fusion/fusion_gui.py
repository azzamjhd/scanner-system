"""
fusion_gui.py
=============
PyQt5 control GUI for the lidar_camera_fusion package.

Single window, four panels:
  1. Node status (heartbeat-based, polled at 2 Hz)
  2. Sweep control (Start/Stop with state machine)
  3. Auto-discovered parameters for scan_assembler_node and cloud_colorizer_node
  4. /rosout log viewer (all nodes)
"""
from __future__ import annotations

import datetime as dt
import sys
from typing import Dict, List, Optional

from PyQt5.QtCore import QObject, Qt, QTimer, pyqtSlot
from PyQt5.QtGui import QColor, QFont, QTextCharFormat, QTextCursor
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QSpinBox,
    QSplitter,
    QStatusBar,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from rcl_interfaces.msg import Log, ParameterType

from lidar_camera_fusion.ros_worker import (
    ROSBridge,
    SweepConfig,
    WorkerThread,
)


# ── Small reusable widgets ──────────────────────────────────────────────────


class _Dot(QLabel):
    """Colored circular indicator. State: 'green' / 'yellow' / 'red' / 'gray'."""

    _COLORS = {
        "green":  "#22c55e",
        "yellow": "#eab308",
        "red":    "#ef4444",
        "gray":   "#6b7280",
    }

    def __init__(self) -> None:
        super().__init__()
        self.setFixedSize(14, 14)
        self.set_state("gray")

    def set_state(self, state: str) -> None:
        c = self._COLORS.get(state, self._COLORS["gray"])
        self.setStyleSheet(
            f"background-color: {c};"
            f"border-radius: 7px;"
            f"border: 1px solid #1f2937;"
        )


class _StatusRow:
    """One row in the node-status panel."""

    def __init__(self, parent_layout: QGridLayout, row: int, label: str) -> None:
        self.dot = _Dot()
        self.label = QLabel(label)
        self.detail = QLabel("—")
        self.detail.setStyleSheet("color: #6b7280;")
        parent_layout.addWidget(self.dot,    row, 0)
        parent_layout.addWidget(self.label,  row, 1)
        parent_layout.addWidget(self.detail, row, 2)
        parent_layout.setColumnStretch(1, 1)
        parent_layout.setColumnStretch(2, 2)

    def set(self, state: str, detail: str) -> None:
        self.dot.set_state(state)
        self.detail.setText(detail)


# ── Main window ─────────────────────────────────────────────────────────────


class FusionMainWindow(QMainWindow):

    # Heartbeat thresholds in seconds
    FRESH_S = 2.0
    STALE_S = 5.0

    def __init__(self, bridge: ROSBridge, worker: WorkerThread) -> None:
        super().__init__()
        self.bridge = bridge
        self.worker = worker
        self.node = worker.node      # FusionGuiNode

        self.setWindowTitle("LiDAR-Camera Fusion — Control")
        self.resize(1100, 760)

        self._is_scanning = False
        self._latest_pos_mm: Optional[float] = None

        # ── Build layout ────────────────────────────────────────────────────
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        top_split = QSplitter(Qt.Horizontal)
        root.addWidget(top_split, 1)

        # left column: status + sweep
        left_col = QWidget()
        left_lay = QVBoxLayout(left_col)
        left_lay.setContentsMargins(0, 0, 0, 0)
        left_lay.addWidget(self._build_status_panel())
        left_lay.addWidget(self._build_sweep_panel())
        left_lay.addStretch(1)
        top_split.addWidget(left_col)

        # right column: params
        top_split.addWidget(self._build_params_panel())
        top_split.setStretchFactor(0, 1)
        top_split.setStretchFactor(1, 1)

        # bottom: log
        root.addWidget(self._build_log_panel(), 1)

        self.setStatusBar(QStatusBar())
        self.statusBar().showMessage("Ready")

        # ── Wire signals ────────────────────────────────────────────────────
        bridge.is_scanning_changed.connect(self._on_is_scanning)
        bridge.position_heartbeat .connect(self._on_position)
        bridge.service_result     .connect(self._on_service_result)
        bridge.sweep_state_changed.connect(self._on_sweep_state)
        bridge.rosout             .connect(self._on_rosout)
        bridge.log                .connect(self._on_internal_log)
        bridge.params_discovered  .connect(self._on_params_discovered)

        # 2 Hz status poll
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._refresh_status)
        self.status_timer.start(500)

        # discover params after the executor is up — give nodes a moment to publish
        QTimer.singleShot(800,  lambda: self._discover("scan_assembler_node"))
        QTimer.singleShot(1200, lambda: self._discover("cloud_colorizer_node"))

    # ── Panel builders ──────────────────────────────────────────────────────

    def _build_status_panel(self) -> QGroupBox:
        box = QGroupBox("Node status")
        grid = QGridLayout(box)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(6)

        self.row_micro     = _StatusRow(grid, 0, "micro_ros_agent")
        self.row_rplidar   = _StatusRow(grid, 1, "rplidar_ros (/scan)")
        self.row_rsp       = _StatusRow(grid, 2, "robot_state_publisher (TF)")
        self.row_camera    = _StatusRow(grid, 3, "v4l2_camera (/image_raw)")
        self.row_assembler = _StatusRow(grid, 4, "scan_assembler_node")
        self.row_colorizer = _StatusRow(grid, 5, "cloud_colorizer_node")

        return box

    def _build_sweep_panel(self) -> QGroupBox:
        box = QGroupBox("Gantry sweep")
        lay = QVBoxLayout(box)

        form = QFormLayout()
        self.sb_start = QDoubleSpinBox()
        self.sb_start.setRange(-2000.0, 2000.0)
        self.sb_start.setDecimals(1)
        self.sb_start.setSuffix(" mm")
        self.sb_start.setSingleStep(1.0)
        self.sb_start.setValue(0.0)
        form.addRow("Start", self.sb_start)

        self.sb_end = QDoubleSpinBox()
        self.sb_end.setRange(-2000.0, 2000.0)
        self.sb_end.setDecimals(1)
        self.sb_end.setSuffix(" mm")
        self.sb_end.setSingleStep(1.0)
        self.sb_end.setValue(100.0)
        form.addRow("End", self.sb_end)

        self.sb_speed = QDoubleSpinBox()
        self.sb_speed.setRange(0.0, 200.0)
        self.sb_speed.setDecimals(1)
        self.sb_speed.setSuffix(" mm/s")
        self.sb_speed.setSingleStep(1.0)
        self.sb_speed.setValue(20.0)
        form.addRow("Speed", self.sb_speed)

        self.sb_tol = QDoubleSpinBox()
        self.sb_tol.setRange(0.05, 10.0)
        self.sb_tol.setDecimals(2)
        self.sb_tol.setSuffix(" mm")
        self.sb_tol.setSingleStep(0.1)
        self.sb_tol.setValue(0.5)
        form.addRow("Arrival tol.", self.sb_tol)

        self.sb_timeout = QDoubleSpinBox()
        self.sb_timeout.setRange(1.0, 600.0)
        self.sb_timeout.setDecimals(0)
        self.sb_timeout.setSuffix(" s")
        self.sb_timeout.setSingleStep(5.0)
        self.sb_timeout.setValue(60.0)
        form.addRow("Move timeout", self.sb_timeout)

        lay.addLayout(form)

        # current position read-out
        pos_row = QHBoxLayout()
        pos_row.addWidget(QLabel("Current:"))
        self.lbl_current = QLabel("— mm")
        self.lbl_current.setStyleSheet("font-family: monospace; font-weight: bold;")
        pos_row.addWidget(self.lbl_current)
        pos_row.addStretch(1)
        lay.addLayout(pos_row)

        # state label
        self.lbl_state = QLabel("State: IDLE")
        self.lbl_state.setStyleSheet("color: #374151;")
        lay.addWidget(self.lbl_state)

        # buttons
        btn_row = QHBoxLayout()
        self.btn_start = QPushButton("Start sweep")
        self.btn_stop  = QPushButton("Stop / interrupt")
        self.btn_clear = QPushButton("Clear cloud")
        self.btn_start.clicked.connect(self._on_start_clicked)
        self.btn_stop .clicked.connect(self._on_stop_clicked)
        self.btn_clear.clicked.connect(self._on_clear_clicked)
        btn_row.addWidget(self.btn_start)
        btn_row.addWidget(self.btn_stop)
        btn_row.addWidget(self.btn_clear)
        lay.addLayout(btn_row)

        return box

    def _build_params_panel(self) -> QGroupBox:
        box = QGroupBox("Node parameters (live)")
        lay = QVBoxLayout(box)

        # node selector + refresh
        top = QHBoxLayout()
        top.addWidget(QLabel("Node:"))
        self.cmb_node = QComboBox()
        self.cmb_node.addItems(["scan_assembler_node", "cloud_colorizer_node"])
        self.cmb_node.currentTextChanged.connect(self._discover)
        top.addWidget(self.cmb_node, 1)
        self.btn_refresh = QToolButton()
        self.btn_refresh.setText("Refresh")
        self.btn_refresh.clicked.connect(
            lambda: self._discover(self.cmb_node.currentText()))
        top.addWidget(self.btn_refresh)
        lay.addLayout(top)

        # form area (rebuilt on each discovery)
        self.params_host = QWidget()
        self.params_host_layout = QFormLayout(self.params_host)
        self.params_host_layout.setLabelAlignment(Qt.AlignRight)
        lay.addWidget(self.params_host, 1)

        self.lbl_params_status = QLabel("Loading…")
        self.lbl_params_status.setStyleSheet("color: #6b7280;")
        lay.addWidget(self.lbl_params_status)

        # widget registry: node_name → list of (param_dict, getter_fn, widget)
        self._param_widgets: Dict[str, List[dict]] = {}
        return box

    def _build_log_panel(self) -> QGroupBox:
        box = QGroupBox("Log (/rosout — all nodes)")
        lay = QVBoxLayout(box)

        ctrl = QHBoxLayout()
        self.chk_autoscroll = QCheckBox("Auto-scroll")
        self.chk_autoscroll.setChecked(True)
        ctrl.addWidget(self.chk_autoscroll)
        ctrl.addStretch(1)
        self.btn_clear_log = QPushButton("Clear")
        self.btn_save_log  = QPushButton("Save…")
        self.btn_clear_log.clicked.connect(self._clear_log)
        self.btn_save_log .clicked.connect(self._save_log)
        ctrl.addWidget(self.btn_clear_log)
        ctrl.addWidget(self.btn_save_log)
        lay.addLayout(ctrl)

        self.txt_log = QPlainTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setMaximumBlockCount(2000)
        self.txt_log.setFont(QFont("Monospace", 9))
        lay.addWidget(self.txt_log, 1)
        return box

    # ── Status polling ──────────────────────────────────────────────────────

    def _classify(self, age: Optional[float]) -> str:
        if age is None:
            return "red"
        if age < self.FRESH_S:
            return "green"
        if age < self.STALE_S:
            return "yellow"
        return "red"

    def _refresh_status(self) -> None:
        if self.node is None:
            return
        # micro_ros_agent → /current_position
        age = self.node.heartbeat_age(self.node.POSITION_TOPIC)
        self.row_micro.set(
            self._classify(age),
            f"{age:.1f}s ago" if age is not None else "no msgs")

        # rplidar → /scan
        age = self.node.heartbeat_age(self.node.SCAN_TOPIC)
        self.row_rplidar.set(
            self._classify(age),
            f"{age:.1f}s ago" if age is not None else "no msgs")

        # camera → /image_raw
        age = self.node.heartbeat_age(self.node.IMAGE_TOPIC)
        self.row_camera.set(
            self._classify(age),
            f"{age:.1f}s ago" if age is not None else "no msgs")

        # robot_state_publisher → TF lookup
        try:
            tf_ok = self.node.tf_base_to_lidar_ok()
        except Exception:   # noqa: BLE001
            tf_ok = False
        self.row_rsp.set("green" if tf_ok else "red",
                         "base_link → lidar_link OK" if tf_ok
                         else "TF not available")

        # node-name based detection for assembler and colorizer
        running = set(self.node.list_running_nodes())
        for row, name in (
            (self.row_assembler, "scan_assembler_node"),
            (self.row_colorizer, "cloud_colorizer_node"),
        ):
            if name in running:
                row.set("green", "running")
            else:
                row.set("red", "not running")

        # button enable/disable
        self.btn_start.setEnabled(
            (self.node.sweep_busy() is False)
            and ("scan_assembler_node" in running)
            and (age is None or True)   # unrelated; keep enabled regardless of camera
        )
        # stop button should always work while sweeping or during a manual scan
        self.btn_stop.setEnabled(
            self.node.sweep_busy() or self._is_scanning)
        self.btn_clear.setEnabled(
            "scan_assembler_node" in running and not self.node.sweep_busy())

    # ── Slots ───────────────────────────────────────────────────────────────

    @pyqtSlot(bool)
    def _on_is_scanning(self, scanning: bool) -> None:
        self._is_scanning = scanning
        self.statusBar().showMessage(
            f"Scanning: {'YES' if scanning else 'no'}    |    "
            f"State: {self.node.sweep_state_name() if self.node else '—'}")

    @pyqtSlot(float)
    def _on_position(self, mm: float) -> None:
        self._latest_pos_mm = mm
        self.lbl_current.setText(f"{mm:8.2f} mm")

    @pyqtSlot(str, bool, str)
    def _on_service_result(self, name: str, ok: bool, msg: str) -> None:
        level = Log.INFO if ok else Log.ERROR
        self._append_log(level, "fusion_gui", f"{name}: {msg}")

    @pyqtSlot(str, str)
    def _on_sweep_state(self, state_name: str, human: str) -> None:
        self.lbl_state.setText(f"State: {state_name} — {human}")
        self._append_log(Log.INFO, "fusion_gui", f"[{state_name}] {human}")

    @pyqtSlot(int, str, str)
    def _on_rosout(self, level: int, name: str, msg: str) -> None:
        self._append_log(level, name, msg)

    @pyqtSlot(int, str)
    def _on_internal_log(self, level: int, msg: str) -> None:
        self._append_log(level, "fusion_gui", msg)

    # ── Buttons ─────────────────────────────────────────────────────────────

    def _on_start_clicked(self) -> None:
        if self.node is None:
            return
        cfg = SweepConfig(
            start_mm=float(self.sb_start.value()),
            end_mm=float(self.sb_end.value()),
            speed_mm_s=float(self.sb_speed.value()),
            tolerance_mm=float(self.sb_tol.value()),
            arrival_timeout_s=float(self.sb_timeout.value()),
        )
        if abs(cfg.end_mm - cfg.start_mm) < cfg.tolerance_mm:
            self._append_log(
                Log.WARN, "fusion_gui",
                "Start and End are within tolerance — refusing to sweep")
            return
        ok = self.node.start_sweep(cfg)
        if not ok:
            self._append_log(Log.WARN, "fusion_gui", "Sweep start refused")

    def _on_stop_clicked(self) -> None:
        if self.node is None:
            return
        self.node.cancel_sweep()

    def _on_clear_clicked(self) -> None:
        if self.node is None:
            return
        self.node.call_trigger(self.node.SVC_CLEAR)

    # ── Param discovery ─────────────────────────────────────────────────────

    def _discover(self, node_name: str) -> None:
        if self.node is None or not node_name:
            return
        self.lbl_params_status.setText(f"Discovering parameters for {node_name}…")
        # clear current widgets
        while self.params_host_layout.rowCount():
            self.params_host_layout.removeRow(0)
        self._param_widgets[node_name] = []
        self.node.discover_params(node_name)

    @pyqtSlot(str, list)
    def _on_params_discovered(self, node_name: str, params: list) -> None:
        # only render if this is the currently-selected node
        if node_name != self.cmb_node.currentText():
            return
        # clear any leftover rows
        while self.params_host_layout.rowCount():
            self.params_host_layout.removeRow(0)

        if not params:
            self.lbl_params_status.setText(f"No parameters available on {node_name}")
            return

        self._param_widgets[node_name] = []
        for p in params:
            w, getter = self._make_param_widget(p)
            row_lbl = QLabel(p["name"])
            if p.get("description"):
                row_lbl.setToolTip(p["description"])
            row_widget = QWidget()
            rh = QHBoxLayout(row_widget)
            rh.setContentsMargins(0, 0, 0, 0)
            rh.addWidget(w, 1)
            apply_btn = QPushButton("Apply")
            apply_btn.setFixedWidth(70)
            if p.get("read_only"):
                w.setEnabled(False)
                apply_btn.setEnabled(False)
                apply_btn.setToolTip("read-only")
            apply_btn.clicked.connect(
                lambda _checked=False, n=node_name, p=p, g=getter:
                self._apply_param(n, p, g))
            rh.addWidget(apply_btn)
            self.params_host_layout.addRow(row_lbl, row_widget)
            self._param_widgets[node_name].append({"param": p, "getter": getter})

        self.lbl_params_status.setText(f"{len(params)} parameters loaded")

    def _make_param_widget(self, p: dict):
        t = p["type"]
        v = p["value"]
        if t == ParameterType.PARAMETER_BOOL:
            cb = QCheckBox()
            cb.setChecked(bool(v) if v is not None else False)
            return cb, cb.isChecked
        if t == ParameterType.PARAMETER_INTEGER:
            sp = QSpinBox()
            sp.setRange(-2_147_483_648, 2_147_483_647)
            sp.setValue(int(v) if v is not None else 0)
            return sp, sp.value
        if t == ParameterType.PARAMETER_DOUBLE:
            ds = QDoubleSpinBox()
            ds.setDecimals(6)
            ds.setRange(-1e9, 1e9)
            ds.setValue(float(v) if v is not None else 0.0)
            return ds, ds.value
        if t == ParameterType.PARAMETER_STRING:
            le = QLineEdit()
            le.setText(str(v) if v is not None else "")
            return le, le.text
        # arrays / unknown → text line, comma separated; sent as string (best effort)
        le = QLineEdit()
        if isinstance(v, list):
            le.setText(", ".join(str(x) for x in v))
        elif v is not None:
            le.setText(str(v))
        return le, le.text

    def _apply_param(self, node_name: str, p: dict, getter) -> None:
        if self.node is None:
            return
        t = p["type"]
        raw = getter()
        try:
            if t == ParameterType.PARAMETER_BOOL:
                value = bool(raw)
            elif t == ParameterType.PARAMETER_INTEGER:
                value = int(raw)
            elif t == ParameterType.PARAMETER_DOUBLE:
                value = float(raw)
            elif t == ParameterType.PARAMETER_STRING:
                value = str(raw)
            elif t == ParameterType.PARAMETER_INTEGER_ARRAY:
                value = [int(x.strip()) for x in str(raw).split(",") if x.strip()]
            elif t == ParameterType.PARAMETER_DOUBLE_ARRAY:
                value = [float(x.strip()) for x in str(raw).split(",") if x.strip()]
            elif t == ParameterType.PARAMETER_STRING_ARRAY:
                value = [x.strip() for x in str(raw).split(",") if x.strip()]
            elif t == ParameterType.PARAMETER_BOOL_ARRAY:
                value = [x.strip().lower() in ("1", "true", "yes")
                         for x in str(raw).split(",") if x.strip()]
            else:
                self._append_log(
                    Log.WARN, "fusion_gui",
                    f"Cannot edit parameter {p['name']} (type {t})")
                return
        except ValueError as ex:
            self._append_log(
                Log.ERROR, "fusion_gui",
                f"Bad value for {p['name']}: {ex}")
            return
        self.node.set_param(node_name, p["name"], value)

    # ── Log handling ────────────────────────────────────────────────────────

    _LEVEL_NAME = {
        Log.DEBUG: "DEBUG",
        Log.INFO:  "INFO",
        Log.WARN:  "WARN",
        Log.ERROR: "ERROR",
        Log.FATAL: "FATAL",
    }
    _LEVEL_COLOR = {
        Log.DEBUG: QColor("#9ca3af"),
        Log.INFO:  QColor("#111827"),
        Log.WARN:  QColor("#b45309"),
        Log.ERROR: QColor("#b91c1c"),
        Log.FATAL: QColor("#7f1d1d"),
    }

    def _append_log(self, level: int, name: str, msg: str) -> None:
        ts = dt.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        lvl_name = self._LEVEL_NAME.get(int(level), str(level))
        line = f"[{ts}] {lvl_name:5s} {name}: {msg}"

        cursor = self.txt_log.textCursor()
        cursor.movePosition(QTextCursor.End)
        fmt = QTextCharFormat()
        fmt.setForeground(self._LEVEL_COLOR.get(int(level), QColor("#111827")))
        if int(level) >= Log.WARN:
            f = QFont("Monospace", 9)
            f.setBold(True)
            fmt.setFont(f)
        cursor.insertText(line + "\n", fmt)

        if self.chk_autoscroll.isChecked():
            self.txt_log.moveCursor(QTextCursor.End)

    def _clear_log(self) -> None:
        self.txt_log.clear()

    def _save_log(self) -> None:
        from PyQt5.QtWidgets import QFileDialog
        path, _ = QFileDialog.getSaveFileName(
            self, "Save log", "fusion_gui.log", "Log files (*.log *.txt)")
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(self.txt_log.toPlainText())
            self._append_log(Log.INFO, "fusion_gui", f"Log saved to {path}")
        except OSError as ex:
            self._append_log(Log.ERROR, "fusion_gui", f"Save failed: {ex}")

    # ── Shutdown ────────────────────────────────────────────────────────────

    def closeEvent(self, event) -> None:    # noqa: N802 (Qt API)
        try:
            if self.node is not None and self.node.sweep_busy():
                self.node.cancel_sweep()
        finally:
            self.worker.shutdown()
            super().closeEvent(event)


# ── main ────────────────────────────────────────────────────────────────────


def main() -> None:
    app = QApplication(sys.argv)

    bridge = ROSBridge()
    worker = WorkerThread(bridge)
    worker.start()
    if not worker.wait_ready(5.0):
        print("ERROR: ROS worker failed to come up within 5 s", file=sys.stderr)
        sys.exit(1)

    win = FusionMainWindow(bridge, worker)
    win.show()
    rc = app.exec_()
    worker.shutdown()
    worker.join(timeout=2.0)
    sys.exit(rc)


if __name__ == "__main__":
    main()
