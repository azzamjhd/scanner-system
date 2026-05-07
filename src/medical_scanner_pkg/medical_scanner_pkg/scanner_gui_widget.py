"""
Scanner Control GUI — PyQt5, default theme + stock widgets.

Clean re-layout of the original. Uses native QGroupBox, QFormLayout,
QPushButton, QDoubleSpinBox, QPlainTextEdit, QStatusBar, QSplitter,
QTabWidget — no custom stylesheets, no custom-painted widgets.
The system's native Qt theme is used as-is.

Drop-in usage in your ROS2 package
──────────────────────────────────
    from scanner_gui_widget import ScannerGUI

    gui = ScannerGUI()
    gui.show()

    # Push live data from your node:
    gui.set_position(mm)
    gui.set_motor_speed(rpm)
    gui.set_points_published(n)
    gui.set_connected(True)
    gui.set_camera_image(qimage)        # cv_bridge → QImage
    gui.set_status('Scanning')
    gui.append_log('Scan started')

    # Listen for operator actions (all signals carry plain values):
    gui.start_scan_clicked.connect(node.on_start)
    gui.stop_scan_clicked.connect(node.on_stop)
    gui.scan_range_changed.connect(node.on_scan_range)         # (start, end)
    gui.move_command_sent.connect(node.on_move)                # (target, speed, accel)
    gui.home_clicked.connect(node.on_home)
    gui.go_to_start_clicked.connect(node.on_go_start)
    gui.go_to_end_clicked.connect(node.on_go_end)
    gui.scanner_settings_applied.connect(node.on_scanner_apply)# (a_min, a_max, range, sim_enc)
    gui.scanner_settings_refresh.connect(node.on_scanner_refresh)
    gui.camera_topic_changed.connect(node.on_camera_topic)     # (str)
    gui.launch_rviz_clicked.connect(node.on_launch_rviz)
    gui.clear_rviz_clicked.connect(node.on_clear_rviz)

Standalone test:
    python3 scanner_gui_widget.py
"""

from datetime import datetime

from PyQt5.QtCore import QSize, Qt, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
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
    QScrollArea,
    QSizePolicy,
    QSplitter,
    QStatusBar,
    QVBoxLayout,
    QWidget,
)


# ─────────────────────────────────────────────────────────────────
# Camera display: a simple QLabel that scales a QImage to fit.
# ─────────────────────────────────────────────────────────────────
class CameraView(QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(320, 240)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setFrameShape(QFrame.StyledPanel)
        self.setText("No camera feed")
        self._pixmap = None

    def set_image(self, qimage: QImage):
        if qimage is None or qimage.isNull():
            self._pixmap = None
            self.setText("No camera feed")
            return
        self._pixmap = QPixmap.fromImage(qimage)
        self._update_scaled()

    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        self._update_scaled()

    def _update_scaled(self):
        if self._pixmap is None:
            return
        self.setPixmap(
            self._pixmap.scaled(
                self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
        )


# ─────────────────────────────────────────────────────────────────
# Main GUI
# ─────────────────────────────────────────────────────────────────
class ScannerGUI(QMainWindow):
    # ── operator-action signals ──
    start_scan_clicked = pyqtSignal()
    stop_scan_clicked = pyqtSignal()
    scan_range_changed = pyqtSignal(float, float)
    move_command_sent = pyqtSignal(float, float, float)  # target, speed, accel
    home_clicked = pyqtSignal()
    go_to_start_clicked = pyqtSignal()
    go_to_end_clicked = pyqtSignal()
    scanner_settings_applied = pyqtSignal(float, float, float, bool)
    scanner_settings_refresh = pyqtSignal()
    camera_topic_changed = pyqtSignal(str)
    launch_rviz_clicked = pyqtSignal()
    clear_rviz_clicked = pyqtSignal()
    capture_spacing_changed = pyqtSignal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Scanner Control")
        self.resize(1200, 760)

        # state mirror
        self._connected = False
        self._position_mm = 0.0
        self._motor_rpm = 0.0
        self._points_pub = 0
        self._scan_status = "Idle"
        self._rviz_running = False

        self._build_ui()
        self._wire_signals()
        self._apply_accent_styles()
        self._refresh_status_bar()

    # ─────────────────── light accent stylesheet ───────────────────
    # Native theme stays the default for everything; we only color a
    # handful of action buttons + status indicators so the user can
    # spot primary actions and state at a glance.
    def _apply_accent_styles(self):
        # Group-box titles get a subtle weight bump
        self.setStyleSheet("""
            QGroupBox {
                font-weight: 600;
                border: 1px solid palette(mid);
                border-radius: 4px;
                margin-top: 10px;
                padding-top: 8px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 6px;
            }
        """)

        # Primary / success — Start Scan, Send Move, Apply Settings
        primary_css = """
            QPushButton {
                background-color: #2e7d32;
                color: white;
                border: 1px solid #1b5e20;
                border-radius: 3px;
                padding: 6px 12px;
                font-weight: 600;
            }
            QPushButton:hover    { background-color: #388e3c; }
            QPushButton:pressed  { background-color: #1b5e20; }
            QPushButton:disabled { background-color: #a5d6a7; color: #eef; border-color: #88b88a; }
        """

        # Danger — Stop Scan
        danger_css = """
            QPushButton {
                background-color: #c62828;
                color: white;
                border: 1px solid #8e0000;
                border-radius: 3px;
                padding: 6px 12px;
                font-weight: 600;
            }
            QPushButton:hover    { background-color: #d32f2f; }
            QPushButton:pressed  { background-color: #8e0000; }
            QPushButton:disabled { background-color: #ef9a9a; color: #fee; border-color: #b27070; }
        """

        # Info — Apply Camera Topic, Refresh
        info_css = """
            QPushButton {
                background-color: #1565c0;
                color: white;
                border: 1px solid #0d47a1;
                border-radius: 3px;
                padding: 6px 12px;
                font-weight: 600;
            }
            QPushButton:hover    { background-color: #1976d2; }
            QPushButton:pressed  { background-color: #0d47a1; }
        """

        # Accent — Launch RViz (purple), Clear RViz (orange)
        purple_css = """
            QPushButton {
                background-color: #6a1b9a;
                color: white;
                border: 1px solid #4a148c;
                border-radius: 3px;
                padding: 6px 12px;
                font-weight: 600;
            }
            QPushButton:hover    { background-color: #7b1fa2; }
            QPushButton:pressed  { background-color: #4a148c; }
        """
        orange_css = """
            QPushButton {
                background-color: #ef6c00;
                color: white;
                border: 1px solid #b53d00;
                border-radius: 3px;
                padding: 6px 12px;
                font-weight: 600;
            }
            QPushButton:hover    { background-color: #f57c00; }
            QPushButton:pressed  { background-color: #b53d00; }
        """

        self.btn_start_scan.setStyleSheet(primary_css)
        self.btn_send_move.setStyleSheet(primary_css)
        self.btn_apply_settings.setStyleSheet(info_css)
        self.btn_apply_topic.setStyleSheet(info_css)
        self.btn_refresh_settings.setStyleSheet(info_css)
        self.btn_stop_scan.setStyleSheet(danger_css)
        self.btn_launch_rviz.setStyleSheet(purple_css)
        self.btn_clear_rviz.setStyleSheet(orange_css)

        # Camera view — subtle dark frame so an empty feed is obvious
        self.camera_view.setStyleSheet("""
            QLabel {
                background-color: #1a1a1a;
                color: #888;
                border: 1px solid #444;
                border-radius: 3px;
            }
        """)

        # Log console — dark monospace
        self.log_view.setStyleSheet("""
            QPlainTextEdit {
                background-color: #1e1e1e;
                color: #dcdcdc;
                font-family: 'JetBrains Mono', 'Consolas', 'DejaVu Sans Mono', monospace;
                font-size: 11px;
                border: 1px solid #444;
                border-radius: 3px;
            }
        """)

        # Status colors
        self._color_connection()
        self._color_rviz()

    # ────────────────────────────── UI BUILD ──────────────────────────────

    def _build_ui(self):
        central = QWidget(self)
        self.setCentralWidget(central)

        root = QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # ── system status strip (top) ─────────────────────────────
        root.addWidget(self._build_system_status())

        # ── main 4-column splitter: scanner | gantry | scan/viz | camera
        main_split = QSplitter(Qt.Horizontal)
        main_split.setChildrenCollapsible(False)
        main_split.addWidget(self._build_scanner_settings())
        main_split.addWidget(self._build_gantry_control())
        main_split.addWidget(self._build_scan_control())
        main_split.addWidget(self._build_camera_panel())
        # initial proportions
        main_split.setSizes([240, 240, 320, 480])

        # ── vertical splitter: main row over log console
        v_split = QSplitter(Qt.Vertical)
        v_split.setChildrenCollapsible(False)
        v_split.addWidget(main_split)
        v_split.addWidget(self._build_log_console())
        v_split.setSizes([520, 200])

        root.addWidget(v_split, 1)

        # ── status bar (bottom) ───────────────────────────────────
        sb = QStatusBar(self)
        self.setStatusBar(sb)
        self._sb_conn = QLabel()
        self._sb_status = QLabel()
        self._sb_clock = QLabel()
        sb.addWidget(self._sb_conn)
        sb.addWidget(self._build_v_separator())
        sb.addWidget(self._sb_status)
        sb.addPermanentWidget(self._sb_clock)

    # ----- Top: System Status -----
    def _build_system_status(self):
        gb = QGroupBox("System Status")
        h = QHBoxLayout(gb)
        h.setSpacing(24)

        self.lbl_connection = QLabel("Connection: <b>Disconnected</b>")
        self.lbl_position = QLabel("Current Position: <b>0.00 mm</b>")
        self.lbl_motor = QLabel("Motor Speed: <b>0.00 RPM</b>")
        self.lbl_points = QLabel("Points Published: <b>0</b>")

        for w in (
            self.lbl_connection,
            self.lbl_position,
            self.lbl_motor,
            self.lbl_points,
        ):
            w.setTextFormat(Qt.RichText)
            h.addWidget(w)
        h.addStretch(1)
        return gb

    # ----- Col 1: Scanner Settings -----
    def _build_scanner_settings(self):
        gb = QGroupBox("Scanner Settings")
        v = QVBoxLayout(gb)

        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignLeft)
        form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)

        self.spin_angle_min = self._make_double(-180, 180, 1, 0.0, "°", 2)
        self.spin_angle_max = self._make_double(-180, 180, 1, 90.0, "°", 2)
        self.spin_range_max = self._make_double(0.1, 100, 0.5, 12.0, " m", 2)

        form.addRow("Angle Min:", self.spin_angle_min)
        form.addRow("Angle Max:", self.spin_angle_max)
        form.addRow("Range Max:", self.spin_range_max)
        v.addLayout(form)

        self.chk_sim_encoder = QCheckBox("Simulate Encoder")
        v.addWidget(self.chk_sim_encoder)

        v.addSpacing(4)
        self.btn_apply_settings = QPushButton("Apply Settings")
        self.btn_refresh_settings = QPushButton("Refresh from Node")
        v.addWidget(self.btn_apply_settings)
        v.addWidget(self.btn_refresh_settings)
        v.addStretch(1)
        return gb

    # ----- Col 2: Gantry Control -----
    def _build_gantry_control(self):
        gb = QGroupBox("Gantry Control")
        v = QVBoxLayout(gb)

        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignLeft)
        form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)

        self.spin_target = self._make_double(0, 5000, 10, 1000.0, " mm", 2)
        self.spin_speed = self._make_double(0.1, 500, 1, 50.0, " mm/s", 2)
        self.spin_accel = self._make_double(1, 2000, 5, 120.0, " mm/s²", 2)

        form.addRow("Target Position:", self.spin_target)
        form.addRow("Speed:", self.spin_speed)
        form.addRow("Acceleration:", self.spin_accel)
        v.addLayout(form)

        nav = QHBoxLayout()
        self.btn_home = QPushButton("Home")
        self.btn_start_pos = QPushButton("Start")
        self.btn_end_pos = QPushButton("End")
        for b in (self.btn_home, self.btn_start_pos, self.btn_end_pos):
            nav.addWidget(b)
        v.addLayout(nav)

        self.btn_send_move = QPushButton("Send Move Command")
        self.btn_send_move.setDefault(True)
        v.addWidget(self.btn_send_move)
        v.addStretch(1)
        return gb

    # ----- Col 3: Scan Control + Visualization -----
    def _build_scan_control(self):
        gb = QGroupBox("Scan Control && Visualization")
        v = QVBoxLayout(gb)

        self.lbl_scan_status = QLabel("Status: <b>Idle</b>")
        self.lbl_scan_status.setTextFormat(Qt.RichText)
        v.addWidget(self.lbl_scan_status)

        # Range row
        range_form = QFormLayout()
        range_form.setLabelAlignment(Qt.AlignLeft)
        range_form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)

        rrow = QHBoxLayout()
        self.spin_scan_start = self._make_double(0, 5000, 10, 0.0, " mm", 2)
        self.spin_scan_end = self._make_double(0, 5000, 10, 1000.0, " mm", 2)
        rrow.addWidget(QLabel("Start Pos:"))
        rrow.addWidget(self.spin_scan_start, 1)
        rrow.addSpacing(8)
        rrow.addWidget(QLabel("End Pos:"))
        rrow.addWidget(self.spin_scan_end, 1)
        v.addLayout(rrow)

        v.addSpacing(4)
        self.btn_start_scan = QPushButton("Start Scan")
        self.btn_stop_scan = QPushButton("Stop Scan")
        v.addWidget(self.btn_start_scan)
        v.addWidget(self.btn_stop_scan)

        v.addSpacing(4)
        self.lbl_stitcher_status = QLabel('Stitcher: <b style="color:#666;">Idle</b>')
        self.lbl_stitcher_status.setTextFormat(Qt.RichText)
        v.addWidget(self.lbl_stitcher_status)

        spacing_row = QHBoxLayout()
        spacing_row.addWidget(QLabel("Capture Spacing:"))
        self.spin_capture_spacing = self._make_double(0.1, 10000.0, 1.0, 5.0, " mm", 2)
        spacing_row.addWidget(self.spin_capture_spacing, 1)
        v.addLayout(spacing_row)

        v.addSpacing(8)
        self.lbl_rviz = QLabel("RViz: Not Running")
        v.addWidget(self.lbl_rviz)

        self.btn_launch_rviz = QPushButton("Launch RViz")
        self.btn_clear_rviz = QPushButton("Clear RViz Display")
        v.addWidget(self.btn_launch_rviz)
        v.addWidget(self.btn_clear_rviz)

        v.addStretch(1)
        return gb

    # ----- Col 4: Camera Feed -----
    def _build_camera_panel(self):
        gb = QGroupBox("Camera Feed")
        v = QVBoxLayout(gb)

        topic_row = QHBoxLayout()
        topic_row.addWidget(QLabel("Image Topic:"))
        self.edit_topic = QLineEdit("/camera/image_raw")
        topic_row.addWidget(self.edit_topic, 1)
        self.btn_apply_topic = QPushButton("Apply Camera Topic")
        topic_row.addWidget(self.btn_apply_topic)
        v.addLayout(topic_row)

        self.lbl_topic_status = QLabel("Camera topic: <b>/camera/image_raw</b>")
        self.lbl_topic_status.setTextFormat(Qt.RichText)
        v.addWidget(self.lbl_topic_status)

        self.camera_view = CameraView()
        v.addWidget(self.camera_view, 1)
        return gb

    # ----- Bottom: Log Console -----
    def _build_log_console(self):
        gb = QGroupBox("Log Console")
        v = QVBoxLayout(gb)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(2000)
        v.addWidget(self.log_view)
        return gb

    # ────────────────────────── helpers ──────────────────────────

    @staticmethod
    def _make_double(minimum, maximum, step, value, suffix="", decimals=2):
        s = QDoubleSpinBox()
        s.setRange(minimum, maximum)
        s.setSingleStep(step)
        s.setDecimals(decimals)
        s.setValue(value)
        if suffix:
            s.setSuffix(suffix)
        s.setAlignment(Qt.AlignRight)
        return s

    @staticmethod
    def _build_v_separator():
        sep = QFrame()
        sep.setFrameShape(QFrame.VLine)
        sep.setFrameShadow(QFrame.Sunken)
        return sep

    # ────────────────────────── wire signals ──────────────────────────

    def _wire_signals(self):
        self.btn_apply_settings.clicked.connect(self._emit_scanner_apply)
        self.btn_refresh_settings.clicked.connect(self.scanner_settings_refresh.emit)

        self.btn_home.clicked.connect(self.home_clicked.emit)
        self.btn_start_pos.clicked.connect(self.go_to_start_clicked.emit)
        self.btn_end_pos.clicked.connect(self.go_to_end_clicked.emit)
        self.btn_send_move.clicked.connect(self._emit_move)

        self.btn_start_scan.clicked.connect(self.start_scan_clicked.emit)
        self.btn_stop_scan.clicked.connect(self.stop_scan_clicked.emit)
        self.spin_scan_start.valueChanged.connect(self._emit_scan_range)
        self.spin_scan_end.valueChanged.connect(self._emit_scan_range)

        self.btn_launch_rviz.clicked.connect(self._on_launch_rviz)
        self.btn_clear_rviz.clicked.connect(self._on_clear_rviz)

        self.btn_apply_topic.clicked.connect(self._emit_camera_topic)
        self.edit_topic.returnPressed.connect(self._emit_camera_topic)

        self.spin_capture_spacing.editingFinished.connect(
            lambda: self.capture_spacing_changed.emit(self.spin_capture_spacing.value())
        )

    def _emit_scanner_apply(self):
        self.scanner_settings_applied.emit(
            self.spin_angle_min.value(),
            self.spin_angle_max.value(),
            self.spin_range_max.value(),
            self.chk_sim_encoder.isChecked(),
        )

    def _emit_move(self):
        self.move_command_sent.emit(
            self.spin_target.value(),
            self.spin_speed.value(),
            self.spin_accel.value(),
        )

    def _emit_scan_range(self):
        self.scan_range_changed.emit(
            self.spin_scan_start.value(),
            self.spin_scan_end.value(),
        )

    def _emit_camera_topic(self):
        topic = self.edit_topic.text().strip()
        if topic:
            self.lbl_topic_status.setText(f"Camera topic: <b>{topic}</b>")
            self.camera_topic_changed.emit(topic)

    def _on_launch_rviz(self):
        self._rviz_running = True
        self._color_rviz()
        self.launch_rviz_clicked.emit()

    def _on_clear_rviz(self):
        self._rviz_running = False
        self._color_rviz()
        self.clear_rviz_clicked.emit()

    # ────────────────────────── public API ──────────────────────────

    def set_connected(self, ok: bool):
        self._connected = ok
        self._color_connection()
        self._refresh_status_bar()

    def _color_connection(self):
        if self._connected:
            self.lbl_connection.setText(
                'Connection: <b><span style="color:#2e7d32;">● Connected</span></b>'
            )
        else:
            self.lbl_connection.setText(
                'Connection: <b><span style="color:#c62828;">● Disconnected</span></b>'
            )
        self.lbl_connection.setTextFormat(Qt.RichText)

    def _color_rviz(self):
        if self._rviz_running:
            self.lbl_rviz.setText(
                'RViz: <b><span style="color:#2e7d32;">● Running</span></b>'
            )
        else:
            self.lbl_rviz.setText('RViz: <span style="color:#888;">Not Running</span>')
        self.lbl_rviz.setTextFormat(Qt.RichText)

    def set_position(self, mm: float):
        self._position_mm = mm
        self.lbl_position.setText(f"Current Position: <b>{mm:.2f} mm</b>")

    def set_motor_speed(self, rpm: float):
        self._motor_rpm = rpm
        self.lbl_motor.setText(f"Motor Speed: <b>{rpm:.2f} RPM</b>")

    def set_points_published(self, n: int):
        self._points_pub = n
        self.lbl_points.setText(f"Points Published: <b>{n:,}</b>")

    def set_status(self, text: str):
        """e.g. 'Idle', 'Scanning', 'Paused', 'Homing', 'Error: ...'."""
        self._scan_status = text
        color = {
            "Idle": "#666",
            "Scanning": "#2e7d32",
            "Paused": "#ef6c00",
            "Homing": "#1565c0",
        }.get(text, "#c62828" if text.lower().startswith("error") else "#333")
        self.lbl_scan_status.setText(
            f'Status: <b><span style="color:{color};">{text}</span></b>'
        )
        self.lbl_scan_status.setTextFormat(Qt.RichText)
        self._refresh_status_bar()

    def set_rviz_running(self, running: bool):
        self._rviz_running = running
        self._color_rviz()

    def set_stitcher_status(self, text: str, ok: bool = True):
        """Update the stitcher status label. ok=True → green, ok=False → red."""
        color = "#2e7d32" if ok else "#c62828"
        self.lbl_stitcher_status.setText(
            f'Stitcher: <b><span style="color:{color};">{text}</span></b>'
        )
        self.lbl_stitcher_status.setTextFormat(Qt.RichText)

    def set_capture_spacing(self, mm: float) -> None:
        self.spin_capture_spacing.blockSignals(True)
        self.spin_capture_spacing.setValue(mm)
        self.spin_capture_spacing.blockSignals(False)

    def set_camera_image(self, qimage: QImage):
        self.camera_view.set_image(qimage)

    def set_camera_topic_label(self, topic: str):
        self.lbl_topic_status.setText(f"Camera topic: <b>{topic}</b>")
        # update edit field too if differs
        if self.edit_topic.text() != topic:
            self.edit_topic.setText(topic)

    def append_log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_view.appendPlainText(f"[{ts}] {msg}")

    def clear_log(self):
        self.log_view.clear()

    # ────────────────────────── internal ──────────────────────────

    def _refresh_status_bar(self):
        conn = "Connected" if self._connected else "Disconnected"
        self._sb_conn.setText(f"  {conn}  ")
        self._sb_status.setText(f"  {self._scan_status}  ")
        self._sb_clock.setText(datetime.now().strftime("%Y-%m-%d %H:%M:%S") + "  ")


# ─────────────────────────────────────────────────────────────────
# Standalone launcher
# ─────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import sys

    app = QApplication(sys.argv)
    gui = ScannerGUI()

    # Demo data so the standalone window isn't empty
    gui.set_connected(True)
    gui.set_position(1000.00)
    gui.set_motor_speed(0.00)
    gui.set_points_published(540283)
    gui.set_status("Idle")
    gui.append_log("Gantry move: target=0.00 mm, speed=50.00 mm/s, accel=120.00 mm/s²")
    gui.append_log(
        "Gantry move: target=1500.00 mm, speed=50.00 mm/s, accel=120.00 mm/s²"
    )
    gui.append_log("Gantry move: target=0.00 mm, speed=50.00 mm/s, accel=120.00 mm/s²")
    gui.append_log("Scan started successfully")
    gui.append_log(
        "Auto traverse: current=0.00 mm, start=0.00, end=1000.00 -> target=1000.00 mm"
    )
    gui.append_log("Scan stopped: Saved 19919 points to /tmp/scan_20260426_182244.ply")

    gui.show()
    sys.exit(app.exec_())
