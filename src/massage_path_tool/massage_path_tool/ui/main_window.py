"""Main window for the Massage Path Authoring Tool.

Ties together the canvas, core math, and live worker. Provides a toolbar
for mode switching, session I/O, and live feed control.
"""
from __future__ import annotations

import os
from pathlib import Path

import cv2
import numpy as np
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QAction
from PyQt6.QtWidgets import (
    QMainWindow,
    QToolBar,
    QFileDialog,
    QInputDialog,
    QMessageBox,
    QStatusBar,
    QLabel,
    QApplication,
)

from massage_path_tool.core.homography import (
    compute_h_norm,
    compute_h_proj,
    apply_homography,
    quad_aspect_ratio,
)
from massage_path_tool.core.bezier import bezier_through_points
from massage_path_tool.io import session as session_io
from massage_path_tool.ui.canvas import CanvasView, MODE_VIEW, MODE_POINT, MODE_PATH, MODE_ANCHOR
from massage_path_tool.ui.live_worker import LiveWorker


class AnchorClickMode:
    """State machine for manual 4-point anchor calibration."""

    LABELS = [
        "L-Shoulder (11)",
        "R-Shoulder (12)",
        "R-Hip (24)",
        "L-Hip (23)",
    ]

    def __init__(self, on_complete):
        self._pts: list[list[float]] = []
        self._on_complete = on_complete

    def click(self, x: float, y: float) -> str:
        """Add one anchor click. Returns next prompt or '' when done."""
        self._pts.append([x, y])
        if len(self._pts) == 4:
            self._on_complete(np.float32(self._pts))
            return ""
        return f"Click {self.LABELS[len(self._pts)]}"

    def reset(self):
        self._pts = []


class MainWindow(QMainWindow):
    """Top-level window for the massage path authoring tool."""

    def __init__(self, image_path: str | None = None, session_path: str | None = None):
        super().__init__()
        self.setWindowTitle("Massage Path Authoring Tool")
        self.resize(1200, 900)

        # --- state -----------------------------------------------------------
        self._session = session_io.make_empty_session()
        self._H_norm: np.ndarray | None = None
        self._anchor_mode: AnchorClickMode | None = None
        self._worker: LiveWorker | None = None
        self._pending_reproject: bool = False  # session loaded before anchors set

        # --- canvas ----------------------------------------------------------
        self._canvas = CanvasView(self)
        self.setCentralWidget(self._canvas)

        self._canvas.point_placed.connect(self._on_point_placed)
        self._canvas.path_finished.connect(self._on_path_finished)
        self._canvas.path_cancelled.connect(self._on_path_cancelled)
        self._canvas.anchor_clicked.connect(self._on_anchor_clicked)
        self._canvas.mode_changed.connect(self._on_mode_changed)

        # --- toolbar ---------------------------------------------------------
        self._toolbar = QToolBar("Main")
        self.addToolBar(self._toolbar)

        self._act_open = QAction("Open Image", self)
        self._act_open.triggered.connect(self._open_image)
        self._toolbar.addAction(self._act_open)

        self._act_detect = QAction("Detect Anchors", self)
        self._act_detect.triggered.connect(self._detect_anchors)
        self._act_detect.setEnabled(False)
        self._toolbar.addAction(self._act_detect)

        self._act_manual_anchor = QAction("Set Anchors Manually", self)
        self._act_manual_anchor.triggered.connect(self._start_manual_anchor)
        self._act_manual_anchor.setEnabled(False)
        self._toolbar.addAction(self._act_manual_anchor)

        self._toolbar.addSeparator()

        self._act_point = QAction("Set Points", self)
        self._act_point.triggered.connect(lambda: self._canvas.set_mode(MODE_POINT))
        self._act_point.setEnabled(False)
        self._toolbar.addAction(self._act_point)

        self._act_path = QAction("Draw Path", self)
        self._act_path.triggered.connect(lambda: self._canvas.set_mode(MODE_PATH))
        self._act_path.setEnabled(False)
        self._toolbar.addAction(self._act_path)

        self._act_view = QAction("View", self)
        self._act_view.triggered.connect(lambda: self._canvas.set_mode(MODE_VIEW))
        self._toolbar.addAction(self._act_view)

        self._toolbar.addSeparator()

        self._act_save = QAction("Save Session", self)
        self._act_save.triggered.connect(self._save_session)
        self._act_save.setEnabled(False)
        self._toolbar.addAction(self._act_save)

        self._act_load = QAction("Load Session", self)
        self._act_load.triggered.connect(self._load_session)
        self._toolbar.addAction(self._act_load)

        self._toolbar.addSeparator()

        self._act_start_live = QAction("Start Live Feed", self)
        self._act_start_live.triggered.connect(self._start_live)
        self._act_start_live.setEnabled(False)
        self._toolbar.addAction(self._act_start_live)

        self._act_stop_live = QAction("Stop Live Feed", self)
        self._act_stop_live.triggered.connect(self._stop_live)
        self._act_stop_live.setEnabled(False)
        self._toolbar.addAction(self._act_stop_live)

        # --- status bar ------------------------------------------------------
        self._status = QStatusBar(self)
        self.setStatusBar(self._status)
        self._status_label = QLabel("Ready")
        self._status.addWidget(self._status_label)

        # --- auto-load -------------------------------------------------------
        if image_path and os.path.exists(image_path):
            self._load_image(image_path)
        if session_path and os.path.exists(session_path):
            self._do_load_session(session_path)

    # --- image / anchor ----------------------------------------------------

    def _open_image(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Open Image", "", "Images (*.png *.jpg *.jpeg *.bmp)"
        )
        if path:
            self._load_image(path)

    def _load_image(self, path: str):
        ok = self._canvas.load_image(path)
        if not ok:
            QMessageBox.critical(self, "Error", f"Failed to load image:\n{path}")
            return
        self._session = session_io.make_empty_session(source_image=path)
        self._H_norm = None
        self._canvas.clear_overlay()
        self._act_detect.setEnabled(True)
        self._act_manual_anchor.setEnabled(True)
        self._act_point.setEnabled(False)
        self._act_path.setEnabled(False)
        self._act_save.setEnabled(False)
        self._act_start_live.setEnabled(False)
        self._status_label.setText(f"Image loaded: {Path(path).name}")

    def _detect_anchors(self):
        from massage_path_tool.core.pose_detector import PoseDetector
        if self._canvas._bg_item is None:
            return
        # Grab the current background pixmap and convert to numpy
        pixmap = self._canvas._bg_item.pixmap()
        img = pixmap.toImage()
        # Convert QImage to numpy BGR
        w, h = img.width(), img.height()
        ptr = img.bits()
        ptr.setsize(h * w * 4)
        arr = np.frombuffer(ptr, np.uint8).reshape((h, w, 4))
        bgr = cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)

        det = PoseDetector(video_mode=False)
        anchors = det.detect_static(bgr)
        det.close()

        if anchors is not None and anchors.valid:
            self._set_anchor_quad(anchors.pts, auto=True)
            self._status_label.setText(
                f"Pose detected  vis={anchors.visibility:.2f}  "
                f"AR={quad_aspect_ratio(anchors.pts):.2f}"
            )
        else:
            reason = "No pose detected"
            if anchors is not None:
                reason = f"Low visibility ({anchors.visibility:.2f})"
            QMessageBox.information(
                self, "Auto-detect failed",
                f"{reason}.\nUse 'Set Anchors Manually' to click the 4 landmarks."
            )
            self._status_label.setText(f"Auto-detect failed: {reason}")

    def _start_manual_anchor(self):
        self._anchor_mode = AnchorClickMode(self._on_manual_anchors_complete)
        self._canvas.clear_overlay()
        self._canvas.set_mode(MODE_ANCHOR)
        prompt = f"Click {self._anchor_mode.LABELS[0]}"
        self._status_label.setText(prompt)

    def _on_anchor_clicked(self, x: float, y: float):
        if self._anchor_mode is None:
            return
        # Draw a temporary cross at the clicked position
        from massage_path_tool.ui.overlay import PointItem
        item = PointItem(x, y, f"A{len(self._anchor_mode._pts)+1}", radius=4)
        self._canvas._scene.addItem(item)
        prompt = self._anchor_mode.click(x, y)
        if prompt:
            self._status_label.setText(prompt)
        else:
            self._anchor_mode = None
            self._canvas.set_mode(MODE_VIEW)

    def _on_manual_anchors_complete(self, pts: np.ndarray):
        self._set_anchor_quad(pts, auto=False)
        self._status_label.setText(
            f"Manual anchors set  AR={quad_aspect_ratio(pts):.2f}"
        )

    def _set_anchor_quad(self, pts: np.ndarray, auto: bool):
        self._H_norm = compute_h_norm(pts)
        ar = quad_aspect_ratio(pts)
        self._session["calibration_aspect_ratio"] = float(ar)
        self._session["anchor_landmarks"] = {
            "left_shoulder":  {"px": float(pts[0][0]), "py": float(pts[0][1]), "u": 0.0, "v": 0.0},
            "right_shoulder": {"px": float(pts[1][0]), "py": float(pts[1][1]), "u": 1.0, "v": 0.0},
            "right_hip":      {"px": float(pts[2][0]), "py": float(pts[2][1]), "u": 1.0, "v": 1.0},
            "left_hip":       {"px": float(pts[3][0]), "py": float(pts[3][1]), "u": 0.0, "v": 1.0},
        }
        self._canvas.set_anchor_quad(pts)
        self._act_point.setEnabled(True)
        self._act_path.setEnabled(True)
        self._act_save.setEnabled(True)
        src = "auto" if auto else "manual"
        self._status_label.setText(
            f"Anchors ({src})  AR={ar:.2f}  ready for authoring"
        )
        # If a session was loaded before anchors were available, re-project now.
        if self._pending_reproject:
            self._pending_reproject = False
            self._canvas.clear_overlay()
            self._reproject_session()

    # --- authoring ---------------------------------------------------------

    def _on_point_placed(self, x: float, y: float):
        if self._H_norm is None:
            return
        uv = apply_homography(self._H_norm, np.float32([[x, y]]))
        u, v = float(uv[0, 0]), float(uv[0, 1])
        label, ok = QInputDialog.getText(
            self, "Point Label", "Name:", text=f"p{len(self._session['discrete_points'])+1}"
        )
        if not ok or not label:
            return
        self._session["discrete_points"].append(
            {"id": label, "label": label, "u": u, "v": v,
             "x_stitch": float(x), "y_stitch": float(y)}
        )
        self._canvas.add_point_item(x, y, label)
        self._update_status_counts()

    def _on_path_finished(self, raw_pts: list):
        """User finished a point-click path (Enter or right-click)."""
        if self._H_norm is None or len(raw_pts) < 2:
            return
        pts = np.array(raw_pts, dtype=np.float64)
        try:
            smoothed = bezier_through_points(pts, n_out=200, tension=0.5)
        except ValueError as e:
            QMessageBox.warning(self, "Path too short", str(e))
            return
        uvs = apply_homography(self._H_norm, smoothed.astype(np.float32))
        label, ok = QInputDialog.getText(
            self, "Path Label", "Name:", text=f"path{len(self._session['paths'])+1}"
        )
        if not ok or not label:
            return
        self._session["paths"].append({
            "id": label,
            "label": label,
            "points": [
                {
                    "u": float(uvs[i, 0]),
                    "v": float(uvs[i, 1]),
                    "x_stitch": float(smoothed[i, 0]),
                    "y_stitch": float(smoothed[i, 1]),
                }
                for i in range(len(uvs))
            ],
        })
        self._canvas.add_path_item(smoothed)
        self._update_status_counts()

    def _on_path_cancelled(self):
        """User cancelled the current path (Escape)."""
        self._status_label.setText("Path cancelled")

    def _update_status_counts(self):
        n_pts = len(self._session["discrete_points"])
        n_paths = len(self._session["paths"])
        self._status_label.setText(
            f"Points: {n_pts}  |  Paths: {n_paths}"
        )

    # --- session I/O -------------------------------------------------------

    def _save_session(self):
        default = "massage_session.json"
        if self._session.get("source_image"):
            src = Path(self._session["source_image"])
            default = str(src.parent / "massage_session.json")
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Session", default, "JSON (*.json)"
        )
        if path:
            session_io.save_session(self._session, path)
            self._status_label.setText(f"Saved: {path}")
            self._act_start_live.setEnabled(
                bool(self._session["discrete_points"] or self._session["paths"])
            )

    def _load_session(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Session", "", "JSON (*.json)"
        )
        if path:
            self._do_load_session(path)

    def _do_load_session(self, path: str):
        try:
            self._session = session_io.load_session(path)
        except (FileNotFoundError, ValueError) as e:
            QMessageBox.critical(self, "Load Error", str(e))
            return
        self._status_label.setText(f"Loaded: {path}")
        # If an image is already open and anchors are set, re-project now.
        # Otherwise flag for reprojection once anchors are available.
        if self._canvas._bg_item is not None:
            if self._H_norm is not None:
                self._canvas.clear_overlay()
                self._reproject_session()
            else:
                self._pending_reproject = True
        self._act_save.setEnabled(True)
        self._act_start_live.setEnabled(
            bool(self._session["discrete_points"] or self._session["paths"])
        )

    def _reproject_session(self):
        """Re-draw all saved points/paths onto the current image.

        Uses the inverse of the CURRENT image's H_norm (i.e. H_proj for the
        currently loaded image), NOT the reference anchor pixels stored in the
        session file. The session only stores UV coordinates which are
        image-agnostic; reprojection must use the current image's geometry.
        """
        if self._H_norm is None:
            return
        # H_proj = H_norm^-1  maps unit square -> current image pixels
        H_proj = np.linalg.inv(self._H_norm)
        for p in self._session["discrete_points"]:
            uv = np.float32([[p["u"], p["v"]]])
            px = apply_homography(H_proj, uv)[0]
            self._canvas.add_point_item(float(px[0]), float(px[1]), p["label"])
        for path in self._session["paths"]:
            uvs = np.array([[pt["u"], pt["v"]] for pt in path["points"]], dtype=np.float32)
            pxs = apply_homography(H_proj, uvs)
            self._canvas.add_path_item(pxs)
        self._update_status_counts()

    # --- live feed ---------------------------------------------------------

    def _start_live(self):
        if self._worker is not None:
            return
        self._worker = LiveWorker(camera_index=0, session=self._session)
        self._worker.frame_ready.connect(self._canvas.update_live_frame)
        self._worker.projection_ready.connect(self._on_projection)
        self._worker.pose_invalid.connect(self._on_pose_invalid)
        self._worker.pose_lost.connect(self._on_pose_lost)
        self._worker.start()
        self._act_start_live.setEnabled(False)
        self._act_stop_live.setEnabled(True)
        self._status_label.setText("Live feed running")

    def _stop_live(self):
        if self._worker is not None:
            self._worker.stop()
            self._worker = None
        self._act_start_live.setEnabled(True)
        self._act_stop_live.setEnabled(False)
        self._canvas.hide_invalid_banner()
        self._status_label.setText("Live feed stopped")

    def _on_projection(self, anchor_pts, projected_px):
        self._canvas.hide_invalid_banner()
        self._canvas.clear_overlay()
        self._canvas.set_anchor_quad(anchor_pts)
        for i, (x, y) in enumerate(projected_px):
            # Distinguish discrete points from path points by session structure
            # For now, draw all as small dots; paths are drawn as polyline
            pass
        # Re-project paths as polylines
        idx = 0
        for p in self._session["discrete_points"]:
            self._canvas.add_point_item(float(projected_px[idx][0]), float(projected_px[idx][1]), p["label"])
            idx += 1
        for path in self._session["paths"]:
            n = len(path["points"])
            if n > 0:
                self._canvas.add_path_item(projected_px[idx:idx+n])
                idx += n

    def _on_pose_invalid(self, reason: str):
        self._canvas.show_invalid_banner("INVALID POSE GEOMETRY")
        QTimer.singleShot(2000, self._canvas.hide_invalid_banner)
        self._status_label.setText(f"INVALID: {reason}")

    def _on_pose_lost(self):
        self._canvas.hide_invalid_banner()
        self._status_label.setText("Pose lost")

    def _on_mode_changed(self, mode: str):
        self._status_label.setText(f"Mode: {mode}")

    def closeEvent(self, event):
        self._stop_live()
        event.accept()
