"""Drawing canvas: QGraphicsView + QGraphicsScene with interaction modes.

Modes:
  MODE_VIEW   — pan/zoom only
  MODE_POINT  — click places a discrete massage point
  MODE_PATH   — click adds control points; right-click or Enter finishes the
                path and emits path_finished. Escape cancels.
  MODE_ANCHOR — click 4 anchor landmarks in order (manual calibration fallback)

The canvas only captures raw pixel coordinates and emits signals; all
homography / Bezier math lives in core/ and is wired up by MainWindow.
"""
from __future__ import annotations

import numpy as np
from PyQt6.QtCore import Qt, pyqtSignal, QPointF, QRectF
from PyQt6.QtGui import QPixmap, QImage, QPainter, QPen, QColor
from PyQt6.QtWidgets import (
    QGraphicsView, QGraphicsScene, QGraphicsPixmapItem,
    QGraphicsPathItem, QGraphicsEllipseItem,
)

from massage_path_tool.ui import overlay

MODE_VIEW = "view"
MODE_POINT = "point"
MODE_PATH = "path"
MODE_ANCHOR = "anchor"


class CanvasView(QGraphicsView):
    """Interactive image canvas for authoring and live projection."""

    point_placed = pyqtSignal(float, float)       # image pixel x, y
    path_finished = pyqtSignal(list)              # list of (x, y) control points
    path_cancelled = pyqtSignal()                   # user cancelled current path
    anchor_clicked = pyqtSignal(float, float)     # one manual anchor click
    mode_changed = pyqtSignal(str)

    def __init__(self, parent=None):
        self._scene = QGraphicsScene(parent)
        super().__init__(self._scene, parent)
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setTransformationAnchor(
            QGraphicsView.ViewportAnchor.AnchorUnderMouse
        )
        self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)

        self._mode = MODE_VIEW
        self._path_control_pts: list[tuple[float, float]] = []
        self._path_temp_line: QGraphicsPathItem | None = None
        self._path_temp_dots: list[QGraphicsEllipseItem] = []

        self._bg_item: QGraphicsPixmapItem | None = None
        self._anchor_item = None
        self._banner_item = None
        self._point_items: list = []
        self._path_items: list = []

    # --- mode --------------------------------------------------------------

    @property
    def mode(self) -> str:
        return self._mode

    def set_mode(self, mode: str) -> None:
        # Cancel any in-progress path when switching modes
        if self._mode == MODE_PATH and mode != MODE_PATH:
            self._cancel_path()
        self._mode = mode
        if mode == MODE_VIEW:
            self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
        else:
            self.setDragMode(QGraphicsView.DragMode.NoDrag)
        self.mode_changed.emit(mode)

    # --- image -------------------------------------------------------------

    def load_image(self, path: str) -> bool:
        pixmap = QPixmap(path)
        if pixmap.isNull():
            return False
        self._reset_scene()
        self._bg_item = self._scene.addPixmap(pixmap)
        self._scene.setSceneRect(QRectF(pixmap.rect()))
        self.fitInView(self._scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)
        return True

    def _reset_scene(self) -> None:
        self._scene.clear()
        self._bg_item = None
        self._anchor_item = None
        self._banner_item = None
        self._point_items = []
        self._path_items = []
        self._path_control_pts = []
        self._path_temp_line = None
        self._path_temp_dots = []

    # --- overlay items -----------------------------------------------------

    def set_anchor_quad(self, pixel_pts: np.ndarray) -> None:
        if self._anchor_item is not None:
            self._scene.removeItem(self._anchor_item)
        self._anchor_item = overlay.make_anchor_quad_item(pixel_pts)
        self._scene.addItem(self._anchor_item)

    def add_point_item(self, x: float, y: float, label: str) -> None:
        item = overlay.PointItem(x, y, label)
        self._scene.addItem(item)
        self._point_items.append(item)

    def add_path_item(self, pixel_pts: np.ndarray) -> None:
        item = overlay.make_path_item(pixel_pts)
        self._scene.addItem(item)
        self._path_items.append(item)

    def clear_overlay(self) -> None:
        """Remove drawn points/paths/anchor but keep the background image."""
        for it in self._point_items + self._path_items:
            self._scene.removeItem(it)
        self._point_items = []
        self._path_items = []
        if self._anchor_item is not None:
            self._scene.removeItem(self._anchor_item)
            self._anchor_item = None
        self._cancel_path()

    # --- live feed ---------------------------------------------------------

    def update_live_frame(self, pixmap: QPixmap) -> None:
        if self._bg_item is None:
            self._bg_item = self._scene.addPixmap(pixmap)
            self._scene.setSceneRect(QRectF(pixmap.rect()))
            self.fitInView(
                self._scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio
            )
        else:
            self._bg_item.setPixmap(pixmap)

    def show_invalid_banner(self, text: str = "INVALID POSE GEOMETRY") -> None:
        self.hide_invalid_banner()
        self._banner_item = overlay.make_invalid_banner_item(
            self._scene.sceneRect(), text
        )
        self._scene.addItem(self._banner_item)

    def hide_invalid_banner(self) -> None:
        if self._banner_item is not None:
            self._scene.removeItem(self._banner_item)
            self._banner_item = None

    # --- path building (point-click) ---------------------------------------

    def _add_path_control_point(self, x: float, y: float) -> None:
        """Add a control point to the path being built."""
        self._path_control_pts.append((x, y))
        # Add a small dot at the control point
        dot = QGraphicsEllipseItem(x - 3, y - 3, 6, 6)
        dot.setPen(QPen(QColor("orange"), 1))
        dot.setBrush(QColor("orange"))
        self._scene.addItem(dot)
        self._path_temp_dots.append(dot)
        self._update_path_temp_line()

    def _update_path_temp_line(self) -> None:
        """Redraw the temporary polyline connecting control points."""
        if self._path_temp_line is not None:
            self._scene.removeItem(self._path_temp_line)
            self._path_temp_line = None
        if len(self._path_control_pts) < 2:
            return
        from PyQt6.QtGui import QPainterPath
        path = QPainterPath()
        path.moveTo(*self._path_control_pts[0])
        for pt in self._path_control_pts[1:]:
            path.lineTo(*pt)
        self._path_temp_line = QGraphicsPathItem(path)
        self._path_temp_line.setPen(QPen(QColor("orange"), 2, Qt.PenStyle.DashLine))
        self._scene.addItem(self._path_temp_line)

    def _finish_path(self) -> None:
        """Emit the finished control points and clear temp items."""
        if len(self._path_control_pts) >= 2:
            pts = list(self._path_control_pts)
            self._clear_path_temp()
            self.path_finished.emit(pts)
        else:
            self._cancel_path()

    def _cancel_path(self) -> None:
        """Cancel the current path being built."""
        self._clear_path_temp()
        if self._path_control_pts:
            self._path_control_pts = []
            self.path_cancelled.emit()

    def _clear_path_temp(self) -> None:
        """Remove temporary path visuals (dots + line)."""
        for dot in self._path_temp_dots:
            self._scene.removeItem(dot)
        self._path_temp_dots = []
        if self._path_temp_line is not None:
            self._scene.removeItem(self._path_temp_line)
            self._path_temp_line = None
        self._path_control_pts = []

    # --- mouse / wheel / keyboard ------------------------------------------

    def wheelEvent(self, event):
        factor = 1.25 if event.angleDelta().y() > 0 else 0.8
        self.scale(factor, factor)

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            sp = self.mapToScene(event.position().toPoint())
            if self._mode == MODE_POINT:
                self.point_placed.emit(sp.x(), sp.y())
                return
            if self._mode == MODE_ANCHOR:
                self.anchor_clicked.emit(sp.x(), sp.y())
                return
            if self._mode == MODE_PATH:
                self._add_path_control_point(sp.x(), sp.y())
                return
        elif event.button() == Qt.MouseButton.RightButton:
            if self._mode == MODE_PATH and self._path_control_pts:
                self._finish_path()
                return
        super().mousePressEvent(event)

    def keyPressEvent(self, event):
        if self._mode == MODE_PATH:
            if event.key() in (Qt.Key.Key_Return, Qt.Key.Key_Enter):
                if self._path_control_pts:
                    self._finish_path()
                return
            elif event.key() == Qt.Key.Key_Escape:
                self._cancel_path()
                return
        super().keyPressEvent(event)
