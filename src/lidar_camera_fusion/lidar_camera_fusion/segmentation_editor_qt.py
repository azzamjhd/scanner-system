#!/usr/bin/env python3
"""
segmentation_editor_qt
======================
pyqtgraph-based interactive polygon segmentation editor for
manual_segmentation_node. Replaces the old matplotlib editor for far better
performance on large clouds (10k+ points stay fluid).

The node hands us (xyz, rgb, labels, region_labels, callbacks) and we run a
modal Qt editor. Labeling semantics are identical to the matplotlib version:
- top-down X-Y view (X right, Y up), equal aspect
- pick a label, draw a polygon, every point inside is tagged (last-wins)
- Undo last, New label, Publish + save
"""
from typing import Callable, List, Optional, Tuple

import numpy as np

# Force the PyQt5 binding for consistency with the rest of this workspace
# (mission_control_gui.py and scanner_gui both use PyQt5). pyqtgraph honours
# PYQTGRAPH_QT_LIB and will also reuse an already-imported binding; we set both
# belt-and-suspenders so we don't accidentally land on the apt-pulled PyQt6,
# whose scoped enums (Qt.ItemDataRole.UserRole etc.) differ from PyQt5's.
import os as _os
_os.environ.setdefault("PYQTGRAPH_QT_LIB", "PyQt5")
import PyQt5.QtCore as _force_pyqt5  # noqa: F401

from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
import pyqtgraph as pg


# Binding-agnostic enum handles. PyQt5 exposes Qt.UserRole / Qt.LeftButton
# directly; PyQt6 nests them under Qt.ItemDataRole / Qt.MouseButton. Resolve
# once here so the rest of the file doesn't care which binding loaded.
def _enum(root, scoped_cls, name):
    obj = getattr(root, scoped_cls, None)
    if obj is not None and hasattr(obj, name):
        return getattr(obj, name)
    return getattr(root, name)

_USER_ROLE   = _enum(QtCore.Qt, "ItemDataRole", "UserRole")
_LEFT_BTN    = _enum(QtCore.Qt, "MouseButton", "LeftButton")
_RIGHT_BTN   = _enum(QtCore.Qt, "MouseButton", "RightButton")

# QShortcut moved from QtWidgets (PyQt5) to QtGui (PyQt6).
_QShortcut = getattr(QtWidgets, "QShortcut", None) or getattr(QtGui, "QShortcut")


def _qcolor(rgb01: Tuple[float, float, float], alpha: int = 255) -> QtGui.QColor:
    r, g, b = rgb01
    return QtGui.QColor(int(r * 255), int(g * 255), int(b * 255), alpha)


class SegmentationEditor(QtWidgets.QMainWindow):
    """Modal-ish pyqtgraph editor. Construct, then show via run().

    Parameters
    ----------
    xyz : (N,3) float32 full cloud (only X-Y used for the view + hit-test)
    base_colors : (N,3) float in [0,1] per-point display color
    labels : (N,) uint8 — mutated in place as the user labels
    region_labels : list[str] — label vocabulary (index+1 == label id)
    make_color : callable(label_id) -> (r,g,b) in [0,1]
    on_publish : callable(polygons) where polygons is
                 list[(label_id, name, vertices Nx2)]; called when user
                 clicks Publish + save.
    title : window title
    """

    def __init__(
        self,
        xyz: np.ndarray,
        base_colors: np.ndarray,
        labels: np.ndarray,
        region_labels: List[str],
        make_color: Callable[[int], Tuple[float, float, float]],
        on_publish: Callable[[List[Tuple[int, str, np.ndarray]]], None],
        title: str = "Manual Segmentation",
        log: Optional[Callable[[str], None]] = None,
    ) -> None:
        super().__init__()
        self.xyz = xyz
        self.xy = xyz[:, :2].astype(np.float64)
        self.base_colors = base_colors.astype(np.float32)
        self.labels = labels
        self.region_labels = region_labels
        self.make_color = make_color
        self.on_publish = on_publish
        self._log = log or (lambda _m: None)

        # Active-label + polygon state
        self.active_label_id = 1
        self.active_label_name = region_labels[0] if region_labels else "region_1"
        # polygons: list of (label_id, name, vertices Nx2, full_mask)
        self.polygons: List[Tuple[int, str, np.ndarray, np.ndarray]] = []

        # Per-point display colors as uint8 RGBA for pyqtgraph brushes
        self._disp_rgba = np.empty((xyz.shape[0], 4), dtype=np.ubyte)
        self._disp_rgba[:, :3] = (self.base_colors[:, :3] * 255).astype(np.ubyte)
        self._disp_rgba[:, 3] = 255

        self.setWindowTitle(title)
        self.resize(1200, 800)
        self._build_ui()
        self._refresh_scatter()

    # ── UI construction ────────────────────────────────────────────────────
    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)

        # Left: the plot
        self.glw = pg.GraphicsLayoutWidget()
        self.plot = self.glw.addPlot()
        self.plot.setAspectLocked(True)          # equal aspect (X-Y in metres)
        self.plot.setLabel("bottom", "X (m, base_link)")
        self.plot.setLabel("left", "Y (m, base_link)")
        self.plot.showGrid(x=True, y=True, alpha=0.2)

        # Scatter of all points. pxMode=True keeps marker size constant in px
        # so it stays fast and readable at any zoom.
        self.scatter = pg.ScatterPlotItem(
            size=3, pen=None, pxMode=True)
        self.plot.addItem(self.scatter)

        # In-progress polygon: a polyline of clicked vertices + markers
        self._draw_pts: List[Tuple[float, float]] = []
        self._draw_line = pg.PlotDataItem(pen=pg.mkPen("w", width=1.5))
        self.plot.addItem(self._draw_line)
        self._draw_markers = pg.ScatterPlotItem(
            size=8, pen=pg.mkPen("w"), brush=pg.mkBrush(255, 255, 255, 180))
        self.plot.addItem(self._draw_markers)

        # Committed polygon overlays live in this list so we can clear them.
        self._overlay_items: List = []

        # Mouse: left-click adds a vertex; right-click / double-click closes.
        self.scatter_proxy = None
        self.plot.scene().sigMouseClicked.connect(self._on_mouse_click)

        root.addWidget(self.glw, stretch=4)

        # Right: control panel
        panel = QtWidgets.QWidget()
        panel.setMaximumWidth(240)
        pv = QtWidgets.QVBoxLayout(panel)

        self.active_lbl = QtWidgets.QLabel()
        self.active_lbl.setWordWrap(True)
        self._update_active_label_text()
        pv.addWidget(self.active_lbl)

        pv.addWidget(QtWidgets.QLabel("Labels:"))
        self.label_list = QtWidgets.QListWidget()
        self.label_list.itemClicked.connect(self._on_label_picked)
        pv.addWidget(self.label_list, stretch=1)
        self._rebuild_label_list()

        btn_finish = QtWidgets.QPushButton("Finish polygon (or right-click)")
        btn_finish.clicked.connect(self._finish_polygon)
        pv.addWidget(btn_finish)

        btn_cancel = QtWidgets.QPushButton("Cancel current polygon (Esc)")
        btn_cancel.clicked.connect(self._cancel_polygon)
        pv.addWidget(btn_cancel)

        btn_undo = QtWidgets.QPushButton("Undo last polygon")
        btn_undo.clicked.connect(self._on_undo)
        pv.addWidget(btn_undo)

        btn_newlbl = QtWidgets.QPushButton("New label…")
        btn_newlbl.clicked.connect(self._on_new_label)
        pv.addWidget(btn_newlbl)

        btn_publish = QtWidgets.QPushButton("Publish + save")
        btn_publish.setStyleSheet(
            "background-color:#3CB371; font-weight:bold; padding:8px;")
        btn_publish.clicked.connect(self._on_publish_clicked)
        pv.addWidget(btn_publish)

        info = QtWidgets.QLabel(
            "Left-click: add vertex\n"
            "Right-click / double-click: close polygon\n"
            "Esc: cancel current\n"
            "Scroll: zoom · drag: pan")
        info.setStyleSheet("color:#888; font-size:10px;")
        info.setWordWrap(True)
        pv.addWidget(info)

        root.addWidget(panel, stretch=0)

        # Esc shortcut to cancel an in-progress polygon
        _QShortcut(QtGui.QKeySequence("Escape"), self,
                   activated=self._cancel_polygon)

    # ── Label panel helpers ────────────────────────────────────────────────
    def _update_active_label_text(self) -> None:
        self.active_lbl.setText(
            f"Active label:\n[{self.active_label_id}] {self.active_label_name}")

    def _rebuild_label_list(self) -> None:
        self.label_list.clear()
        for i, name in enumerate(self.region_labels):
            lid = i + 1
            item = QtWidgets.QListWidgetItem(f"{lid}: {name}")
            item.setData(_USER_ROLE, (lid, name))
            col = _qcolor(self.make_color(lid))
            item.setBackground(QtGui.QBrush(col))
            # readable text on colored bg
            item.setForeground(QtGui.QBrush(QtGui.QColor("black")))
            self.label_list.addItem(item)

    def _on_label_picked(self, item) -> None:
        lid, name = item.data(_USER_ROLE)
        self.active_label_id = int(lid)
        self.active_label_name = str(name)
        self._update_active_label_text()

    # ── Scatter color refresh ──────────────────────────────────────────────
    def _refresh_scatter(self) -> None:
        """Push the current per-point RGBA to the scatter in one call."""
        spots = dict(
            x=self.xy[:, 0], y=self.xy[:, 1],
            brush=[pg.mkBrush(*tuple(c)) for c in self._disp_rgba],
        )
        # Faster path: set data with a single color array via setData kwargs.
        self.scatter.setData(
            x=self.xy[:, 0], y=self.xy[:, 1],
            brush=spots["brush"], size=3, pen=None, pxMode=True)

    def _recolor_points(self, mask: np.ndarray, lid: int) -> None:
        """Recolor only the masked points to label lid's color (in the cached
        RGBA array) and push to the scatter."""
        r, g, b = self.make_color(lid)
        self._disp_rgba[mask, 0] = int(r * 255)
        self._disp_rgba[mask, 1] = int(g * 255)
        self._disp_rgba[mask, 2] = int(b * 255)
        self._disp_rgba[mask, 3] = 255
        self._refresh_scatter()

    def _recolor_all_from_labels(self) -> None:
        """Rebuild all display colors from base + current labels (used by undo)."""
        self._disp_rgba[:, :3] = (self.base_colors[:, :3] * 255).astype(np.ubyte)
        self._disp_rgba[:, 3] = 255
        for li in np.unique(self.labels):
            if li == 0:
                continue
            m = self.labels == li
            r, g, b = self.make_color(int(li))
            self._disp_rgba[m, 0] = int(r * 255)
            self._disp_rgba[m, 1] = int(g * 255)
            self._disp_rgba[m, 2] = int(b * 255)
        self._refresh_scatter()

    # ── Polygon drawing ────────────────────────────────────────────────────
    def _on_mouse_click(self, ev) -> None:
        # Map scene pos -> data coords
        vb = self.plot.vb
        if not self.plot.sceneBoundingRect().contains(ev.scenePos()):
            return
        pt = vb.mapSceneToView(ev.scenePos())
        x, y = float(pt.x()), float(pt.y())

        # Right-click or double-click closes the polygon.
        if ev.button() == _RIGHT_BTN or ev.double():
            self._finish_polygon()
            ev.accept()
            return
        if ev.button() == _LEFT_BTN:
            self._draw_pts.append((x, y))
            self._redraw_inprogress()
            ev.accept()

    def _redraw_inprogress(self) -> None:
        if not self._draw_pts:
            self._draw_line.setData([], [])
            self._draw_markers.setData([], [])
            return
        arr = np.asarray(self._draw_pts, dtype=np.float64)
        # close the loop visually
        loop = np.vstack([arr, arr[:1]]) if arr.shape[0] >= 2 else arr
        self._draw_line.setData(loop[:, 0], loop[:, 1])
        self._draw_markers.setData(arr[:, 0], arr[:, 1])

    def _cancel_polygon(self) -> None:
        self._draw_pts = []
        self._redraw_inprogress()

    def _finish_polygon(self) -> None:
        if len(self._draw_pts) < 3:
            self._log("Polygon needs >=3 vertices")
            self._cancel_polygon()
            return
        verts = np.asarray(self._draw_pts, dtype=np.float64)
        lid = self.active_label_id
        name = self.active_label_name
        mask = self._point_in_poly(verts)
        self.labels[mask] = lid                      # last-wins
        self.polygons.append((lid, name, verts, mask))
        self._recolor_points(mask, lid)
        self._add_overlay(verts, lid)
        self._log(
            f"Polygon committed: label={lid}({name}), "
            f"vertices={verts.shape[0]}, points_labeled={int(mask.sum())}")
        self._cancel_polygon()

    def _point_in_poly(self, verts: np.ndarray) -> np.ndarray:
        """Boolean mask of full-cloud points inside the polygon (X-Y).
        Uses QPolygonF.containsPoint via a vectorized matplotlib-free test."""
        # Ray-casting vectorized over all points.
        x = self.xy[:, 0]
        y = self.xy[:, 1]
        vx = verts[:, 0]
        vy = verts[:, 1]
        n = len(vx)
        inside = np.zeros(x.shape[0], dtype=bool)
        j = n - 1
        for i in range(n):
            xi, yi = vx[i], vy[i]
            xj, yj = vx[j], vy[j]
            cond = ((yi > y) != (yj > y)) & (
                x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi)
            inside ^= cond
            j = i
        return inside

    def _add_overlay(self, verts: np.ndarray, lid: int) -> None:
        col = _qcolor(self.make_color(lid))
        loop = np.vstack([verts, verts[:1]])
        line = pg.PlotDataItem(loop[:, 0], loop[:, 1],
                               pen=pg.mkPen(col, width=2))
        self.plot.addItem(line)
        self._overlay_items.append(line)

    def _rebuild_overlays(self) -> None:
        for it in self._overlay_items:
            self.plot.removeItem(it)
        self._overlay_items.clear()
        for lid, _name, verts, _mask in self.polygons:
            self._add_overlay(verts, lid)

    # ── Buttons ────────────────────────────────────────────────────────────
    def _on_undo(self) -> None:
        if not self.polygons:
            return
        self.polygons.pop()
        # Recompute labels from cached masks (no point-in-poly re-run).
        self.labels[:] = 0
        for lid, _name, _verts, mask in self.polygons:
            self.labels[mask] = lid
        self._recolor_all_from_labels()
        self._rebuild_overlays()

    def _on_new_label(self) -> None:
        text, ok = QtWidgets.QInputDialog.getText(
            self, "New label", "Label name:")
        if not ok or not text:
            return
        name = text.strip().replace(" ", "_")
        if not name:
            return
        if len(self.region_labels) >= 24:
            self._log("Max 24 labels reached — drop one first")
            return
        self.region_labels.append(name)
        self._rebuild_label_list()

    def _on_publish_clicked(self) -> None:
        try:
            # Hand back 3-tuples (lid, name, verts) to match node's signature.
            polys3 = [(lid, name, verts)
                      for (lid, name, verts, _mask) in self.polygons]
            self.on_publish(polys3)
            # Non-modal feedback (a modal QMessageBox would block a headless
            # event loop and is intrusive in normal use).
            n_lbl = int((self.labels > 0).sum())
            self.statusBar().showMessage(
                f"Published + saved — {n_lbl} points labeled across "
                f"{len(polys3)} polygons.", 8000)
            self._log(f"Published + saved ({n_lbl} labeled).")
        except Exception as exc:  # noqa: BLE001
            self._log(f"Publish/save failed: {exc}")
            self.statusBar().showMessage(f"Publish/save FAILED: {exc}", 0)

    def run(self) -> None:
        """Show modally; returns when the window is closed."""
        self.show()
