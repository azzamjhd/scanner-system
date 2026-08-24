import sys
import numpy as np
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore, QtGui
from typing import List, Tuple, Optional

class SegmentationGUI(QtWidgets.QMainWindow):
    def __init__(self, xyz: np.ndarray, rgb: Optional[np.ndarray], labels: np.ndarray,
                 region_labels: List[str], max_display: int = 15000):
        super().__init__()
        self.xyz = xyz
        self.labels = labels
        self.n_full = xyz.shape[0]
        self.region_labels = region_labels
        self.polygons = []  # list of (lid, name, verts, mask)
        
        # UI State
        self.active_label_id = 1
        self.active_label_name = region_labels[0] if region_labels else "region_1"
        self.publish_requested = False
        
        # Display Downsampling (PG can handle more, but cap at 15k for safety)
        if self.n_full > max_display:
            stride = int(np.ceil(self.n_full / max_display))
            self.disp_idx = np.arange(0, self.n_full, stride)
        else:
            self.disp_idx = np.arange(self.n_full)
            
        self.disp_xy = self.xyz[self.disp_idx, :2]
        
        # Base Colors
        if rgb is not None:
            self.base_colors = rgb[self.disp_idx]
        else:
            z = xyz[self.disp_idx, 2]
            zmin, zmax = z.min(), z.max()
            denom = max(zmax - zmin, 1e-6)
            norm_z = (z - zmin) / denom
            cmap = pg.colormap.get('viridis')
            self.base_colors = cmap.map(norm_z, mode='byte')
            
        self.disp_colors = self.base_colors.copy()
        if self.disp_colors.shape[1] == 3:
            alpha = np.full((self.disp_colors.shape[0], 1), 255, dtype=np.uint8)
            self.disp_colors = np.hstack([self.disp_colors, alpha])
            
        self.full_to_disp = np.full(self.n_full, -1, dtype=np.int64)
        self.full_to_disp[self.disp_idx] = np.arange(self.disp_idx.size)
        
        self.init_ui()
        
    def init_ui(self):
        self.setWindowTitle(f"Manual Segmentation - {self.n_full} points")
        self.resize(1200, 800)
        
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)
        
        # Plot
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setAspectLocked(True)
        self.plot_widget.setMouseEnabled(x=True, y=True)
        self.plot_widget.showGrid(x=True, y=True)
        layout.addWidget(self.plot_widget, stretch=4)
        
        # Scatter
        self.scatter = pg.ScatterPlotItem(
            pos=self.disp_xy, 
            brush=self.disp_colors,
            pen=None,
            size=3
        )
        self.plot_widget.addItem(self.scatter)
        
        # ROI for drawing
        self.roi = pg.PolyLineROI([], closed=False, removable=True, movable=False)
        self.plot_widget.addItem(self.roi)
        self.roi.hide()
        
        # View interaction overrides
        self.plot_widget.scene().sigMouseClicked.connect(self.on_mouse_click)
        
        # Sidebar
        sidebar = QtWidgets.QVBoxLayout()
        layout.addLayout(sidebar, stretch=1)
        
        inst = QtWidgets.QLabel("Left-click to add points.\nRight-click to close polygon.")
        sidebar.addWidget(inst)
        
        self.lbl_active = QtWidgets.QLabel(f"Active: {self.active_label_name}")
        self.lbl_active.setStyleSheet("font-weight: bold; font-size: 14px;")
        sidebar.addWidget(self.lbl_active)
        
        self.btn_group = QtWidgets.QButtonGroup()
        self.layout_btns = QtWidgets.QVBoxLayout()
        self.refresh_label_buttons()
        sidebar.addLayout(self.layout_btns)
        
        sidebar.addStretch()
        
        btn_undo = QtWidgets.QPushButton("Undo Last")
        btn_undo.clicked.connect(self.undo)
        sidebar.addWidget(btn_undo)
        
        btn_new = QtWidgets.QPushButton("New Label...")
        btn_new.clicked.connect(self.new_label)
        sidebar.addWidget(btn_new)
        
        btn_pub = QtWidgets.QPushButton("Publish + Save")
        btn_pub.setStyleSheet("background-color: #3CB371; color: white;")
        btn_pub.clicked.connect(self.publish)
        sidebar.addWidget(btn_pub)

    def refresh_label_buttons(self):
        # Clear old
        while self.layout_btns.count():
            item = self.layout_btns.takeAt(0)
            w = item.widget()
            if w: w.deleteLater()
            
        for i, name in enumerate(self.region_labels, start=1):
            btn = QtWidgets.QPushButton(f"[{i}] {name}")
            btn.setCheckable(True)
            if i == self.active_label_id:
                btn.setChecked(True)
            
            # Create closure for lid/name binding
            def make_cb(lid=i, lname=name):
                def cb():
                    self.active_label_id = lid
                    self.active_label_name = lname
                    self.lbl_active.setText(f"Active: {lname}")
                return cb
                
            btn.clicked.connect(make_cb())
            self.btn_group.addButton(btn)
            self.layout_btns.addWidget(btn)
            
    def make_color(self, lid: int):
        cmap = pg.colormap.get('tab20')
        c = cmap.map(lid % 20, mode='byte')
        return c

    def on_mouse_click(self, ev):
        if ev.button() == QtCore.Qt.MouseButton.LeftButton:
            pos = self.plot_widget.plotItem.vb.mapSceneToView(ev.scenePos())
            if not self.roi.isVisible():
                self.roi.clearPoints()
                self.roi.show()
            self.roi.addFreeHandle((pos.x(), pos.y()))
            ev.accept()
        elif ev.button() == QtCore.Qt.MouseButton.RightButton and self.roi.isVisible():
            # Close polygon
            handles = self.roi.getHandles()
            if len(handles) > 2:
                pts = [[h.pos().x(), h.pos().y()] for h in handles]
                self.commit_polygon(np.array(pts))
            self.roi.hide()
            self.roi.clearPoints()
            ev.accept()

    def commit_polygon(self, verts: np.ndarray):
        from matplotlib.path import Path
        mpl_path = Path(verts)
        inside = mpl_path.contains_points(self.xyz[:, :2])
        self.labels[inside] = self.active_label_id
        
        self.polygons.append((self.active_label_id, self.active_label_name, verts, inside))
        
        # Redraw
        poly_item = pg.PlotDataItem(np.vstack([verts, verts[:1]]), pen=pg.mkPen(self.make_color(self.active_label_id), width=2))
        self.plot_widget.addItem(poly_item)
        
        self.recolor_all()
        
    def recolor_all(self):
        disp_labels = self.labels[self.disp_idx]
        self.disp_colors[:, :3] = self.base_colors[:, :3]
        for li in np.unique(disp_labels):
            if li == 0: continue
            mask = disp_labels == li
            self.disp_colors[mask, :3] = self.make_color(li)[:3]
            
        self.scatter.setBrush(self.disp_colors)

    def undo(self):
        if not self.polygons: return
        self.polygons.pop()
        
        # Remove top plot item (roi is 1, scatter is 0)
        items = self.plot_widget.plotItem.items
        if len(items) > 2:
            self.plot_widget.removeItem(items[-1])
            
        self.labels[:] = 0
        for lid, _, _, full_mask in self.polygons:
            self.labels[full_mask] = lid
            
        self.recolor_all()

    def new_label(self):
        text, ok = QtWidgets.QInputDialog.getText(self, 'New Label', 'Name:')
        if ok and text:
            name = text.strip().replace(" ", "_")
            if name:
                self.region_labels.append(name)
                self.refresh_label_buttons()

    def publish(self):
        self.publish_requested = True
        self.close()

def run_pyqtgraph_editor(xyz, rgb, labels, region_labels, max_display):
    app = QtWidgets.QApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(sys.argv)
    
    gui = SegmentationGUI(xyz, rgb, labels, region_labels, max_display)
    gui.show()
    app.exec_()
    return gui.publish_requested, gui.polygons
