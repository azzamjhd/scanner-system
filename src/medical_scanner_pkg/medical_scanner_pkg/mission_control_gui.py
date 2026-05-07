"""
Mission Control Scanner GUI — PyQt5 port

Drop-in widget for ROS2 packages. The MissionControlGUI widget exposes a
clean API your ROS2 node can call to push telemetry / log lines, and emits
Qt signals when the operator interacts (start scan, stop scan, send move,
e-stop, etc.). It does NOT import rclpy directly — wire it from your node.

USAGE (in your ROS2 package, e.g. scanner_gui/scanner_gui_node.py):

    import sys, rclpy
    from rclpy.node import Node
    from PyQt5.QtWidgets import QApplication
    from scanner_gui.mission_control_gui import MissionControlGUI

    class ScannerGuiNode(Node):
        def __init__(self, gui):
            super().__init__('scanner_gui')
            self.gui = gui
            # subscribe to your topics, then call gui.set_position(x), etc.
            gui.start_scan_requested.connect(self.on_start_scan)
            gui.stop_scan_requested.connect(self.on_stop_scan)
            gui.estop_requested.connect(self.on_estop)
            gui.move_requested.connect(self.on_move)        # (target, speed, accel)
            gui.scan_settings_changed.connect(self.on_settings)
            # ... etc

    def main():
        rclpy.init()
        app = QApplication(sys.argv)
        gui = MissionControlGUI()
        node = ScannerGuiNode(gui)
        gui.show()
        # spin ROS2 in a QTimer so Qt event loop stays responsive
        from PyQt5.QtCore import QTimer
        spin_timer = QTimer()
        spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
        spin_timer.start(10)
        sys.exit(app.exec_())

The widget renders identically dark or light — set `theme='dark'` or `'light'`.
Compact density mode shrinks paddings/font sizes for embedded touch panels.
"""

import math
import time
import random
from datetime import datetime

from PyQt5.QtCore import (
    Qt, QTimer, QSize, QRectF, QPointF, pyqtSignal, pyqtProperty
)
from PyQt5.QtGui import (
    QPainter, QColor, QPen, QBrush, QFont, QFontDatabase, QLinearGradient,
    QPainterPath, QPalette, QRadialGradient
)
from PyQt5.QtWidgets import (
    QWidget, QFrame, QLabel, QPushButton, QLineEdit, QCheckBox, QSpinBox,
    QDoubleSpinBox, QToolButton, QSizePolicy, QHBoxLayout, QVBoxLayout,
    QGridLayout, QScrollArea, QPlainTextEdit, QApplication, QStyle,
    QStyleOption
)


# ───────────────────────── PALETTE ─────────────────────────

PALETTES = {
    'dark': dict(
        bg='#0a0d10', panel='#10151a', panel_hi='#141a21',
        border='#1c242e', border_hi='#2a3540',
        ink='#e6e8eb', dim='#7a8492', faint='#52606e',
        accent='#5fffb0', accent_ink='#001a0e',
        warn='#ffc857', err='#ff5d5d',
        blue='#5fb7ff', purple='#b48bff',
        log_bg='#070a0d',
    ),
    'light': dict(
        bg='#eef1f4', panel='#ffffff', panel_hi='#f7f9fb',
        border='#dde1e5', border_hi='#c4cad1',
        ink='#1a1f24', dim='#6b727a', faint='#9aa3ad',
        accent='#0a8a4a', accent_ink='#ffffff',
        warn='#a8721a', err='#c4282c',
        blue='#1f6fcf', purple='#7a4fcf',
        log_bg='#fafbfc',
    ),
}


# ───────────────────────── HELPERS ─────────────────────────

def _families():
    """Return installed font families — compatible with Qt5 instance and Qt6 static API."""
    db = QFontDatabase()
    try:
        return db.families()
    except TypeError:
        return QFontDatabase.families()


def _font(family='Inter', size=11, weight=QFont.Normal, mono=False):
    """Return a QFont — falls back gracefully if Inter/JetBrains Mono not installed."""
    installed = _families()
    if mono:
        f = QFont('JetBrains Mono', size)
        f.setStyleHint(QFont.Monospace)
        if 'JetBrains Mono' not in installed:
            for cand in ('Fira Code', 'Source Code Pro', 'DejaVu Sans Mono',
                         'Liberation Mono', 'Menlo', 'Consolas', 'monospace'):
                if cand in installed:
                    f = QFont(cand, size); f.setStyleHint(QFont.Monospace); break
    else:
        f = QFont(family, size)
        if family not in installed:
            for cand in ('Helvetica Neue', 'Helvetica', 'Arial',
                         'Cantarell', 'Ubuntu', 'Sans Serif'):
                if cand in installed:
                    f = QFont(cand, size); break
    f.setWeight(weight)
    return f


# ───────────────────────── SECTION FRAME ─────────────────────────

class SectionFrame(QFrame):
    """Bordered panel with a header strip (accent dot + uppercase title + optional right widget)."""

    def __init__(self, title, accent=None, right_widget=None, palette_dict=None,
                 compact=False, parent=None):
        super().__init__(parent)
        self._pal = palette_dict
        self._title = title
        self._accent = accent
        self.setObjectName('SectionFrame')

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        # header
        self._header = QFrame(self)
        self._header.setObjectName('SectionHeader')
        h = QHBoxLayout(self._header)
        pad = 10 if compact else 14
        vpad = 6 if compact else 8
        h.setContentsMargins(pad, vpad, pad, vpad)
        h.setSpacing(8)

        # accent dot
        self._dot = QLabel('●', self._header)
        self._dot.setObjectName('AccentDot')
        if accent:
            self._dot.setStyleSheet(f'color: {accent}; font-size: 8px;')
        else:
            self._dot.hide()

        self._title_lbl = QLabel(title.upper(), self._header)
        self._title_lbl.setObjectName('SectionTitle')

        h.addWidget(self._dot)
        h.addWidget(self._title_lbl)
        h.addStretch(1)
        if right_widget is not None:
            h.addWidget(right_widget)
        self._right_widget = right_widget

        outer.addWidget(self._header)

        # body
        self._body = QFrame(self)
        self._body.setObjectName('SectionBody')
        body_lay = QVBoxLayout(self._body)
        body_lay.setContentsMargins(pad, pad, pad, pad)
        body_lay.setSpacing(10)
        outer.addWidget(self._body, 1)

        self.body_layout = body_lay

        self.apply_palette(palette_dict)

    def set_right_widget(self, w):
        if self._right_widget is not None:
            self._right_widget.setParent(None)
        self._right_widget = w
        self._header.layout().addWidget(w)

    def apply_palette(self, p):
        self._pal = p
        self.setStyleSheet(f"""
            QFrame#SectionFrame {{
                background: {p['panel']};
                border: 1px solid {p['border']};
                border-radius: 4px;
            }}
            QFrame#SectionHeader {{
                background: {p['panel_hi']};
                border: none;
                border-bottom: 1px solid {p['border']};
            }}
            QFrame#SectionBody {{ background: transparent; border: none; }}
            QLabel#SectionTitle {{
                color: {p['dim']};
                font-family: 'JetBrains Mono', monospace;
                font-size: 10px;
                font-weight: 600;
                letter-spacing: 1.4px;
            }}
        """)


# ───────────────────────── BUTTONS ─────────────────────────

def styled_button(label, variant='default', palette=None, compact=False,
                  on_click=None, parent=None):
    p = palette
    btn = QPushButton(label, parent)
    variants = {
        'default': dict(bg=p['border'],   fg=p['ink'],     bd=p['border'], hbg=p['border_hi']),
        'primary': dict(bg=p['accent'],   fg=p['accent_ink'], bd=p['accent'], hbg=p['accent']),
        'danger':  dict(bg='#2a1416',     fg=p['err'],     bd='#5a2024', hbg='#3a1c1f'),
        'warn':    dict(bg='#2a2010',     fg=p['warn'],    bd='#5a4520', hbg='#3a2c14'),
        'ghost':   dict(bg='transparent', fg=p['ink'],     bd=p['border'], hbg=p['panel_hi']),
        'blue':    dict(bg='#10212e',     fg=p['blue'],    bd='#1f4665', hbg='#173049'),
        'purple':  dict(bg='#1d1530',     fg=p['purple'],  bd='#3d2a5e', hbg='#241a3a'),
    }
    # light-mode tweaks for tinted variants
    if p is PALETTES['light']:
        variants['danger'] = dict(bg='#fde2e2', fg=p['err'], bd='#f0a0a0', hbg='#f8c8c8')
        variants['warn']   = dict(bg='#fef3d6', fg=p['warn'], bd='#e8c87a', hbg='#fce8b8')
        variants['blue']   = dict(bg='#dbe9f7', fg=p['blue'], bd='#a8caec', hbg='#c5dcef')
        variants['purple'] = dict(bg='#e7dcf7', fg=p['purple'], bd='#c4adde', hbg='#dbcbed')
        variants['default']= dict(bg='#e8ebef', fg=p['ink'], bd=p['border'], hbg='#dde1e5')

    v = variants.get(variant, variants['default'])
    pad = '5px 10px' if compact else '7px 12px'
    fs = 10 if compact else 11
    btn.setStyleSheet(f"""
        QPushButton {{
            background: {v['bg']};
            color: {v['fg']};
            border: 1px solid {v['bd']};
            border-radius: 3px;
            padding: {pad};
            font-family: 'JetBrains Mono', monospace;
            font-size: {fs}px;
            font-weight: 600;
            letter-spacing: 0.4px;
            text-transform: uppercase;
        }}
        QPushButton:hover  {{ background: {v['hbg']}; }}
        QPushButton:disabled {{ color: {p['faint']}; background: transparent; border-color: {p['border']}; }}
    """)
    btn.setCursor(Qt.PointingHandCursor)
    if on_click:
        btn.clicked.connect(on_click)
    return btn


# ───────────────────────── STAT TILE ─────────────────────────

class StatTile(QFrame):
    """Big numeric readout: LABEL + VALUE + UNIT."""

    def __init__(self, label, value='0', unit='', accent=None, big=True,
                 palette=None, compact=False, parent=None):
        super().__init__(parent)
        self._pal = palette
        self._accent = accent
        self._big = big
        self._compact = compact

        self.setObjectName('StatTile')
        lay = QVBoxLayout(self)
        pad_v = 8 if compact else 10
        pad_h = 10 if compact else 12
        lay.setContentsMargins(pad_h, pad_v, pad_h, pad_v)
        lay.setSpacing(4)

        self._label = QLabel(label.upper(), self)
        self._label.setObjectName('StatLabel')

        self._row = QHBoxLayout()
        self._row.setSpacing(4)
        self._row.setContentsMargins(0, 0, 0, 0)

        self._value = QLabel(str(value), self)
        self._value.setObjectName('StatValue')
        self._unit = QLabel(unit, self)
        self._unit.setObjectName('StatUnit')

        self._row.addWidget(self._value)
        self._row.addWidget(self._unit)
        self._row.addStretch(1)

        lay.addWidget(self._label)
        lay.addLayout(self._row)

        self.apply_palette(palette)

    def setValue(self, value, unit=None):
        self._value.setText(str(value))
        if unit is not None:
            self._unit.setText(unit)

    def setAccent(self, color):
        self._accent = color
        self.apply_palette(self._pal)

    def apply_palette(self, p):
        self._pal = p
        big_size = 22 if self._compact else 28
        small_size = 16 if self._compact else 20
        vsize = big_size if self._big else small_size
        accent = self._accent or p['ink']
        self.setStyleSheet(f"""
            QFrame#StatTile {{
                background: {p['log_bg']};
                border: 1px solid {p['border']};
                border-radius: 4px;
            }}
            QLabel#StatLabel {{
                color: {p['dim']};
                font-family: 'JetBrains Mono', monospace;
                font-size: 9px;
                font-weight: 600;
                letter-spacing: 1.2px;
            }}
            QLabel#StatValue {{
                color: {accent};
                font-family: 'JetBrains Mono', monospace;
                font-size: {vsize}px;
                font-weight: 700;
            }}
            QLabel#StatUnit {{
                color: {p['dim']};
                font-family: 'JetBrains Mono', monospace;
                font-size: 11px;
                font-weight: 500;
            }}
        """)


# ───────────────────────── PARAM FIELD (label + spinbox + unit) ─────────────────────────

class ParamField(QWidget):
    """Compact labeled number input with up/down steppers and a unit suffix."""
    valueChanged = pyqtSignal(float)

    def __init__(self, label, value=0.0, step=1.0, suffix='', minimum=-1e6,
                 maximum=1e6, decimals=2, hint=None, palette=None, compact=False,
                 parent=None):
        super().__init__(parent)
        self._pal = palette
        self._compact = compact

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(4)

        head = QHBoxLayout()
        head.setContentsMargins(0, 0, 0, 0)
        self._label = QLabel(label.upper(), self)
        self._label.setObjectName('FieldLabel')
        head.addWidget(self._label)
        head.addStretch(1)
        if hint:
            self._hint = QLabel(hint, self)
            self._hint.setObjectName('FieldHint')
            head.addWidget(self._hint)
        lay.addLayout(head)

        self.spin = QDoubleSpinBox(self)
        self.spin.setRange(minimum, maximum)
        self.spin.setSingleStep(step)
        self.spin.setDecimals(decimals)
        self.spin.setValue(value)
        self.spin.setSuffix(f' {suffix}' if suffix else '')
        self.spin.setAlignment(Qt.AlignRight)
        self.spin.setObjectName('ParamSpin')
        self.spin.valueChanged.connect(self.valueChanged.emit)
        lay.addWidget(self.spin)

        self.apply_palette(palette)

    def value(self):
        return self.spin.value()

    def setValue(self, v):
        self.spin.blockSignals(True)
        self.spin.setValue(v)
        self.spin.blockSignals(False)

    def apply_palette(self, p):
        self._pal = p
        h = 26 if self._compact else 32
        fs = 11 if self._compact else 12
        self.setStyleSheet(f"""
            QLabel#FieldLabel {{
                color: {p['dim']};
                font-family: 'JetBrains Mono', monospace;
                font-size: 9px;
                font-weight: 600;
                letter-spacing: 1px;
            }}
            QLabel#FieldHint {{
                color: {p['faint']};
                font-family: 'JetBrains Mono', monospace;
                font-size: 9px;
            }}
            QDoubleSpinBox#ParamSpin {{
                background: {p['log_bg']};
                color: {p['ink']};
                border: 1px solid {p['border']};
                border-radius: 4px;
                padding: 0 6px;
                min-height: {h}px;
                font-family: 'JetBrains Mono', monospace;
                font-size: {fs}px;
            }}
            QDoubleSpinBox#ParamSpin:focus {{ border-color: {p['accent']}; }}
            QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {{
                width: 16px;
                background: {p['panel_hi']};
                border-left: 1px solid {p['border']};
            }}
            QDoubleSpinBox::up-button:hover, QDoubleSpinBox::down-button:hover {{
                background: {p['border_hi']};
            }}
        """)


# ───────────────────────── GANTRY TRACK WIDGET ─────────────────────────

class GantryTrack(QWidget):
    """Horizontal track showing carriage position, scan band, and tick marks."""

    def __init__(self, palette=None, parent=None):
        super().__init__(parent)
        self._pal = palette
        self._pos = 0.0
        self._start = 0.0
        self._end = 1000.0
        self._max = 2000.0
        self.setMinimumHeight(56)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    def setState(self, pos, start, end, max_pos=None):
        self._pos = pos
        self._start = start
        self._end = end
        if max_pos:
            self._max = max_pos
        self.update()

    def apply_palette(self, p):
        self._pal = p
        self.update()

    def paintEvent(self, ev):
        p = self._pal
        qp = QPainter(self)
        qp.setRenderHint(QPainter.Antialiasing)
        w = self.width()
        h = self.height() - 18  # leave room for tick labels
        track_y = 0
        # background
        qp.setBrush(QBrush(QColor(p['log_bg'])))
        qp.setPen(QPen(QColor(p['border']), 1))
        qp.drawRoundedRect(QRectF(0.5, track_y + 0.5, w - 1, h - 1), 3, 3)

        # scan range band
        sx = (self._start / self._max) * w
        ex = (self._end / self._max) * w
        band = QColor(p['accent'])
        band.setAlpha(28)
        qp.setBrush(QBrush(band))
        qp.setPen(Qt.NoPen)
        qp.drawRect(QRectF(sx, track_y, ex - sx, h))
        # dashed band borders
        pen = QPen(QColor(p['accent']), 1, Qt.DashLine)
        qp.setPen(pen)
        qp.drawLine(int(sx), track_y, int(sx), track_y + h)
        qp.drawLine(int(ex), track_y, int(ex), track_y + h)

        # tick marks + labels
        qp.setFont(_font(size=8, mono=True))
        for t in (0.0, 0.25, 0.5, 0.75, 1.0):
            x = int(t * w)
            qp.setPen(QPen(QColor(p['border']), 1))
            qp.drawLine(x, track_y, x, track_y + h)
            qp.setPen(QPen(QColor(p['faint']), 1))
            label = f"{int(t * self._max)}"
            tw = qp.fontMetrics().width(label)
            qp.drawText(max(0, min(w - tw, x - tw // 2)), h + 14, label)

        # carriage
        cx = (self._pos / self._max) * w
        # glow
        glow = QColor(p['accent']); glow.setAlpha(80)
        qp.setBrush(QBrush(glow)); qp.setPen(Qt.NoPen)
        qp.drawRect(QRectF(cx - 1.5, track_y - 2, 3, h + 4))
        # square knob
        qp.setBrush(QBrush(QColor(p['accent'])))
        qp.setPen(QPen(QColor(p['bg']), 2))
        size = 12
        qp.drawRect(QRectF(cx - size/2, track_y + h/2 - size/2, size, size))


# ───────────────────────── PROGRESS BAR ─────────────────────────

class ScanProgressBar(QWidget):
    def __init__(self, palette=None, parent=None):
        super().__init__(parent)
        self._pal = palette
        self._pct = 0.0
        self._active = False
        self._shimmer = 0.0
        self.setMinimumHeight(10)
        self.setMaximumHeight(10)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        # shimmer animation
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(33)

    def setProgress(self, pct, active=False):
        self._pct = max(0.0, min(100.0, pct))
        self._active = active
        self.update()

    def _tick(self):
        if self._active:
            self._shimmer = (self._shimmer + 0.02) % 1.0
            self.update()

    def apply_palette(self, p):
        self._pal = p
        self.update()

    def paintEvent(self, ev):
        p = self._pal
        qp = QPainter(self)
        qp.setRenderHint(QPainter.Antialiasing)
        w = self.width()
        h = self.height()
        # background
        qp.setBrush(QBrush(QColor(p['log_bg'])))
        qp.setPen(QPen(QColor(p['border']), 1))
        qp.drawRoundedRect(QRectF(0.5, 0.5, w - 1, h - 1), 2, 2)
        # fill
        fill_w = (self._pct / 100.0) * (w - 2)
        if fill_w > 0:
            if self._active:
                grad = QLinearGradient(1, 0, 1 + fill_w, 0)
                grad.setColorAt(0.0, QColor(p['accent']))
                grad.setColorAt(1.0, QColor(p['blue']))
                qp.setBrush(QBrush(grad))
            else:
                qp.setBrush(QBrush(QColor(p['dim'])))
            qp.setPen(Qt.NoPen)
            qp.drawRoundedRect(QRectF(1, 1, fill_w, h - 2), 2, 2)
            # shimmer
            if self._active and fill_w > 30:
                shx = self._shimmer * fill_w
                grad2 = QLinearGradient(shx - 30, 0, shx + 30, 0)
                trans = QColor(255, 255, 255, 0)
                bright = QColor(255, 255, 255, 80)
                grad2.setColorAt(0.0, trans)
                grad2.setColorAt(0.5, bright)
                grad2.setColorAt(1.0, trans)
                qp.setBrush(QBrush(grad2))
                qp.drawRect(QRectF(max(1, shx - 30), 1, 60, h - 2))


# ───────────────────────── POINT CLOUD MINI ─────────────────────────

class PointCloudPreview(QWidget):
    """Spinning fake point-cloud spiral that animates while running."""

    def __init__(self, palette=None, parent=None):
        super().__init__(parent)
        self._pal = palette
        self._running = False
        self._points = 0
        self._angle = 0.0
        self._cloud = self._gen_cloud(800)
        self.setMinimumHeight(180)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(40)

    def _gen_cloud(self, n):
        pts = []
        for i in range(n):
            t = i / n
            r = 0.4 + 0.2 * math.sin(t * 18)
            a = t * math.pi * 8
            pts.append((
                r * math.cos(a) + (random.random() - 0.5) * 0.06,
                (t - 0.5) * 1.4 + (random.random() - 0.5) * 0.04,
                r * math.sin(a) + (random.random() - 0.5) * 0.06,
            ))
        return pts

    def setState(self, running, points):
        self._running = running
        if points != self._points:
            self._points = points
            n = max(300, min(2000, points // 10))
            if abs(n - len(self._cloud)) > 200:
                self._cloud = self._gen_cloud(n)
        self.update()

    def _tick(self):
        if self._running:
            self._angle += 0.012
            self.update()

    def apply_palette(self, p):
        self._pal = p
        self.update()

    def paintEvent(self, ev):
        p = self._pal
        qp = QPainter(self)
        qp.setRenderHint(QPainter.Antialiasing)
        w = self.width()
        h = self.height()
        # bg
        qp.fillRect(0, 0, w, h, QColor(p['log_bg']))
        # grid
        qp.setPen(QPen(QColor(p['border']), 1))
        for i in range(11):
            x = int(i / 10 * w)
            y = int(i / 10 * h)
            qp.drawLine(x, 0, x, h)
            qp.drawLine(0, y, w, y)
        # points
        cx, cy = w / 2, h / 2
        scale = min(w, h) * 0.32
        ang = self._angle
        cos_a, sin_a = math.cos(ang), math.sin(ang)
        for x, y, z in self._cloud:
            xr = x * cos_a - z * sin_a
            zr = x * sin_a + z * cos_a
            px = cx + xr * scale
            py = cy + y * scale * 0.9
            depth = (zr + 1) / 2
            sz = 0.8 + depth * 1.6
            hue = (160 + zr * 40) / 360.0
            color = QColor.fromHsvF(hue % 1.0, 0.7, 0.4 + depth * 0.4,
                                    0.4 + depth * 0.55)
            qp.fillRect(QRectF(px, py, sz, sz), color)


# ───────────────────────── CAMERA FEED PLACEHOLDER ─────────────────────────

class CameraFeedWidget(QWidget):
    """Animated camera placeholder. Call set_image(QImage) to display real frames."""

    def __init__(self, palette=None, parent=None):
        super().__init__(parent)
        self._pal = palette
        self._image = None
        self._t = 0.0
        self._topic = '/camera/image_raw'
        self.setMinimumHeight(180)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(33)

    def set_image(self, qimage):
        self._image = qimage
        self.update()

    def set_topic(self, topic):
        self._topic = topic
        self.update()

    def apply_palette(self, p):
        self._pal = p
        self.update()

    def _tick(self):
        if self._image is None:
            self._t += 0.016
            self.update()

    def paintEvent(self, ev):
        p = self._pal
        qp = QPainter(self)
        qp.setRenderHint(QPainter.Antialiasing)
        w = self.width(); h = self.height()
        if self._image is not None:
            qp.drawImage(self.rect(), self._image)
        else:
            qp.fillRect(0, 0, w, h, QColor(p['log_bg']))
            # diagonal stripes
            qp.setPen(QPen(QColor(255, 255, 255, 12), 1))
            for x in range(-h, w, 14):
                qp.drawLine(x, 0, x + h, h)
            # crosshairs
            qp.setPen(QPen(QColor(255, 255, 255, 40), 1))
            qp.drawLine(w // 2, 0, w // 2, h)
            qp.drawLine(0, h // 2, w, h // 2)
            # scanline
            y = (math.sin(self._t * 0.6) * 0.5 + 0.5) * h
            grad = QLinearGradient(0, y - 30, 0, y + 30)
            c = QColor(p['accent'])
            grad.setColorAt(0.0, QColor(c.red(), c.green(), c.blue(), 0))
            grad.setColorAt(0.5, QColor(c.red(), c.green(), c.blue(), 50))
            grad.setColorAt(1.0, QColor(c.red(), c.green(), c.blue(), 0))
            qp.setBrush(QBrush(grad))
            qp.setPen(Qt.NoPen)
            qp.drawRect(QRectF(0, y - 30, w, 60))

        # LIVE pill
        qp.setFont(_font(size=8, mono=True, weight=QFont.Bold))
        pill_text = f'● LIVE · {self._topic}'
        tw = qp.fontMetrics().width(pill_text) + 14
        qp.setBrush(QBrush(QColor(0, 0, 0, 140)))
        qp.setPen(QPen(QColor(p['accent']), 1))
        qp.drawRoundedRect(QRectF(8, 8, tw, 18), 3, 3)
        qp.setPen(QPen(QColor(p['accent'])))
        qp.drawText(QRectF(8, 8, tw, 18), Qt.AlignCenter, pill_text)


# ───────────────────────── E-STOP BUTTON ─────────────────────────

class EStopButton(QPushButton):
    def __init__(self, parent=None):
        super().__init__('■ EMERGENCY STOP', parent)
        self.setCursor(Qt.PointingHandCursor)
        self.setStyleSheet("""
            QPushButton {
                background: #c4282c;
                color: #fff;
                border: 2px solid #ff8080;
                border-radius: 4px;
                padding: 8px 18px;
                font-family: 'JetBrains Mono', monospace;
                font-size: 12px;
                font-weight: 700;
                letter-spacing: 1.5px;
                text-transform: uppercase;
            }
            QPushButton:hover { background: #d83438; }
            QPushButton:pressed { background: #a82024; }
        """)


# ───────────────────────── MAIN GUI WIDGET ─────────────────────────

class MissionControlGUI(QWidget):
    """
    Main scanner GUI widget. Wire your ROS2 node to:

    Inputs (call from your node):
        gui.set_position(mm)
        gui.set_rpm(rpm)
        gui.set_points(count)
        gui.set_connected(bool)
        gui.set_scan_state('idle' | 'scanning' | 'paused')
        gui.set_estop_active(bool)
        gui.append_log(kind='info'|'ok'|'warn'|'err', msg=str)
        gui.set_camera_image(QImage)        # optional, real frames

    Signals (connect from your node):
        start_scan_requested        -> ()
        stop_scan_requested         -> ()
        pause_scan_requested        -> ()
        estop_requested             -> ()
        estop_reset_requested       -> ()
        move_requested              -> (target_mm, speed_mms, accel_mms2)
        home_requested              -> ()
        scan_settings_changed       -> (start_mm, end_mm)
        lidar_settings_changed      -> (angle_min, angle_max, range_max, sim_encoder)
        rviz_launch_requested       -> ()
        rviz_clear_requested        -> ()
    """

    # signals
    start_scan_requested  = pyqtSignal()
    stop_scan_requested   = pyqtSignal()
    pause_scan_requested  = pyqtSignal()
    estop_requested       = pyqtSignal()
    estop_reset_requested = pyqtSignal()
    move_requested        = pyqtSignal(float, float, float)  # target, speed, accel
    home_requested        = pyqtSignal()
    scan_settings_changed = pyqtSignal(float, float)
    lidar_settings_changed = pyqtSignal(float, float, float, bool)
    rviz_launch_requested = pyqtSignal()
    rviz_clear_requested  = pyqtSignal()
    preset_loaded         = pyqtSignal(dict)

    def __init__(self, theme='dark', density='comfortable',
                 demo_mode=True, parent=None):
        super().__init__(parent)
        self._theme = theme
        self._compact = (density == 'compact')
        self._pal = PALETTES[theme]
        self._demo_mode = demo_mode

        # state
        self._position = 0.0
        self._target = 1000.0
        self._start_pos = 0.0
        self._end_pos = 1000.0
        self._speed = 50.0
        self._accel = 120.0
        self._angle_min = 0.0
        self._angle_max = 90.0
        self._range_max = 12.0
        self._sim_encoder = False
        self._rpm = 0.0
        self._points = 0
        self._connected = True
        self._scan_state = 'idle'
        self._estop = False
        self._rviz_running = False

        self.setWindowTitle('Scanner Control · Mission Control')
        self.setMinimumSize(1280, 800)

        self._build_ui()
        self._apply_palette()

        # clock tick
        self._clock_timer = QTimer(self)
        self._clock_timer.timeout.connect(self._update_clock)
        self._clock_timer.start(1000)

        # demo simulation
        if self._demo_mode:
            self._sim_dir = 1
            self._sim_timer = QTimer(self)
            self._sim_timer.timeout.connect(self._sim_tick)
            self._sim_timer.start(50)

        # seed log
        self.append_log('info', 'Scanner GUI initialized — awaiting telemetry.')

    # ───────── UI BUILD ─────────

    def _build_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        outer.addWidget(self._build_topbar())
        outer.addWidget(self._build_telemetry_strip())
        outer.addWidget(self._build_main_grid(), 1)

    def _build_topbar(self):
        bar = QFrame()
        bar.setObjectName('TopBar')
        h = QHBoxLayout(bar)
        h.setContentsMargins(18, 10, 18, 10)
        h.setSpacing(16)

        # logo block
        logo = QLabel('S'); logo.setObjectName('Logo')
        logo.setFixedSize(26, 26)
        logo.setAlignment(Qt.AlignCenter)

        title = QLabel('SCANNER CONTROL'); title.setObjectName('AppTitle')
        sub = QLabel('v2.4 · ROS2 HUMBLE'); sub.setObjectName('AppSub')

        title_col = QVBoxLayout()
        title_col.setSpacing(0)
        title_col.addWidget(title)
        title_col.addWidget(sub)
        title_box = QHBoxLayout()
        title_box.setSpacing(8)
        title_box.addWidget(logo)
        title_box.addLayout(title_col)
        h.addLayout(title_box)

        # divider
        sep = QFrame(); sep.setFrameShape(QFrame.VLine); sep.setObjectName('TopSep')
        h.addWidget(sep)

        # status pills
        self.lbl_link  = self._status_pill('LINK',  'ONLINE', ok=True)
        self.lbl_mode  = self._status_pill('MODE',  'IDLE')
        self.lbl_node  = self._status_pill('NODE',  '/scanner_node', ok=True)
        self.lbl_clock = self._status_pill('CLOCK', '--:--:--')
        for w in (self.lbl_link, self.lbl_mode, self.lbl_node, self.lbl_clock):
            h.addWidget(w)
        h.addStretch(1)

        # E-Stop
        self.estop_btn = EStopButton()
        self.estop_btn.clicked.connect(self._on_estop_click)
        self.estop_reset_btn = styled_button('↻ Reset E-Stop', 'warn',
                                             palette=self._pal, compact=self._compact,
                                             on_click=self.estop_reset_requested.emit)
        self.estop_reset_btn.hide()
        h.addWidget(self.estop_btn)
        h.addWidget(self.estop_reset_btn)
        return bar

    def _status_pill(self, label, value, ok=False, err=False):
        w = QWidget()
        l = QHBoxLayout(w); l.setContentsMargins(0, 0, 0, 0); l.setSpacing(6)
        lbl = QLabel(label); lbl.setObjectName('StatusLabel')
        val = QLabel(value); val.setObjectName('StatusValue')
        if ok: val.setProperty('state', 'ok')
        elif err: val.setProperty('state', 'err')
        l.addWidget(lbl); l.addWidget(val)
        w._lbl = lbl; w._val = val
        return w

    def _set_pill(self, pill, value, ok=False, err=False):
        pill._val.setText(value)
        state = 'ok' if ok else 'err' if err else 'normal'
        pill._val.setProperty('state', state)
        pill._val.style().unpolish(pill._val); pill._val.style().polish(pill._val)

    def _build_telemetry_strip(self):
        wrap = QFrame(); wrap.setObjectName('TelemetryStrip')
        g = QHBoxLayout(wrap)
        g.setContentsMargins(18, 12, 18, 0)
        g.setSpacing(12)

        self.tile_pos    = StatTile('POSITION',      '0.0',     'mm',   accent=self._pal['accent'], palette=self._pal, compact=self._compact)
        self.tile_rpm    = StatTile('MOTOR RPM',     '0',       'rpm',  accent=self._pal['dim'],    palette=self._pal, compact=self._compact)
        self.tile_points = StatTile('POINTS',        '0',       'pts',  accent=self._pal['blue'],   palette=self._pal, compact=self._compact)
        self.tile_prog   = StatTile('SCAN PROGRESS', '0',       '%',    accent=self._pal['purple'], palette=self._pal, compact=self._compact)
        self.tile_speed  = StatTile('SPEED',         '50',      'mm/s',                              palette=self._pal, compact=self._compact)
        self.tile_range  = StatTile('RANGE',         '0°→90°',  '12m',                               palette=self._pal, compact=self._compact)

        for t in (self.tile_pos, self.tile_rpm, self.tile_points,
                  self.tile_prog, self.tile_speed, self.tile_range):
            g.addWidget(t, 1)
        return wrap

    def _build_main_grid(self):
        wrap = QFrame(); wrap.setObjectName('MainGrid')
        grid = QGridLayout(wrap)
        grid.setContentsMargins(18, 12, 18, 12)
        grid.setSpacing(12)
        grid.setColumnStretch(0, 0)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 0)
        grid.setColumnMinimumWidth(0, 280)
        grid.setColumnMinimumWidth(2, 380)

        # LEFT
        left = QWidget()
        ll = QVBoxLayout(left); ll.setContentsMargins(0,0,0,0); ll.setSpacing(12)
        ll.addWidget(self._build_lidar_panel())
        ll.addWidget(self._build_gantry_panel())
        ll.addWidget(self._build_presets_panel())
        ll.addStretch(1)

        # CENTER
        center = QWidget()
        cl = QVBoxLayout(center); cl.setContentsMargins(0,0,0,0); cl.setSpacing(12)
        cl.addWidget(self._build_scan_ops_panel())
        cl.addWidget(self._build_log_panel(), 1)

        # RIGHT
        right = QWidget()
        rl = QVBoxLayout(right); rl.setContentsMargins(0,0,0,0); rl.setSpacing(12)
        rl.addWidget(self._build_camera_panel())
        rl.addWidget(self._build_pointcloud_panel(), 1)
        rl.addWidget(self._build_history_panel())

        grid.addWidget(left,   0, 0)
        grid.addWidget(center, 0, 1)
        grid.addWidget(right,  0, 2)
        return wrap

    def _build_lidar_panel(self):
        sec = SectionFrame('LIDAR Config', accent=self._pal['accent'],
                           palette_dict=self._pal, compact=self._compact)
        self.f_angle_min = ParamField('Angle Min', self._angle_min, step=5,
                                      suffix='°', minimum=-180, maximum=180,
                                      hint='degrees', palette=self._pal, compact=self._compact)
        self.f_angle_max = ParamField('Angle Max', self._angle_max, step=5,
                                      suffix='°', minimum=-180, maximum=180,
                                      hint='degrees', palette=self._pal, compact=self._compact)
        self.f_range_max = ParamField('Range Max', self._range_max, step=1,
                                      suffix='m', minimum=0.5, maximum=30,
                                      hint='meters', palette=self._pal, compact=self._compact)
        self.cb_sim = QCheckBox('SIMULATE ENCODER')
        self.cb_sim.setObjectName('TweakCheck')
        self.cb_sim.toggled.connect(self._emit_lidar)

        for w in (self.f_angle_min, self.f_angle_max, self.f_range_max, self.cb_sim):
            sec.body_layout.addWidget(w)

        for fld in (self.f_angle_min, self.f_angle_max, self.f_range_max):
            fld.valueChanged.connect(self._emit_lidar)

        btn_row = QHBoxLayout(); btn_row.setSpacing(6)
        b_apply = styled_button('Apply', 'primary', palette=self._pal,
                                compact=self._compact,
                                on_click=lambda: (self._emit_lidar(),
                                                  self.append_log('ok', 'Settings applied to /scanner_node')))
        b_refresh = styled_button('Refresh', 'ghost', palette=self._pal,
                                  compact=self._compact,
                                  on_click=lambda: self.append_log('info', 'Refreshed from node'))
        btn_row.addWidget(b_apply, 1); btn_row.addWidget(b_refresh, 1)
        sec.body_layout.addLayout(btn_row)
        sec.body_layout.addStretch(1)
        return sec

    def _build_gantry_panel(self):
        sec = SectionFrame('Gantry Control', accent=self._pal['blue'],
                           palette_dict=self._pal, compact=self._compact)
        self.f_target = ParamField('Target', self._target, step=50, suffix='mm',
                                   minimum=0, maximum=2000, hint='mm',
                                   palette=self._pal, compact=self._compact)
        self.f_speed  = ParamField('Speed', self._speed, step=5, suffix='mm/s',
                                   minimum=1, maximum=300, hint='mm/s',
                                   palette=self._pal, compact=self._compact)
        self.f_accel  = ParamField('Acceleration', self._accel, step=10,
                                   suffix='mm/s²', minimum=10, maximum=500,
                                   hint='mm/s²', palette=self._pal, compact=self._compact)

        for w in (self.f_target, self.f_speed, self.f_accel):
            sec.body_layout.addWidget(w)

        nav_row = QHBoxLayout(); nav_row.setSpacing(6)
        nav_row.addWidget(styled_button('Home',  'default', palette=self._pal,
            compact=self._compact, on_click=lambda: (self.f_target.setValue(0), self.home_requested.emit())), 1)
        nav_row.addWidget(styled_button('Start', 'default', palette=self._pal,
            compact=self._compact, on_click=lambda: self.f_target.setValue(self._start_pos)), 1)
        nav_row.addWidget(styled_button('End',   'default', palette=self._pal,
            compact=self._compact, on_click=lambda: self.f_target.setValue(self._end_pos)), 1)
        sec.body_layout.addLayout(nav_row)

        send = styled_button('► Send Move Command', 'blue', palette=self._pal,
                             compact=self._compact, on_click=self._on_send_move)
        sec.body_layout.addWidget(send)
        sec.body_layout.addStretch(1)
        return sec

    def _build_presets_panel(self):
        sec = SectionFrame('Presets', accent=self._pal['warn'],
                           palette_dict=self._pal, compact=self._compact)
        presets = [
            dict(name='High-density · 0.5 m', angle_min=0, angle_max=90,
                 range_max=6, speed=25, accel=80),
            dict(name='Standard floor scan',  angle_min=0, angle_max=90,
                 range_max=12, speed=50, accel=120),
            dict(name='Wide sweep · fast',    angle_min=-45, angle_max=135,
                 range_max=20, speed=100, accel=200),
        ]
        for p in presets:
            btn = QPushButton()
            btn.setObjectName('PresetBtn')
            inner = QVBoxLayout(btn); inner.setContentsMargins(10, 6, 10, 6); inner.setSpacing(2)
            n = QLabel(p['name']); n.setObjectName('PresetName')
            d = QLabel(f"{p['angle_min']}°→{p['angle_max']}° · {p['speed']}mm/s")
            d.setObjectName('PresetDetail')
            inner.addWidget(n); inner.addWidget(d)
            btn.setFixedHeight(46)
            btn.clicked.connect(lambda _=False, pp=p: self._load_preset(pp))
            sec.body_layout.addWidget(btn)
        save = styled_button('+ Save Current', 'ghost', palette=self._pal,
                             compact=self._compact)
        sec.body_layout.addWidget(save)
        sec.body_layout.addStretch(1)
        return sec

    def _build_scan_ops_panel(self):
        # right widget: status indicator
        self.scan_state_lbl = QLabel('○ IDLE')
        self.scan_state_lbl.setObjectName('ScanStateLbl')
        sec = SectionFrame('Scan Operations', accent=self._pal['dim'],
                           palette_dict=self._pal, compact=self._compact,
                           right_widget=self.scan_state_lbl)

        # start/end inputs
        rng = QHBoxLayout(); rng.setSpacing(10)
        self.f_start = ParamField('Start Position', self._start_pos, step=50,
                                  suffix='mm', minimum=0, maximum=2000, hint='mm',
                                  palette=self._pal, compact=self._compact)
        self.f_end = ParamField('End Position', self._end_pos, step=50,
                                suffix='mm', minimum=0, maximum=2000, hint='mm',
                                palette=self._pal, compact=self._compact)
        self.f_start.valueChanged.connect(self._emit_scan_settings)
        self.f_end.valueChanged.connect(self._emit_scan_settings)
        rng.addWidget(self.f_start, 1); rng.addWidget(self.f_end, 1)
        sec.body_layout.addLayout(rng)

        # gantry track
        track_head = QHBoxLayout()
        th_l = QLabel('GANTRY TRACK · 2000 mm'); th_l.setObjectName('FieldLabel')
        self.track_pos_lbl = QLabel('0.0 mm'); self.track_pos_lbl.setObjectName('TrackPosLbl')
        track_head.addWidget(th_l); track_head.addStretch(1); track_head.addWidget(self.track_pos_lbl)
        sec.body_layout.addLayout(track_head)
        self.track = GantryTrack(palette=self._pal)
        sec.body_layout.addWidget(self.track)

        # progress
        prog_head = QHBoxLayout()
        ph_l = QLabel('SCAN PROGRESS'); ph_l.setObjectName('FieldLabel')
        self.prog_pct_lbl = QLabel('0.0%'); self.prog_pct_lbl.setObjectName('FieldHint')
        prog_head.addWidget(ph_l); prog_head.addStretch(1); prog_head.addWidget(self.prog_pct_lbl)
        sec.body_layout.addLayout(prog_head)
        self.prog_bar = ScanProgressBar(palette=self._pal)
        sec.body_layout.addWidget(self.prog_bar)

        # transport
        tx = QHBoxLayout(); tx.setSpacing(8)
        self.btn_start = styled_button('▶ Start Scan', 'primary', palette=self._pal,
                                       compact=self._compact, on_click=self._on_start)
        self.btn_pause = styled_button('❚❚ Pause', 'warn', palette=self._pal,
                                       compact=self._compact, on_click=self._on_pause)
        self.btn_resume= styled_button('▶ Resume', 'primary', palette=self._pal,
                                       compact=self._compact, on_click=self._on_pause)
        self.btn_stop  = styled_button('■ Stop',  'danger', palette=self._pal,
                                       compact=self._compact, on_click=self._on_stop)
        self.btn_reset = styled_button('Reset', 'ghost', palette=self._pal,
                                       compact=self._compact, on_click=self._on_reset_range)
        self.btn_pause.hide(); self.btn_resume.hide()
        tx.addWidget(self.btn_start, 2)
        tx.addWidget(self.btn_pause, 2)
        tx.addWidget(self.btn_resume, 2)
        tx.addWidget(self.btn_stop, 1)
        tx.addWidget(self.btn_reset, 1)
        sec.body_layout.addLayout(tx)

        # cloud quality strip
        ql = QGridLayout(); ql.setSpacing(8)
        self.lbl_density = self._mini_stat('Density', '0', 'pts/mm')
        self.lbl_coverage= self._mini_stat('Coverage', '0', '%')
        self.lbl_avgrange= self._mini_stat('Avg Range', '0.00', 'm')
        self.lbl_drop    = self._mini_stat('Drop Rate', '0.04', '%')
        for i, w in enumerate((self.lbl_density, self.lbl_coverage,
                                self.lbl_avgrange, self.lbl_drop)):
            ql.addWidget(w, 0, i)
        wrap = QFrame(); wrap.setObjectName('QualityStrip')
        wrap.setStyleSheet(f'#QualityStrip {{ border-top: 1px solid {self._pal["border"]}; }}')
        wrap.setLayout(ql)
        ql.setContentsMargins(0, 10, 0, 0)
        sec.body_layout.addWidget(wrap)

        return sec

    def _mini_stat(self, label, value, unit):
        w = QWidget()
        l = QVBoxLayout(w); l.setContentsMargins(0, 0, 0, 0); l.setSpacing(2)
        lab = QLabel(label.upper()); lab.setObjectName('MiniStatLbl')
        val = QLabel(f"{value} <span style='color:{self._pal['dim']};font-weight:500;font-size:9px;'>{unit}</span>")
        val.setObjectName('MiniStatVal')
        val.setTextFormat(Qt.RichText)
        l.addWidget(lab); l.addWidget(val)
        w._val = val; w._unit = unit
        return w

    def _set_mini_stat(self, w, value):
        w._val.setText(f"{value} <span style='color:{self._pal['dim']};font-weight:500;font-size:9px;'>{w._unit}</span>")

    def _build_log_panel(self):
        sec = SectionFrame('Log Console', accent=self._pal['purple'],
                           palette_dict=self._pal, compact=self._compact)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setObjectName('LogConsole')
        self.log.setFont(_font(size=10, mono=True))
        sec.body_layout.addWidget(self.log, 1)
        return sec

    def _build_camera_panel(self):
        sec = SectionFrame('Camera Feed', accent=self._pal['accent'],
                           palette_dict=self._pal, compact=self._compact)
        self.camera = CameraFeedWidget(palette=self._pal)
        self.camera.setMinimumHeight(180)
        sec.body_layout.addWidget(self.camera)
        return sec

    def _build_pointcloud_panel(self):
        # right widget: launch/clear toggle
        self.btn_rviz = QPushButton('▶ LAUNCH')
        self.btn_rviz.setObjectName('RvizBtn')
        self.btn_rviz.setCursor(Qt.PointingHandCursor)
        self.btn_rviz.clicked.connect(self._on_rviz_toggle)

        sec = SectionFrame('Point Cloud · RViz', accent=self._pal['blue'],
                           palette_dict=self._pal, compact=self._compact,
                           right_widget=self.btn_rviz)
        self.cloud = PointCloudPreview(palette=self._pal)
        sec.body_layout.addWidget(self.cloud, 1)
        self.rviz_status_lbl = QLabel('RVIZ NOT RUNNING — START A SCAN')
        self.rviz_status_lbl.setObjectName('RvizStatus')
        self.rviz_status_lbl.setAlignment(Qt.AlignCenter)
        sec.body_layout.addWidget(self.rviz_status_lbl)
        return sec

    def _build_history_panel(self):
        sec = SectionFrame('Recent Scans', accent=self._pal['warn'],
                           palette_dict=self._pal, compact=self._compact)
        sec.setMaximumHeight(180)
        items = [
            ('scan_20260426_182244', '0 → 1000 mm', 19919, '0:28', '2 min ago'),
            ('scan_20260426_174102', '0 → 1500 mm', 31872, '0:42', '52 min ago'),
            ('scan_20260426_152417', '500 → 2000 mm', 27431, '0:35', '4h ago'),
            ('scan_20260425_201803', '0 → 1000 mm', 18204, '0:27', 'Yesterday'),
        ]
        scroll = QScrollArea(); scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setObjectName('HistScroll')
        inner = QWidget(); il = QVBoxLayout(inner); il.setContentsMargins(0,0,0,0); il.setSpacing(4)
        for name, rng, pts, dur, when in items:
            card = QFrame(); card.setObjectName('HistCard')
            cl = QGridLayout(card); cl.setContentsMargins(8, 6, 8, 6); cl.setSpacing(2)
            n = QLabel(f'{name}.ply'); n.setObjectName('HistName')
            w = QLabel(when); w.setObjectName('HistWhen')
            d = QLabel(f"{rng} · {pts:,} pts · {dur}"); d.setObjectName('HistDetail')
            cl.addWidget(n, 0, 0); cl.addWidget(w, 0, 1, Qt.AlignRight)
            cl.addWidget(d, 1, 0, 1, 2)
            il.addWidget(card)
        il.addStretch(1)
        scroll.setWidget(inner)
        sec.body_layout.addWidget(scroll)
        return sec

    # ───────── INTERACTION HANDLERS ─────────

    def _on_start(self):
        self._scan_state = 'scanning'
        self._points = 0
        self._update_transport()
        self.append_log('ok', f'Scan started: {self._start_pos:.2f} → {self._end_pos:.2f} mm')
        self.start_scan_requested.emit()

    def _on_stop(self):
        self._scan_state = 'idle'
        self._update_transport()
        self.append_log('warn', f'Scan stopped: Saved {self._points} points to /tmp/scan_{int(time.time())}.ply')
        self.stop_scan_requested.emit()

    def _on_pause(self):
        self._scan_state = 'paused' if self._scan_state == 'scanning' else 'scanning'
        self._update_transport()
        self.append_log('info', 'Scan resumed' if self._scan_state == 'scanning' else 'Scan paused')
        self.pause_scan_requested.emit()

    def _on_reset_range(self):
        self.f_start.setValue(0); self.f_end.setValue(1000)
        self._start_pos = 0; self._end_pos = 1000
        self._emit_scan_settings()

    def _on_send_move(self):
        if self._estop: return
        self._target = self.f_target.value()
        self._speed = self.f_speed.value()
        self._accel = self.f_accel.value()
        self.append_log('info', f'Gantry move: target={self._target:.2f} mm, speed={self._speed:.2f} mm/s, accel={self._accel:.2f} mm/s²')
        self.move_requested.emit(self._target, self._speed, self._accel)

    def _on_estop_click(self):
        self._estop = True
        self._scan_state = 'idle'
        self.estop_btn.hide()
        self.estop_reset_btn.show()
        self.append_log('err', 'EMERGENCY STOP — all motion halted')
        self._update_transport()
        self.estop_requested.emit()

    def _on_estop_reset(self):
        self._estop = False
        self.estop_btn.show()
        self.estop_reset_btn.hide()
        self.append_log('ok', 'E-Stop cleared. System ready.')
        self._update_transport()

    def _on_rviz_toggle(self):
        if self._rviz_running:
            self._rviz_running = False
            self.btn_rviz.setText('▶ LAUNCH')
            self.append_log('info', 'RViz display cleared.')
            self.rviz_clear_requested.emit()
        else:
            self._rviz_running = True
            self.btn_rviz.setText('■ CLEAR')
            self.append_log('info', 'RViz launched.')
            self.rviz_launch_requested.emit()

    def _emit_scan_settings(self):
        self._start_pos = self.f_start.value()
        self._end_pos = self.f_end.value()
        self.scan_settings_changed.emit(self._start_pos, self._end_pos)

    def _emit_lidar(self):
        self._angle_min = self.f_angle_min.value()
        self._angle_max = self.f_angle_max.value()
        self._range_max = self.f_range_max.value()
        self._sim_encoder = self.cb_sim.isChecked()
        self.lidar_settings_changed.emit(self._angle_min, self._angle_max,
                                          self._range_max, self._sim_encoder)
        # update range tile
        self.tile_range.setValue(f'{self._angle_min:.0f}°→{self._angle_max:.0f}°',
                                 f'{self._range_max:.0f}m')

    def _load_preset(self, p):
        self.f_angle_min.setValue(p['angle_min'])
        self.f_angle_max.setValue(p['angle_max'])
        self.f_range_max.setValue(p['range_max'])
        self.f_speed.setValue(p['speed'])
        self.f_accel.setValue(p['accel'])
        self._emit_lidar()
        self.append_log('info', f"Loaded preset: {p['name']}")
        self.preset_loaded.emit(p)

    # ───────── PUBLIC API (for ROS2 node) ─────────

    def set_position(self, mm):
        self._position = mm
        self.tile_pos.setValue(f'{mm:.1f}')
        self.track_pos_lbl.setText(f'{mm:.1f} mm')
        self._refresh_progress()

    def set_rpm(self, rpm):
        self._rpm = rpm
        self.tile_rpm.setValue(f'{rpm:.0f}')
        accent = self._pal['accent'] if rpm > 100 else self._pal['dim']
        self.tile_rpm.setAccent(accent)

    def set_points(self, count):
        self._points = count
        self.tile_points.setValue(f'{count:,}')
        if count > 0:
            self.cloud.setState(self._scan_state == 'scanning' or self._rviz_running, count)

    def set_connected(self, ok):
        self._connected = ok
        self._set_pill(self.lbl_link, 'ONLINE' if ok else 'OFFLINE', ok=ok, err=not ok)

    def set_scan_state(self, state):
        self._scan_state = state
        self._update_transport()

    def set_estop_active(self, active):
        if active and not self._estop: self._on_estop_click()
        elif not active and self._estop: self._on_estop_reset()

    def append_log(self, kind, msg):
        ts = datetime.now().strftime('%H:%M:%S')
        kind_colors = {
            'info': self._pal['dim'],
            'ok':   self._pal['accent'],
            'warn': self._pal['warn'],
            'err':  self._pal['err'],
        }
        kind_label = {'ok': 'OK  ', 'warn': 'WARN', 'err': 'ERR ', 'info': 'INFO'}[kind]
        col = kind_colors[kind]
        line = (f"<span style='color:{self._pal['faint']};'>[{ts}]</span> "
                f"<span style='color:{col};font-weight:600;'>{kind_label}</span> "
                f"<span style='color:{self._pal['ink']};'>{msg}</span>")
        self.log.appendHtml(line)
        self.log.verticalScrollBar().setValue(self.log.verticalScrollBar().maximum())

    def set_camera_image(self, qimage):
        self.camera.set_image(qimage)

    def set_camera_topic(self, topic):
        self.camera.set_topic(topic)

    # ───────── INTERNAL UPDATE LOOPS ─────────

    def _refresh_progress(self):
        if self._end_pos == self._start_pos:
            pct = 0.0
        else:
            pct = max(0.0, min(100.0, (self._position - self._start_pos) /
                                       (self._end_pos - self._start_pos) * 100))
        self.prog_pct_lbl.setText(f'{pct:.1f}%')
        self.prog_bar.setProgress(pct, active=self._scan_state == 'scanning')
        self.tile_prog.setValue(f'{pct:.0f}')
        self.track.setState(self._position, self._start_pos, self._end_pos)

        # mini stats
        self._set_mini_stat(self.lbl_density,
                            f'{(self._points / max(1, pct * 2)):.0f}')
        self._set_mini_stat(self.lbl_coverage, f'{pct:.0f}')
        self._set_mini_stat(self.lbl_avgrange, f'{(self._range_max * 0.62):.2f}')

    def _update_transport(self):
        is_scan = self._scan_state == 'scanning'
        is_pause = self._scan_state == 'paused'
        self.btn_start.setVisible(not is_scan and not is_pause)
        self.btn_pause.setVisible(is_scan)
        self.btn_resume.setVisible(is_pause)
        self.btn_start.setEnabled(not self._estop)
        self.btn_stop.setEnabled(is_scan or is_pause)

        if self._estop:
            self.scan_state_lbl.setText('■ E-STOP')
            self.scan_state_lbl.setStyleSheet(f'color:{self._pal["err"]};font-family:"JetBrains Mono",monospace;font-size:10px;font-weight:700;letter-spacing:1px;')
            self._set_pill(self.lbl_mode, 'E-STOP', err=True)
        elif is_scan:
            self.scan_state_lbl.setText('● ACTIVE')
            self.scan_state_lbl.setStyleSheet(f'color:{self._pal["accent"]};font-family:"JetBrains Mono",monospace;font-size:10px;font-weight:700;letter-spacing:1px;')
            self._set_pill(self.lbl_mode, 'SCANNING', ok=True)
        elif is_pause:
            self.scan_state_lbl.setText('❚❚ PAUSED')
            self.scan_state_lbl.setStyleSheet(f'color:{self._pal["warn"]};font-family:"JetBrains Mono",monospace;font-size:10px;font-weight:700;letter-spacing:1px;')
            self._set_pill(self.lbl_mode, 'PAUSED')
        else:
            self.scan_state_lbl.setText('○ IDLE')
            self.scan_state_lbl.setStyleSheet(f'color:{self._pal["dim"]};font-family:"JetBrains Mono",monospace;font-size:10px;font-weight:700;letter-spacing:1px;')
            self._set_pill(self.lbl_mode, 'IDLE')

    def _update_clock(self):
        self._set_pill(self.lbl_clock, datetime.now().strftime('%H:%M:%S'))

    def _sim_tick(self):
        if self._estop:
            return
        if self._scan_state == 'scanning':
            d = self._sim_dir
            np = self._position + d * self._speed * 0.05
            if np >= self._end_pos:
                np = self._end_pos; self._sim_dir = -1
            if np <= self._start_pos:
                np = self._start_pos; self._sim_dir = 1
            self.set_position(np)
            self.set_rpm(800 + math.sin(time.time() * 0.7) * 30 + random.random() * 10)
            self.set_points(self._points + random.randint(120, 200))
        else:
            # decay rpm
            self.set_rpm(self._rpm * 0.85)
            # drift toward target
            diff = self._target - self._position
            if abs(diff) > 0.5:
                step = math.copysign(min(abs(diff), self._speed * 0.05), diff)
                self.set_position(self._position + step)

    # ───────── PALETTE / STYLESHEET ─────────

    def _apply_palette(self):
        p = self._pal
        # estop_reset_btn click connect (delayed because button created in topbar)
        self.estop_reset_btn.clicked.connect(self._on_estop_reset)

        self.setStyleSheet(f"""
            QWidget {{
                background: {p['bg']};
                color: {p['ink']};
                font-family: 'Inter', 'Helvetica Neue', sans-serif;
                font-size: 12px;
            }}
            QFrame#TopBar {{
                background: {p['panel']};
                border-bottom: 1px solid {p['border']};
            }}
            QFrame#TelemetryStrip, QFrame#MainGrid {{ background: {p['bg']}; }}
            QLabel#Logo {{
                background: {p['accent']};
                color: {p['accent_ink']};
                border-radius: 4px;
                font-family: 'JetBrains Mono', monospace;
                font-size: 13px;
                font-weight: 700;
            }}
            QLabel#AppTitle {{
                font-size: 12px;
                font-weight: 600;
                color: {p['ink']};
            }}
            QLabel#AppSub {{
                font-family: 'JetBrains Mono', monospace;
                font-size: 9px;
                color: {p['dim']};
                letter-spacing: 1px;
            }}
            QFrame#TopSep {{ color: {p['border']}; max-width: 1px; }}
            QLabel#StatusLabel {{
                color: {p['faint']};
                font-family: 'JetBrains Mono', monospace;
                font-size: 11px;
                font-weight: 500;
                letter-spacing: 1px;
            }}
            QLabel#StatusValue {{
                color: {p['ink']};
                font-family: 'JetBrains Mono', monospace;
                font-size: 11px;
                font-weight: 600;
            }}
            QLabel#StatusValue[state="ok"]  {{ color: {p['accent']}; }}
            QLabel#StatusValue[state="err"] {{ color: {p['err']}; }}
            QLabel#FieldLabel {{
                color: {p['dim']};
                font-family: 'JetBrains Mono', monospace;
                font-size: 9px;
                font-weight: 600;
                letter-spacing: 1px;
            }}
            QLabel#FieldHint {{
                color: {p['faint']};
                font-family: 'JetBrains Mono', monospace;
                font-size: 9px;
            }}
            QLabel#TrackPosLbl {{
                color: {p['accent']};
                font-family: 'JetBrains Mono', monospace;
                font-size: 10px;
                font-weight: 600;
            }}
            QLabel#MiniStatLbl {{
                color: {p['dim']};
                font-family: 'JetBrains Mono', monospace;
                font-size: 9px;
                font-weight: 600;
                letter-spacing: 1px;
            }}
            QLabel#MiniStatVal {{
                color: {p['ink']};
                font-family: 'JetBrains Mono', monospace;
                font-size: 14px;
                font-weight: 600;
            }}
            QPlainTextEdit#LogConsole {{
                background: {p['log_bg']};
                color: {p['ink']};
                border: 1px solid {p['border']};
                border-radius: 3px;
                padding: 4px 6px;
                selection-background-color: {p['accent']};
            }}
            QCheckBox#TweakCheck {{
                color: {p['dim']};
                font-family: 'JetBrains Mono', monospace;
                font-size: 10px;
                letter-spacing: 0.5px;
                spacing: 8px;
            }}
            QCheckBox#TweakCheck::indicator {{
                width: 14px; height: 14px;
                background: {p['log_bg']};
                border: 1px solid {p['border']};
                border-radius: 2px;
            }}
            QCheckBox#TweakCheck::indicator:checked {{
                background: {p['accent']};
                border: 1px solid {p['accent']};
            }}
            QPushButton#PresetBtn {{
                background: {p['log_bg']};
                border: 1px solid {p['border']};
                border-radius: 3px;
                text-align: left;
                padding: 0;
            }}
            QPushButton#PresetBtn:hover {{
                background: {p['panel_hi']};
                border-color: {p['border_hi']};
            }}
            QLabel#PresetName {{
                color: {p['ink']};
                font-size: 11px;
                font-weight: 500;
            }}
            QLabel#PresetDetail {{
                color: {p['dim']};
                font-family: 'JetBrains Mono', monospace;
                font-size: 9px;
            }}
            QPushButton#RvizBtn {{
                background: transparent;
                color: {p['purple']};
                border: 1px solid {p['purple']};
                border-radius: 2px;
                padding: 3px 8px;
                font-family: 'JetBrains Mono', monospace;
                font-size: 9px;
                font-weight: 600;
                letter-spacing: 0.5px;
                text-transform: uppercase;
            }}
            QPushButton#RvizBtn:hover {{
                background: {p['purple']};
                color: {p['accent_ink']};
            }}
            QLabel#RvizStatus {{
                color: {p['dim']};
                font-family: 'JetBrains Mono', monospace;
                font-size: 10px;
                letter-spacing: 1px;
                padding: 6px 0 0;
            }}
            QFrame#HistCard {{
                background: {p['log_bg']};
                border: 1px solid {p['border']};
                border-radius: 3px;
            }}
            QLabel#HistName {{
                color: {p['ink']};
                font-family: 'JetBrains Mono', monospace;
                font-size: 10px;
            }}
            QLabel#HistWhen {{
                color: {p['dim']};
                font-family: 'JetBrains Mono', monospace;
                font-size: 9px;
            }}
            QLabel#HistDetail {{
                color: {p['dim']};
                font-family: 'JetBrains Mono', monospace;
                font-size: 9px;
            }}
            QScrollArea#HistScroll {{ background: transparent; border: none; }}
            QScrollBar:vertical {{
                background: transparent;
                width: 8px;
                margin: 0;
            }}
            QScrollBar::handle:vertical {{
                background: {p['border_hi']};
                border-radius: 3px;
                min-height: 24px;
            }}
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0; }}
            QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {{ background: transparent; }}
            QLabel#ScanStateLbl {{
                color: {p['dim']};
                font-family: 'JetBrains Mono', monospace;
                font-size: 10px;
                font-weight: 700;
                letter-spacing: 1px;
            }}
        """)


# ───────────────────────── STANDALONE LAUNCHER ─────────────────────────

if __name__ == '__main__':
    import sys
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    gui = MissionControlGUI(theme='dark', density='comfortable', demo_mode=True)
    gui.resize(1440, 900)
    gui.show()
    sys.exit(app.exec_())
