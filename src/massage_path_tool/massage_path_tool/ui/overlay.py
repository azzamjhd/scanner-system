"""Custom QGraphicsItem helpers for drawing the anchor quad, points, paths,
and an invalid-pose banner in the massage_path_tool scene."""

from PyQt6.QtCore import Qt, QRectF, QPointF
from PyQt6.QtGui import QPen, QColor, QBrush, QPolygonF, QPainterPath, QFont
from PyQt6.QtWidgets import (
    QGraphicsEllipseItem,
    QGraphicsPolygonItem,
    QGraphicsPathItem,
    QGraphicsTextItem,
    QGraphicsRectItem,
    QGraphicsItemGroup,
)

# --- Style constants ---------------------------------------------------------

QUAD_PEN = QPen(QColor(255, 0, 255), 2, Qt.PenStyle.DashLine)

POINT_BRUSH = QBrush(QColor(255, 165, 0))

POINT_PEN = QPen(QColor(0, 0, 0), 1)

PATH_PEN = QPen(QColor(0, 255, 255), 2, Qt.PenStyle.SolidLine)
PATH_PEN.setCapStyle(Qt.PenCapStyle.RoundCap)
PATH_PEN.setJoinStyle(Qt.PenJoinStyle.RoundJoin)

POINT_RADIUS = 6  # scene units


# --- Anchor quad -------------------------------------------------------------

def make_anchor_quad_item(pts) -> QGraphicsPolygonItem:
    """Build a dashed magenta polygon from 4 TL/TR/BR/BL pixel points."""
    polygon = QPolygonF()
    for i in range(4):
        polygon.append(QPointF(float(pts[i][0]), float(pts[i][1])))
    item = QGraphicsPolygonItem(polygon)
    item.setPen(QUAD_PEN)
    item.setBrush(QBrush(Qt.BrushStyle.NoBrush))
    return item


# --- Points ------------------------------------------------------------------

class PointItem(QGraphicsEllipseItem):
    """An orange filled circle with a small text label."""

    def __init__(self, x, y, label: str, radius=POINT_RADIUS):
        x = float(x)
        y = float(y)
        super().__init__(QRectF(x - radius, y - radius, 2 * radius, 2 * radius))
        self.label = label
        self.cx = x
        self.cy = y
        self.setBrush(POINT_BRUSH)
        self.setPen(POINT_PEN)

        text = QGraphicsTextItem(label, self)
        font = QFont()
        font.setPointSize(8)
        text.setFont(font)
        text.setDefaultTextColor(QColor(240, 240, 240))
        text.setPos(x + radius + 2, y - radius)


# --- Path --------------------------------------------------------------------

def make_path_item(pts) -> QGraphicsPathItem:
    """Build a cyan polyline path through the given pixel points."""
    path = QPainterPath()
    path.moveTo(float(pts[0][0]), float(pts[0][1]))
    for i in range(1, len(pts)):
        path.lineTo(float(pts[i][0]), float(pts[i][1]))
    item = QGraphicsPathItem(path)
    item.setPen(PATH_PEN)
    return item


# --- Invalid-pose banner -----------------------------------------------------

def make_invalid_banner_item(scene_rect: QRectF,
                             text='INVALID POSE GEOMETRY') -> QGraphicsItemGroup:
    """Build a semi-transparent red overlay with a centered warning label."""
    group = QGraphicsItemGroup()

    rect_item = QGraphicsRectItem(scene_rect)
    rect_item.setBrush(QBrush(QColor(255, 0, 0, 90)))
    rect_item.setPen(QPen(Qt.PenStyle.NoPen))
    group.addToGroup(rect_item)

    text_item = QGraphicsTextItem(text)
    font = QFont()
    font.setPointSize(24)
    font.setBold(True)
    text_item.setFont(font)
    text_item.setDefaultTextColor(QColor(255, 255, 255))
    text_width = text_item.boundingRect().width()
    x = scene_rect.left() + (scene_rect.width() - text_width) / 2.0
    y = scene_rect.top() + 10.0
    text_item.setPos(x, y)
    group.addToGroup(text_item)

    return group
