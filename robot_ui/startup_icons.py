"""Consistent vector icons painted with QtGui; no optional SVG/font dependency."""
from PyQt6.QtCore import Qt, QRectF, QPointF
from PyQt6.QtGui import QColor, QIcon, QPainter, QPen, QPixmap, QPolygonF

LINES = {
    'waypoints': [[(5,9),(5,13),(12,22),(19,13),(19,9)]],
    'docking': [[(4,11),(20,11),(20,20),(4,20),(4,11)],[(8,11),(8,5),(16,5),(16,11)],[(13,13),(10,16),(14,16),(11,19)]],
    'load_map': [[(3,5),(9,3),(15,5),(21,3),(21,19),(15,21),(9,19),(3,21),(3,5)],[(9,3),(9,19)],[(15,5),(15,21)]],
    'new_map': [[(5,4),(14,4),(19,9),(19,20),(5,20),(5,4)],[(14,4),(14,9),(19,9)],[(9,14),(15,14)],[(12,11),(12,17)]],
    'tracking': [[(12,2),(12,5)],[(12,19),(12,22)],[(2,12),(5,12)],[(19,12),(22,12)]],
    'reestimate': [[(20,3),(20,8),(15,8)]],
    'nav2': [[(12,3),(20,20),(12,16),(4,20),(12,3)]],
    'language': [[(3,5),(13,5)],[(8,3),(8,5)],[(5,5),(7,10),(12,15)],[(11,5),(8,11),(3,15)],[(14,21),(18,11),(22,21)],[(16,17),(20,17)]],
    'developer': [[(8,6),(2,12),(8,18)],[(16,6),(22,12),(16,18)],[(14,3),(10,21)]],
    'mic': [[(5,10),(5,13),(7,17),(12,19),(17,17),(19,13),(19,10)],[(12,19),(12,22)],[(8,22),(16,22)]],
    'STM32': [[(9,2),(9,6)],[(15,2),(15,6)],[(9,18),(9,22)],[(15,18),(15,22)],[(2,9),(6,9)],[(2,15),(6,15)],[(18,9),(22,9)],[(18,15),(22,15)]],
    'front': [[(12,15),(12,22)]], 'rear': [[(12,2),(12,9)]],
}


def startup_icon(name, color='#21409a', size=24):
    pixmap = QPixmap(size * 2, size * 2)
    pixmap.fill(Qt.GlobalColor.transparent)
    painter = QPainter(pixmap)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing)
    painter.scale(size * 2 / 24, size * 2 / 24)
    painter.setPen(QPen(QColor(color), 1.7, Qt.PenStyle.SolidLine,
                        Qt.PenCapStyle.RoundCap, Qt.PenJoinStyle.RoundJoin))
    for points in LINES[name]:
        painter.drawPolyline(QPolygonF([QPointF(x, y) for x, y in points]))
    if name == 'mic':
        painter.drawRoundedRect(QRectF(9, 2, 6, 13), 3, 3)
    elif name == 'STM32':
        painter.drawRoundedRect(QRectF(6, 6, 12, 12), 2, 2)
        painter.drawRoundedRect(QRectF(9, 9, 6, 6), 1, 1)
    elif name == 'tracking':
        painter.drawEllipse(QRectF(5, 5, 14, 14))
        painter.drawEllipse(QRectF(10, 10, 4, 4))
    elif name == 'waypoints':
        painter.drawArc(QRectF(5, 2, 14, 14), 0, 180 * 16)
        painter.drawEllipse(QRectF(10, 7, 4, 4))
    elif name == 'reestimate':
        painter.drawArc(QRectF(4, 4, 16, 16), 30 * 16, 300 * 16)
    elif name in ('front', 'rear'):
        painter.drawEllipse(QRectF(9, 9, 6, 6))
        angle = 45 if name == 'front' else 225
        painter.drawArc(QRectF(4, 4, 16, 16), angle * 16, 90 * 16)
        painter.drawArc(QRectF(1, 1, 22, 22), angle * 16, 90 * 16)
    painter.end()
    pixmap.setDevicePixelRatio(2)
    return QIcon(pixmap)
