"""Original, offline vector companion. No robot, voice, or network dependencies."""
import math
import time

from PyQt6.QtCore import QPointF, QRectF, Qt, QTimer
from PyQt6.QtGui import QColor, QLinearGradient, QPainter, QPainterPath, QPen
from PyQt6.QtWidgets import QApplication, QSizePolicy, QWidget


class MangoCompanion(QWidget):
    """A bounded mouse/keyboard playground painted in a 240 × 260 art space."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(0, 110)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.setCursor(Qt.CursorShape.OpenHandCursor)
        self.state = "idle"
        self._position = QPointF(.5, .5)
        self._press = None
        self._offset = QPointF()
        self._started = time.monotonic()
        self._state_started = self._started
        self._timer = QTimer(self)
        self._timer.setInterval(33)
        self._timer.timeout.connect(self._tick)
        self.set_language("vi")

    def set_language(self, language):
        hint = ("Mango: click or press Space to play; drag to move."
                if language == "en" else
                "Xoài: bấm hoặc nhấn phím cách để chơi; kéo để di chuyển.")
        self.setAccessibleName("Mango companion" if language == "en" else "Bạn Xoài")
        self.setAccessibleDescription(hint)
        self.setToolTip(hint)

    def _set_state(self, state):
        self.state = state
        self._state_started = time.monotonic()
        self.update()

    def _tick(self):
        if self.state in ("happy", "released") and time.monotonic() - self._state_started >= 1.1:
            self._set_state("idle")
        self.update()

    def showEvent(self, event):
        super().showEvent(event)
        self._timer.start()

    def hideEvent(self, event):
        self._timer.stop()
        self._press = None
        self.setCursor(Qt.CursorShape.OpenHandCursor)
        self._set_state("idle")
        super().hideEvent(event)

    def character_rect(self):
        # Leave room on all sides for rotation, bobbing, and raised hands.
        scale = max(0., min((self.width() - 16) / 280, (self.height() - 16) / 300, 1.25))
        width, height = 240 * scale, 260 * scale
        margin = 8 + 20 * scale
        x = margin + max(0., self.width() - width - 2 * margin) * self._position.x()
        y = margin + max(0., self.height() - height - 2 * margin) * self._position.y()
        return QRectF(x, y, width, height)

    def _hit(self, point):
        rect = self.character_rect()
        if rect.width() <= 0:
            return False
        p = (point - rect.topLeft()) / (rect.width() / 240)
        # Exclude empty corners of the artwork's rectangle.
        return (((p.x() - 120) / 104) ** 2 + ((p.y() - 138) / 122) ** 2) <= 1

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton and self._hit(event.position()):
            self.setFocus(Qt.FocusReason.MouseFocusReason)
            self._press = event.position()
            self._offset = event.position() - self.character_rect().center()
            self.setCursor(Qt.CursorShape.ClosedHandCursor)
            self._set_state("touched")
        event.accept()

    def mouseMoveEvent(self, event):
        if self._press is not None:
            if self.state != "dragging" and (event.position() - self._press).manhattanLength() >= QApplication.startDragDistance():
                self._set_state("dragging")
            if self.state == "dragging":
                rect = self.character_rect()
                scale = rect.width() / 240
                margin = 8 + 20 * scale
                center = event.position() - self._offset
                span_x = self.width() - rect.width() - 2 * margin
                span_y = self.height() - rect.height() - 2 * margin
                self._position = QPointF(
                    min(1., max(0., (center.x() - margin - rect.width() / 2) / span_x)) if span_x > 0 else .5,
                    min(1., max(0., (center.y() - margin - rect.height() / 2) / span_y)) if span_y > 0 else .5,
                )
                self.update()
        event.accept()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton and self._press is not None:
            self._press = None
            self.setCursor(Qt.CursorShape.OpenHandCursor)
            self._set_state("released" if self.state == "dragging" else "happy")
        event.accept()

    def keyPressEvent(self, event):
        if event.key() in (Qt.Key.Key_Space, Qt.Key.Key_Return, Qt.Key.Key_Enter) and self._press is None:
            if not event.isAutoRepeat():
                self._set_state("happy")
            event.accept()
        else:
            super().keyPressEvent(event)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        rect = self.character_rect()
        if rect.width() <= 0:
            return
        painter.translate(rect.topLeft())
        painter.scale(rect.width() / 240, rect.height() / 260)
        elapsed = time.monotonic() - self._started
        age = time.monotonic() - self._state_started
        bob = math.sin(elapsed * 2.5) * 3
        if self.state in ("released", "happy"):
            bob -= abs(math.sin(age * 10)) * 10 * max(0., 1 - age / 1.1)

        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(10, 28, 74, 65))
        painter.drawEllipse(QRectF(52, 240, 142, 12))
        painter.translate(120, 135 + bob)
        painter.rotate(-5 if self.state == "touched" else 5 if self.state == "dragging" else math.sin(elapsed * 2) * 2)
        painter.translate(-120, -135)

        def curve(points, color, width):
            path = QPainterPath(QPointF(*points[0]))
            path.cubicTo(*points[1], *points[2], *points[3])
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.setPen(QPen(QColor(color), width, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
            painter.drawPath(path)

        outline = "#754424"
        wave = math.sin(elapsed * 3) * 5
        lifted = self.state in ("happy", "dragging", "released")
        curve([(62, 142), (30, 150), (25, 165), (18, 157 + wave)], outline, 9)
        curve([(181, 135), (214, 148), (225, 100 if lifted else 155), (222, 88 + wave if lifted else 159 + wave)], outline, 9)
        curve([(93, 205), (90, 227), (93, 240), (77, 240)], outline, 11)
        curve([(151, 201), (156, 223), (153, 240), (170, 240)], outline, 11)
        body = QPainterPath(QPointF(123, 43))
        body.cubicTo(192, 30, 211, 97, 185, 164)
        body.cubicTo(169, 211, 123, 233, 79, 207)
        body.cubicTo(29, 178, 40, 112, 69, 80)
        body.cubicTo(88, 60, 99, 49, 123, 43)
        gradient = QLinearGradient(65, 70, 182, 209)
        gradient.setColorAt(0, QColor("#ffe77b"))
        gradient.setColorAt(.55, QColor("#ffc43d"))
        gradient.setColorAt(1, QColor("#f29132"))
        painter.setBrush(gradient)
        painter.setPen(QPen(QColor(outline), 4))
        painter.drawPath(body)
        curve([(128, 48), (128, 36), (136, 29), (137, 22)], outline, 6)
        leaf = QPainterPath(QPointF(134, 33))
        leaf.cubicTo(149, 0, 184, 11, 190, 16)
        leaf.cubicTo(177, 42, 150, 49, 134, 33)
        painter.setBrush(QColor("#69bd73"))
        painter.setPen(QPen(QColor("#285b46"), 3))
        painter.drawPath(leaf)
        curve([(138, 33), (152, 30), (161, 23), (176, 20)], "#285b46", 2)

        closed = self.state == "touched" or elapsed % 4.2 < .16
        for x in (91, 149):
            if closed or self.state == "happy":
                curve([(x - 8, 120), (x - 3, 113), (x + 3, 113), (x + 8, 120)], outline, 4)
            else:
                painter.setPen(Qt.PenStyle.NoPen)
                painter.setBrush(QColor("#3f342d"))
                painter.drawEllipse(QRectF(x - 6, 108, 12, 21))
                painter.setBrush(QColor("#ffffff"))
                painter.drawEllipse(QRectF(x - 3, 111, 4, 6))
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QColor("#f49a68"))
            painter.drawEllipse(QRectF(x - 15, 137, 24, 11))
        if self.state == "dragging":
            painter.setBrush(QColor(outline))
            painter.drawEllipse(QRectF(112, 143, 14, 19))
        else:
            curve([(101, 149), (110, 167 if lifted else 160), (129, 167 if lifted else 160), (139, 147)], outline, 4)
