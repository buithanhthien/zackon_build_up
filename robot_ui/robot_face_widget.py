import math
from PyQt6.QtWidgets import QWidget
from PyQt6.QtCore import QTimer, QRect, Qt
from PyQt6.QtGui import QPainter, QColor, QPen, QBrush

# Emotions: happy, sad, neutral, thinking, speaking, listening
EMOTIONS = {
    "happy":     {"face": "#4fc3f7", "cheek": "#f48fb1", "mouth_arc": 180,  "eye_shape": "normal"},
    "sad":       {"face": "#90a4ae", "cheek": "#b0bec5", "mouth_arc": -180, "eye_shape": "sad"},
    "neutral":   {"face": "#81d4fa", "cheek": "#b0bec5", "mouth_arc": 0,    "eye_shape": "normal"},
    "thinking":  {"face": "#ce93d8", "cheek": "#e1bee7", "mouth_arc": 90,   "eye_shape": "thinking"},
    "speaking":  {"face": "#a5d6a7", "cheek": "#c8e6c9", "mouth_arc": 180,  "eye_shape": "normal"},
    "listening": {"face": "#fff176", "cheek": "#fff9c4", "mouth_arc": 60,   "eye_shape": "listening"},

}


class RobotFaceWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(180, 200)
        self._emotion = "neutral"
        self._tick = 0
        self._wave_phase = 0.0
        self._mouth_open = 0.0   # for speaking animation
        self._mouth_dir = 1

        timer = QTimer(self)
        timer.timeout.connect(self._tick_update)
        timer.start(30)  # ~33fps

    def set_emotion(self, emotion: str):
        if emotion in EMOTIONS:
            self._emotion = emotion

    def _tick_update(self):
        self._tick += 1
        self._wave_phase += 0.15
        if self._emotion == "speaking":
            self._mouth_open += 0.18 * self._mouth_dir
            if self._mouth_open >= 1.0 or self._mouth_open <= 0.0:
                self._mouth_dir *= -1
        else:
            self._mouth_open = 0.0
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        cfg = EMOTIONS[self._emotion]
        cx = self.width() // 2
        cy = self.height() // 2 - 10

        # Idle bob
        bob = int(math.sin(self._tick * 0.06) * 3)
        cy += bob

        # ── Face circle ──────────────────────────────────────────
        face_color = QColor(cfg["face"])
        p.setBrush(QBrush(face_color))
        p.setPen(QPen(QColor("#37474f"), 2))
        r = 60
        p.drawEllipse(cx - r, cy - r, r * 2, r * 2)

        # ── Antenna ──────────────────────────────────────────────
        p.setPen(QPen(QColor("#37474f"), 2))
        p.drawLine(cx, cy - r, cx, cy - r - 16)
        p.setBrush(QBrush(QColor("#ef5350")))
        p.drawEllipse(cx - 5, cy - r - 22, 10, 10)

        # ── Cheeks ───────────────────────────────────────────────
        cheek = QColor(cfg["cheek"])
        cheek.setAlpha(160)
        p.setBrush(QBrush(cheek))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(cx - 46, cy + 10, 20, 12)
        p.drawEllipse(cx + 26, cy + 10, 20, 12)

        # ── Eyes ─────────────────────────────────────────────────
        p.setPen(QPen(QColor("#37474f"), 2))
        p.setBrush(QBrush(QColor("#37474f")))
        blink = (self._tick % 150 < 4) and self._emotion not in ("thinking", "listening")
        eye_h = 2 if blink else 14

        shape = cfg["eye_shape"]
        if shape == "sad":
            # Sad: draw downward arc for each eye
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.setPen(QPen(QColor("#37474f"), 3))
            p.drawArc(QRect(cx - 40, cy - 26, 18, 14), 0 * 16, -180 * 16)
            p.drawArc(QRect(cx + 22, cy - 26, 18, 14), 0 * 16, -180 * 16)
        elif shape == "thinking":
            # One eye normal, one squinted
            p.setBrush(QBrush(QColor("#37474f")))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawEllipse(cx - 38, cy - 24, 16, eye_h)
            p.drawEllipse(cx + 22, cy - 24, 16, 6)   # squint
        elif shape == "listening":
            # Wide eyes
            p.setBrush(QBrush(QColor("#37474f")))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawEllipse(cx - 40, cy - 26, 20, 18)
            p.drawEllipse(cx + 20, cy - 26, 20, 18)
            # Highlight
            p.setBrush(QBrush(QColor("#ffffff")))
            p.drawEllipse(cx - 34, cy - 24, 6, 6)
            p.drawEllipse(cx + 26, cy - 24, 6, 6)

        else:
            p.setBrush(QBrush(QColor("#37474f")))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawEllipse(cx - 38, cy - 24, 16, eye_h)
            p.drawEllipse(cx + 22, cy - 24, 16, eye_h)
            if not blink:
                p.setBrush(QBrush(QColor("#ffffff")))
                p.drawEllipse(cx - 34, cy - 22, 5, 5)
                p.drawEllipse(cx + 26, cy - 22, 5, 5)

        # ── Mouth ────────────────────────────────────────────────
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setPen(QPen(QColor("#37474f"), 3))
        arc = cfg["mouth_arc"]

        if self._emotion == "speaking":
            # Animated open/close
            h = int(6 + self._mouth_open * 16)
            p.drawArc(QRect(cx - 18, cy + 16, 36, h), 0, -180 * 16)
        elif arc == 0:
            # Flat mouth
            p.drawLine(cx - 18, cy + 22, cx + 18, cy + 22)
        else:
            span = arc * 16
            p.drawArc(QRect(cx - 18, cy + 10, 36, 20), 0, span)

        # ── Wave effect (sound bars) ──────────────────────────────
        if self._emotion in ("speaking", "listening"):
            self._draw_wave(p, cx, cy + r + 18)

        p.end()

    def _draw_wave(self, p: QPainter, cx: int, y: int):
        bars = 7
        bar_w = 5
        gap = 4
        total_w = bars * (bar_w + gap) - gap
        x0 = cx - total_w // 2

        color = QColor("#4fc3f7") if self._emotion == "listening" else QColor("#a5d6a7")
        p.setPen(Qt.PenStyle.NoPen)

        for i in range(bars):
            h = int(6 + 10 * abs(math.sin(self._wave_phase + i * 0.7)))
            bx = x0 + i * (bar_w + gap)
            p.setBrush(QBrush(color))
            p.drawRoundedRect(bx, y - h // 2, bar_w, h, 2, 2)
