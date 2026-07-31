# -*- coding: utf-8 -*-
"""ApexFlight 电机位置示意图（v0.97）：Betaflight 风格四轴 X 布局。

- 四轴时按 BF QUAD X 编号：4 左前 / 2 右前 / 3 左后 / 1 右后
- 其他通道数按圆环均匀排布（从右后顺时针编号）
- 转动中的电机按油门大小高亮（深灰 → 品牌青色）
"""

import math

from PyQt6.QtCore import QPointF, Qt
from PyQt6.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PyQt6.QtWidgets import QWidget

# BF QUAD X 物理位置（俯视，机头朝上）：编号 → 归一化 (x, y)
_QUADX_POS = {
    2: (0.80, 0.22), 4: (0.20, 0.22),
    3: (0.20, 0.78), 1: (0.80, 0.78),
}


def _ring_pos(n: int) -> dict:
    """非四轴：n 个位置沿圆环均匀分布（从右后开始顺时针）"""
    pos = {}
    for i in range(n):
        ang = math.radians(45 + i * 360.0 / n)   # 45° 起 ≈ 右后
        pos[i + 1] = (0.5 + 0.3 * math.cos(ang), 0.5 + 0.3 * math.sin(ang))
    return pos


class MotorDiagramWidget(QWidget):
    """电机布局示意：机臂、编号圆圈、机头箭头、转动高亮"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(240, 210)
        self._count = 4
        self._values = {}                        # 编号 -> 0.0~1.0 油门

    def set_motor_count(self, n: int):
        self._count = max(1, n)
        self.update()

    def set_values(self, values: dict):
        self._values = dict(values)
        self.update()

    def _positions(self) -> dict:
        if self._count == 4:
            return _QUADX_POS
        return _ring_pos(self._count)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor("#14171B"))
        w, h = self.width(), self.height()
        cx, cy = w / 2, h / 2
        pos = self._positions()

        # 机臂（X 形粗线，圆角端头）
        p.setPen(QPen(QColor("#3A3F47"), 13, Qt.PenStyle.SolidLine,
                      Qt.PenCapStyle.RoundCap))
        for nx, ny in pos.values():
            p.drawLine(QPointF(cx, cy), QPointF(nx * w, ny * h))

        # 机头方向箭头（红色，BF 同款三角）
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor("#E04545"))
        aw, ah = 9, 22
        p.drawPolygon(QPolygonF([
            QPointF(cx, cy - ah - 14),
            QPointF(cx - aw, cy - 12),
            QPointF(cx + aw, cy - 12),
        ]))

        # 电机圆圈：底色随油门加深 → 品牌青
        r = 24
        for num, (nx, ny) in pos.items():
            mx, my = nx * w, ny * h
            v = max(0.0, min(1.0, self._values.get(num, 0.0)))
            base = QColor("#1E2228")
            hot = QColor("#3EC6E8")
            fill = QColor(
                int(base.red() + (hot.red() - base.red()) * v),
                int(base.green() + (hot.green() - base.green()) * v),
                int(base.blue() + (hot.blue() - base.blue()) * v))
            pen_color = QColor("#3EC6E8") if v > 0 else QColor("#8A9099")
            p.setPen(QPen(pen_color, 3))
            p.setBrush(fill)
            p.drawEllipse(QPointF(mx, my), r, r)
            p.setPen(QColor("#FFFFFF") if v > 0.05 else QColor("#C9CDD3"))
            p.setFont(QFont("Microsoft YaHei", 14, QFont.Weight.Bold))
            p.drawText(int(mx - r), int(my - r), r * 2, r * 2,
                       Qt.AlignmentFlag.AlignCenter, str(num))

        # 中央机体点
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor("#5A6069"))
        p.drawEllipse(QPointF(cx, cy), 7, 7)
        p.end()
