# -*- coding: utf-8 -*-
"""ApexFlight - 自定义控件（ToggleSwitch 开关、AttitudeIndicator 姿态仪）"""

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor, QPainter, QPainterPath, QPen
from PyQt6.QtWidgets import QAbstractButton, QWidget

# ============================================================
# 第五部分：自定义控件
# ============================================================

class ToggleSwitch(QAbstractButton):
    """
    胶囊开关（仿 Betaflight Configurator 的 toggle 样式）：
    关 = 灰色胶囊 + 白色圆点在左；开 = 橙色胶囊 + 白色圆点在右。
    用法和 QCheckBox 一样：isChecked() / setChecked() / toggled 信号。
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setCheckable(True)
        self.setFixedSize(38, 20)
        self.setCursor(Qt.CursorShape.PointingHandCursor)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        rect = self.rect().adjusted(1, 1, -1, -1)
        on = self.isChecked()

        # 胶囊背景（开 = 图标橙，关 = 深灰）
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(245, 168, 61) if on else QColor(54, 60, 68))
        painter.drawRoundedRect(rect, rect.height() / 2, rect.height() / 2)

        # 白色圆形滑块
        d = rect.height() - 4
        x = rect.right() - d - 2 if on else rect.left() + 2
        painter.setBrush(QColor(255, 255, 255))
        painter.drawEllipse(int(x), int(rect.top() + 2), int(d), int(d))
        painter.end()

    def sizeHint(self):
        return self.minimumSizeHint()


class AttitudeIndicator(QWidget):
    """
    人工地平线仪表：模拟真实飞行仪表，显示横滚和俯仰。
    蓝色 = 天空，棕色 = 地面，中间的线 = 地平线。
    """

    def __init__(self):
        super().__init__()
        self._roll = 0.0
        self._pitch = 0.0
        self.setMinimumSize(180, 180)

    def set_attitude(self, roll: float, pitch: float):
        """更新姿态角（单位：度）并重绘。
        v0.91：平滑插值——显示值向目标值渐进收敛，
        快速转动时仪表不再生硬跳变，观感更顺滑。"""
        self._roll += (roll - self._roll) * 0.45
        self._pitch += (pitch - self._pitch) * 0.45
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        size = int(min(self.width(), self.height()))
        painter.translate(self.width() / 2, self.height() / 2)

        # 圆形裁剪，画出仪表外形
        # 注意 1：QPainter 没有 setClipEllipse 方法（Qt4 老 API），
        #         Qt6 必须用 QPainterPath + setClipPath 实现椭圆裁剪
        # 注意 2：fillRect 等只接受整数坐标，浮点数会抛 TypeError
        radius = int(size / 2 - 4)
        clip_path = QPainterPath()
        clip_path.addEllipse(-radius, -radius, radius * 2, radius * 2)
        painter.setClipPath(clip_path)

        # 按横滚角旋转、按俯仰角上下平移整个天地
        # Betaflight 的角度约定与航空仪表相反：
        #   俯仰值为正 = 机头下压 → 应看到更多地面 → 地平线上移（取负号）
        #   横滚旋转方向同理取反，与 Configurator 的 3D 模型保持一致
        painter.save()
        painter.rotate(self._roll)
        pitch_pixels = int(max(-radius, min(radius, -self._pitch * 2)))

        # 天/地配色与 ApexFlight 图标一致：青色天空 + 深棕地面
        painter.fillRect(-size, -size * 2 + pitch_pixels,
                         size * 2, size * 2, QColor(62, 198, 232))   # 天空
        painter.fillRect(-size, pitch_pixels,
                         size * 2, size * 2, QColor(74, 56, 38))     # 地面
        # 地平线
        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.drawLine(-size, int(pitch_pixels), size, int(pitch_pixels))
        painter.restore()

        # 中央固定的飞机符号（图标橙色，不随姿态转动）
        painter.setPen(QPen(QColor(245, 168, 61), 4))
        painter.drawLine(-30, 0, -10, 0)
        painter.drawLine(10, 0, 30, 0)
        painter.drawLine(0, 0, 0, 6)

        # 外圈边框
        painter.setClipping(False)
        painter.setPen(QPen(QColor(120, 120, 120), 2))
        painter.drawEllipse(int(-radius), int(-radius),
                            int(radius * 2), int(radius * 2))
        painter.end()


