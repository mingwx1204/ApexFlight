# -*- coding: utf-8 -*-
"""电机测试页（v0.99 从 main.py 拆出）：对齐 BF 最新 MotorsTab"""

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import (
    QCheckBox, QFrame, QGroupBox, QHBoxLayout, QLabel,
    QProgressBar, QPushButton, QSlider, QVBoxLayout, QWidget)

from apex_fc import set_motors
from apex_i18n import tr
from apex_log import log_event

# DShot 停转 / 满油门（µs），对齐 BF MotorsTab
MOTOR_MIN_US = 1000
MOTOR_MAX_US = 2000

class MotorTabMixin:
    """电机测试页全部 UI 与处理器（self 即 MainWindow）"""
    def _build_motor_tab(self) -> QWidget:
        """电机测试页（v0.99 对齐 BF Configurator 最新 MotorsTab.vue）：
        - 滑块直接以 µs 为单位（DShot 停转=1000，满油门=2000）
        - 行序同 BF：编号 → 竖条+读数 → 遥测 → 滑块（末列主控橙色总滑块）
        - 测试期间 5Hz 读回飞控实际输出（MSP_MOTOR），竖条显示真实值
        - 红色危险框 + 单一启用开关（BF 同款交互）"""
        tab = QWidget()
        layout = QVBoxLayout(tab)

        # 滑块去抖定时器：拖动停止 100ms 后才真正发送 MSP_SET_MOTOR
        self._motor_timer = QTimer(self)
        self._motor_timer.setSingleShot(True)
        self._motor_timer.setInterval(100)
        self._motor_timer.timeout.connect(self._send_motor_values)
        self._motor_values = {}                   # 编号 -> 当前油门 0..1
        # 读回轮询：仅在电机测试启用期间运行（对标 BF motorData polling）
        self._motor_poll_timer = QTimer(self)
        self._motor_poll_timer.setInterval(200)
        self._motor_poll_timer.timeout.connect(
            lambda: self._run_in_thread(self.worker.read_motor_values))

        # 顶部：红色危险框（BF 同款：警告说明 + 启用开关）
        danger = QGroupBox()
        danger.setStyleSheet(
            "QGroupBox { border: 1px solid #5A2A2E; border-radius: 8px;"
            " background: #241B1D; margin-top: 4px; }")
        drow = QHBoxLayout(danger)
        warning = QLabel("⚠️ 危险：电机测试会让电机真实转动！\n"
                         "使用前必须【拆下所有螺旋桨】，确认飞机固定牢固、"
                         "周围没有人员和杂物。")
        warning.setStyleSheet("color: #E04545; font-weight: bold;"
                              " border: none; background: transparent;")
        warning.setWordWrap(True)
        drow.addWidget(warning, 1)
        self.motor_enable = QCheckBox(tr("我已拆下螺旋桨，启用电机测试"))
        self.motor_enable.setStyleSheet(
            "font-weight: bold; border: none; background: transparent;")
        self.motor_enable.stateChanged.connect(self._update_motor_lock)
        drow.addWidget(self.motor_enable)
        layout.addWidget(danger)

        # 主体：左示意图 + 右滑块排
        body = QHBoxLayout()

        from apex_motor import MotorDiagramWidget
        left_box = QGroupBox(tr("电机位置示意（编号与 BF 一致）"))
        left_box.setMaximumWidth(340)
        left_col = QVBoxLayout(left_box)
        self.motor_diagram = MotorDiagramWidget()
        left_col.addWidget(self.motor_diagram)
        hint = QLabel(tr("QUAD X：1 右后 / 2 右前 / 3 左后 / 4 左前\n"
                         "转动中的电机随油门高亮"))
        hint.setStyleSheet("color: #9AA0A6; font-size: 13px;")
        hint.setAlignment(Qt.AlignmentFlag.AlignCenter)
        left_col.addWidget(hint)
        body.addWidget(left_box, 0)

        # 右侧：滑块排（连接后按实际通道数动态生成，BF 风格竖滑块）
        self.motor_area = QGroupBox("电机输出（未连接）")
        self.motor_layout = QHBoxLayout(self.motor_area)
        self.motor_layout.setSpacing(12)
        body.addWidget(self.motor_area, 1)
        layout.addLayout(body, 1)

        # 底部：停转电机（右对齐，BF 同款位置）
        bottom = QHBoxLayout()
        bottom.addStretch()
        self.motor_stop_btn = QPushButton("🛑 " + tr("停转电机"))
        self.motor_stop_btn.setObjectName("dangerBtn")
        self.motor_stop_btn.setEnabled(False)
        self.motor_stop_btn.setMinimumWidth(160)
        self.motor_stop_btn.clicked.connect(self.on_motor_stop)
        bottom.addWidget(self.motor_stop_btn)
        layout.addLayout(bottom)
        return tab


    def on_motor_count(self, count: int):
        """根据电机通道数动态生成 BF 风格滑块排。
        列序同 BF MotorsTab：编号 → 读数 → 竖条 → 遥测 → 滑块 → µs 值。
        滑块直接以 µs 为单位（1000=DShot 停转，2000=满油门）。"""
        self._motor_count = count
        self.motor_area.setTitle(f"电机输出（{count} 个通道）")
        self.motor_diagram.set_motor_count(count)
        # 清空旧滑块（含子布局里的列）
        while self.motor_layout.count():
            item = self.motor_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
            elif item.layout():
                while item.layout().count():
                    sub = item.layout().takeAt(0)
                    if sub.widget():
                        sub.widget().deleteLater()
        self._motor_sliders = []              # [(值标签, 竖条, 滑块, 遥测标签)]
        self._motor_values = {}
        for i in range(count):
            col = QVBoxLayout()
            num_label = QLabel(str(i + 1))
            num_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            num_label.setStyleSheet("font-weight: bold; font-size: 16px;")
            val_label = QLabel(str(MOTOR_MIN_US))
            val_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            val_label.setStyleSheet("color: #3EC6E8; font-size: 14px;")
            bar = QProgressBar()
            bar.setOrientation(Qt.Orientation.Vertical)
            bar.setRange(0, 100)
            bar.setValue(0)
            bar.setTextVisible(False)
            bar.setFixedWidth(26)
            bar.setMinimumHeight(90)
            tele_label = QLabel("—")
            tele_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            tele_label.setStyleSheet("color: #9AA0A6; font-size: 11px;")
            tele_label.setToolTip(
                tr("RPM / 错误率 / 温度（需开启双向 DShot，后续版本接入）"))
            slider = QSlider(Qt.Orientation.Vertical)
            slider.setRange(MOTOR_MIN_US, MOTOR_MAX_US)
            slider.setValue(MOTOR_MIN_US)
            slider.setSingleStep(25)          # BF 同款滚轮步进
            slider.setPageStep(50)
            slider.setMinimumHeight(150)
            slider.setEnabled(False)
            us_label = QLabel(str(MOTOR_MIN_US))
            us_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            us_label.setStyleSheet("font-size: 12px; color: #C9CDD3;")
            slider.valueChanged.connect(
                lambda v, idx=i, lab=us_label:
                self._on_motor_slider(idx, v, lab))
            col.addWidget(num_label)
            col.addWidget(val_label)
            col.addWidget(bar, 0, Qt.AlignmentFlag.AlignHCenter)
            col.addWidget(tele_label)
            col.addWidget(slider, 1, Qt.AlignmentFlag.AlignHCenter)
            col.addWidget(us_label)
            self.motor_layout.addLayout(col)
            self._motor_sliders.append((val_label, bar, slider, tele_label))
        # 分隔线 + 主控制（BF 同款：橙色总滑块同时驱动全部电机）
        line = QFrame()
        line.setFrameShape(QFrame.Shape.VLine)
        line.setStyleSheet("color: #3A3F47;")
        self.motor_layout.addWidget(line)
        mcol = QVBoxLayout()
        mspace = QLabel(tr("主控"))
        mspace.setAlignment(Qt.AlignmentFlag.AlignCenter)
        mspace.setStyleSheet("font-weight: bold; font-size: 16px;"
                             " color: #F5A83D;")
        self.motor_master_label = QLabel(str(MOTOR_MIN_US))
        self.motor_master_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.motor_master_label.setStyleSheet(
            "color: #F5A83D; font-size: 14px;")
        self.motor_master = QSlider(Qt.Orientation.Vertical)
        self.motor_master.setRange(MOTOR_MIN_US, MOTOR_MAX_US)
        self.motor_master.setValue(MOTOR_MIN_US)
        self.motor_master.setSingleStep(25)
        self.motor_master.setPageStep(50)
        self.motor_master.setMinimumHeight(150)
        self.motor_master.setEnabled(False)
        self.motor_master.valueChanged.connect(self._on_master_slider)
        mcol.addWidget(mspace)
        mcol.addWidget(self.motor_master_label)
        mcol.addWidget(QLabel(""))
        mcol.addWidget(self.motor_master, 1,
                       Qt.AlignmentFlag.AlignHCenter)
        mcol.addWidget(QLabel(""))
        self.motor_layout.addLayout(mcol)
        self.motor_layout.addStretch()
        self._update_motor_lock()


    def _update_motor_lock(self):
        """BF 同款：启用开关勾选且已连接才能动滑块；
        启用期间 5Hz 读回飞控实际输出，停用时先停转再停止轮询"""
        unlocked = (self.motor_enable.isChecked()
                    and self.worker.is_connected)
        for _, _, slider, _ in self._motor_sliders:
            slider.setEnabled(unlocked)
        if hasattr(self, "motor_master"):
            self.motor_master.setEnabled(unlocked)
        self.motor_stop_btn.setEnabled(unlocked)
        if unlocked:
            self._motor_poll_timer.start()
            log_event("电机测试已启用（读回轮询 5Hz）")
        else:
            self._motor_poll_timer.stop()
            # BF 安全纪律：停用时把所有输出拉回停转值
            if self.worker.is_connected and self._motor_count:
                self._send_motor_stop()


    def _on_motor_slider(self, index: int, value: int, label: QLabel):
        """滑块变化：µs 读数 + 示意图高亮 + 重启去抖定时器"""
        label.setText(str(value))
        self._motor_values[index + 1] = \
            (value - MOTOR_MIN_US) / (MOTOR_MAX_US - MOTOR_MIN_US)
        self.motor_diagram.set_values(self._motor_values)
        self._motor_timer.start()


    def _on_master_slider(self, value: int):
        """主控制：把所有电机滑块拉到同一值（各滑块自己去抖发送）"""
        self.motor_master_label.setText(str(value))
        for _, _, slider, _ in self._motor_sliders:
            slider.setValue(value)      # 触发各自 _on_motor_slider


    def _current_motor_command(self) -> list:
        """当前滑块 µs 值，补齐到协议固定的 8 通道（补停转值）"""
        values = [s.value() for _, _, s, _ in self._motor_sliders]
        values += [MOTOR_MIN_US] * (8 - len(values))
        return values[:8]


    def _send_motor_values(self):
        """去抖定时器触发：把当前全部滑块 µs 值发给飞控"""
        self._run_in_thread(self.worker.set_motor_values,
                            self._current_motor_command())


    def _send_motor_stop(self):
        """立即发送全部停转（BF stopAllMotors 同款）"""
        self._run_in_thread(self.worker.set_motor_values,
                            [MOTOR_MIN_US] * 8)


    def on_motor_stop(self):
        """停转电机：所有滑块（含主控）拉回停转值并立即发送"""
        for _, _, slider, _ in self._motor_sliders:
            slider.setValue(MOTOR_MIN_US)
        if hasattr(self, "motor_master"):
            self.motor_master.setValue(MOTOR_MIN_US)
        self._send_motor_stop()


    def on_motor_values(self, values: list):
        """读回飞控实际输出：竖条+读数显示真实值（对标 BF motorData）"""
        for i, (val_label, bar, _, _) in enumerate(self._motor_sliders):
            if i >= len(values):
                break
            v = values[i]
            pct = (v - MOTOR_MIN_US) * 100 // (MOTOR_MAX_US - MOTOR_MIN_US)
            bar.setValue(max(0, min(100, pct)))
            val_label.setText(str(v))


    def _stop_motors_safely(self):
        """断开/关闭前尝试把所有电机停掉"""
        if self.worker.is_connected and self._motor_count:
            try:
                set_motors(self.worker.serial_port, [MOTOR_MIN_US] * 8)
            except Exception:
                pass

