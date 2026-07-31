# -*- coding: utf-8 -*-
"""v0.97 回归：BF 风格电机页（竖滑块 + 主控制 + 位置示意图 + 安全锁）。

offscreen 运行：QT_QPA_PLATFORM=offscreen python tests/check_motor_tab.py
断言通过打印 MOTOR_TAB_OK。
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from PyQt6.QtWidgets import QApplication          # noqa: E402

app = QApplication([])

import main                                       # noqa: E402

win = main.MainWindow()

# 1. 模拟连接后拿到 4 个电机通道 → 4 滑块 + 主控制 + 示意图 4 位
win.on_motor_count(4)
assert len(win._motor_sliders) == 4, "expect 4 sliders"
assert hasattr(win, "motor_master"), "master slider missing"
assert win.motor_diagram._count == 4

# 2. 安全锁：未确认 / 未连接时全部禁用
assert not win._motor_sliders[0][1].isEnabled()
assert not win.motor_master.isEnabled()
win.motor_check1.setChecked(True)
win.motor_check2.setChecked(True)
assert not win._motor_sliders[0][1].isEnabled(), "not connected: keep locked"

# 3. 模拟已连接 → 解锁
class _W:
    is_connected = True

_old_worker = win.worker
win.worker = _W()
win._update_motor_lock()
assert win._motor_sliders[0][1].isEnabled(), "should unlock when connected"
assert win.motor_master.isEnabled()

# 4. 主控制驱动全部滑块；读数按 BF 惯例显示 µs（0 或 1000+value）
win.motor_master.setValue(120)
assert all(s.value() == 120 for _, s in win._motor_sliders)
assert win._motor_sliders[0][0].text() == "1120"
assert win.motor_master_label.text() == "1120"

# 5. 发送映射：滑块 0 → 发 0（停转），其余 → 1000+value
win._motor_sliders[1][1].setValue(0)
vals = [(0 if s.value() == 0 else 1000 + s.value())
        for _, s in win._motor_sliders]
assert vals == [1120, 0, 1120, 1120], vals

# 6. 停转电机 → 全部归零
win.motor_master.setValue(300)
win.on_motor_stop()
assert all(s.value() == 0 for _, s in win._motor_sliders)
assert win.motor_master.value() == 0
assert win._motor_sliders[0][0].text() == "0"

# 7. 8 通道（H743 全部输出）也能正常排布
win.on_motor_count(8)
assert len(win._motor_sliders) == 8
assert win.motor_diagram._count == 8

win.worker = _old_worker
print("MOTOR_TAB_OK")
