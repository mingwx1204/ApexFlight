# -*- coding: utf-8 -*-
"""v0.99 回归：BF 对齐电机页（µs 滑块 + 主控 + 读回竖条 + 安全锁）。

offscreen 运行：QT_QPA_PLATFORM=offscreen python tests/check_motor_tab.py
断言通过打印 MOTOR_TAB_OK。
"""

import os
import sys

os.environ["QT_QPA_PLATFORM"] = "offscreen"
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from PyQt6.QtWidgets import QApplication          # noqa: E402

app = QApplication([])

import main                                       # noqa: E402

win = main.MainWindow()

# 1. 模拟连接后拿到 4 个电机通道 → 4 列 + 主控 + 示意图 4 位
win.on_motor_count(4)
assert len(win._motor_sliders) == 4, "expect 4 sliders"
assert hasattr(win, "motor_master"), "master slider missing"
assert win.motor_diagram._count == 4
val_label, bar, slider, tele = win._motor_sliders[0]

# 2. 滑块为 µs 语义（BF：DShot 停转 1000，满油门 2000）
assert slider.minimum() == 1000 and slider.maximum() == 2000
assert slider.value() == 1000
assert win.motor_master.value() == 1000

# 3. 安全锁：未确认 / 未连接时全部禁用
assert not slider.isEnabled()
assert not win.motor_master.isEnabled()
win.motor_enable.setChecked(True)
assert not slider.isEnabled(), "not connected: keep locked"

# 4. 模拟已连接 → 解锁并启动读回轮询
class _W:
    is_connected = True
    def read_motor_values(self): pass
    def set_motor_values(self, values): pass

_old_worker = win.worker
win.worker = _W()
win._update_motor_lock()
assert slider.isEnabled(), "should unlock when connected"
assert win.motor_master.isEnabled()
assert win._motor_poll_timer.isActive(), "读回轮询应启动"

# 5. 主控驱动全部滑块，读数直接显示 µs（BF 语义）
win.motor_master.setValue(1120)
assert all(s.value() == 1120 for _, _, s, _ in win._motor_sliders)
assert win._motor_sliders[0][2].value() == 1120
assert win._motor_sliders[0][0].text() == "1000"  # 读数留给读回
assert win.motor_master_label.text() == "1120"
# 滑块下的 µs 标签
assert win._motor_sliders[0][2].value() == 1120

# 6. 发送映射：滑块值即 µs，补停转值到 8 通道
vals = win._current_motor_command()
assert vals == [1120, 1120, 1120, 1120, 1000, 1000, 1000, 1000], vals

# 7. 读回：飞控实际输出驱动竖条与读数（BF motorData 语义）
win.on_motor_values([1050, 1100, 1200, 1300])
assert win._motor_sliders[0][1].value() == 5     # (1050-1000)/10
assert win._motor_sliders[1][1].value() == 10
assert win._motor_sliders[2][1].value() == 20
assert win._motor_sliders[3][0].text() == "1300"

# 8. 停转电机 → 全部拉回 1000（BF stopAllMotors）
win.motor_master.setValue(1300)
win.on_motor_stop()
assert all(s.value() == 1000 for _, _, s, _ in win._motor_sliders)
assert win.motor_master.value() == 1000

# 9. 取消启用 → 轮询停止
win.motor_enable.setChecked(False)
assert not win._motor_poll_timer.isActive()

# 10. 8 通道（H743 全部输出）也能正常排布
win.on_motor_count(8)
assert len(win._motor_sliders) == 8
assert win.motor_diagram._count == 8

win.worker = _old_worker
print("MOTOR_TAB_OK")
