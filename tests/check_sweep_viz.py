# -*- coding: utf-8 -*-
"""v1.0 扫频伯德图可视化回归：峰值/带宽/PM 标记、轴切换、悬浮数据、截图。

offscreen 运行：QT_QPA_PLATFORM=offscreen python tests/check_sweep_viz.py
断言通过打印 SWEEP_VIZ_OK。
"""

import math
import os
import sys

os.environ["QT_QPA_PLATFORM"] = "offscreen"
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import numpy as np                                # noqa: E402
from PyQt6.QtWidgets import QApplication          # noqa: E402

app = QApplication([])

import main                                       # noqa: E402
import apex_sweep                                 # noqa: E402

win = main.MainWindow()
win.show()
app.processEvents()
win._ensure_page(7)          # 扫频页（懒加载）
win._ensure_page(6)          # 黑匣子页（截图按钮断言用）

# ---- 构造带已知谐振峰的辨识结果 ----
FS = 2000.0
t = np.arange(0, 12.0, 1.0 / FS)
k = math.log(250.0 / 2.0)
phase = 2 * math.pi * 2.0 * t[-1] / k * (np.exp(k * t / t[-1]) - 1)
u = 300.0 * np.sin(phase)


def plant(u, wn_hz, zeta):
    wn = 2 * math.pi * wn_hz
    T = 1.0 / FS
    x1 = x2 = 0.0
    y = np.zeros_like(u)
    for i in range(len(u)):
        x1 += T * x2
        x2 += T * (wn * wn * (u[i] - x1) - 2 * zeta * wn * x2)
        y[i] = x1
    return y


m_roll = apex_sweep.analyze_axis(u, plant(u, 40.0, 0.55), FS)
m_roll["resonances"] = [(185.0, 8.2), (320.0, 4.1)]   # 注入已知谐振
result = {"axes": {"横滚": m_roll}, "suggestions": []}
win._sweep_show_result(result)
app.processEvents()

ax1, ax2, ax3 = win._sweep_axes

# 1. 三子图 + 曲线
assert len(win.sweep_figure.axes) == 3
assert len(ax1.lines) >= 1, "幅值曲线缺失"

# 2. 谐振标记：红圈 + "共振" 标注 + 危险区底纹
ann_texts = [a.get_text() for a in ax1.texts]
assert any("185Hz 共振" in t for t in ann_texts), f"谐振标注缺失: {ann_texts}"
assert any("320Hz 共振" in t for t in ann_texts), "第二谐振标注缺失"
assert len(ax1.patches) >= 2, "危险区间底纹缺失"

# 3. 带宽标注 + 相位裕度点
bw = m_roll["bandwidth_hz"]
assert any(f"带宽 {bw:.0f}Hz" in t for t in ann_texts), "带宽标注缺失"
pm = m_roll["phase_margin_deg"]
ann2 = [a.get_text() for a in ax2.texts]
assert any(f"PM {pm:.0f}°" in t for t in ann2), f"PM 标注缺失: {ann2}"

# 4. 悬浮数据已备好
assert len(win._sweep_hover_data) == 1
ds = win._sweep_hover_data[0]
assert ds["name"] == "横滚" and len(ds["f"]) > 100

# 5. 悬浮事件模拟：在 ax1 幅值图上给出坐标读数
class _Evt:
    inaxes = ax1
    xdata = 50.0
    x, y = 300, 200

win._sweep_on_hover(_Evt())
assert win._sweep_hover_ann is not None, "悬浮标注未生成"
txt = win._sweep_hover_ann.get_text()
assert "Hz" in txt and "dB" in txt and "横滚" in txt, txt
# 相位图悬浮
_evt2 = type(_Evt)("Evt", (), {"inaxes": ax2, "xdata": 50.0,
                               "x": 300, "y": 400})()
win._sweep_on_hover(_evt2)
assert "相位" in win._sweep_hover_ann.get_text()
# 移出图表 → 标注清除
_evt3 = type(_Evt)("Evt", (), {"inaxes": None, "xdata": None,
                               "x": 0, "y": 0})()
win._sweep_on_hover(_evt3)
assert win._sweep_hover_ann is None, "悬浮标注未清除"

# 6. 截图保存
path = win._save_figure_png(win.sweep_figure, "sweep_test")
assert path is not None and os.path.exists(path) \
    and os.path.getsize(path) > 20000, "截图未生成"
print(f"  截图: {path}")

# 7. 轴切换：全部叠加 ←→ 单轴（结果里只有横滚，切到偏航应为空图但不崩）
win._sweep_result = result
win.sweep_axis_combo.setCurrentText("偏航")
app.processEvents()
assert len(win._sweep_hover_data) == 0, "单轴过滤失败"
win.sweep_axis_combo.setCurrentText("横滚")
app.processEvents()
assert len(win._sweep_hover_data) == 1

# 8. 黑匣子页截图按钮存在且可用同一保存函数
assert hasattr(win, "bb_shot_btn")

print("SWEEP_VIZ_OK")
