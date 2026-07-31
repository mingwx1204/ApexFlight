# -*- coding: utf-8 -*-
"""v0.99 extras 回归：地图比例尺单位切换 + 机型信息备注 + 日志备注标签。

offscreen 运行：QT_QPA_PLATFORM=offscreen python tests/check_extras_v099.py
断言通过打印 EXTRAS_OK。
"""

import json
import os
import sys
from pathlib import Path

os.environ["QT_QPA_PLATFORM"] = "offscreen"
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from PyQt6.QtWidgets import QApplication          # noqa: E402

app = QApplication([])

import main                                       # noqa: E402
import apex_map as am                             # noqa: E402

win = main.MainWindow()
win.show()
app.processEvents()
win._ensure_page(6)          # 黑匣子页（懒加载）
win._ensure_page(10)         # 地图页（懒加载）

# ---- 1. 地图比例尺：公制 ----
w = am.SlippyMapWidget()
w.resize(800, 600)
w.center_lat, w.zoom = 24.3, 12
w.set_units("metric")
blen, btext = w._scale_bar()
assert 40 < blen < 300, f"比例尺长度异常 {blen}"
assert btext.endswith(("m", "km")), btext
print(f"  公制比例尺: {blen:.0f}px = {btext}")

# ---- 2. 英制切换 ----
w.set_units("imperial")
blen2, btext2 = w._scale_bar()
assert btext2.endswith(("ft", "mi")), btext2
print(f"  英制比例尺: {blen2:.0f}px = {btext2}")
w.set_units("火星制")                             # 非法值回退公制
assert w.units == "metric"

# ---- 3. 主窗口地图控件单位跟随配置 ----
assert hasattr(win.map_widget, "set_units")
win.map_widget.set_units(win._cfg.get("units", "metric"))

# ---- 4. 机型信息备注：字段齐全 + 保存到 config ----
assert set(win._craft_edits.keys()) == \
    {"名称", "机架", "电机", "桨叶", "电池", "备注"}
win._craft_edits["名称"].setText("5寸花飞机")
win._craft_edits["电机"].setText("2207 1950KV")
win._save_craft_info()
import apex_i18n as i18n
cfg = i18n.load_config()
assert cfg["craft"]["名称"] == "5寸花飞机", cfg.get("craft")
assert cfg["craft"]["电机"] == "2207 1950KV"
print("  机型信息已写入 config.json")

# ---- 5. 日志备注标签 ----
win.bb_sessions = [Path("logs/demo_flight.csv"), Path("logs/demo_hover.csv")]
win._setup_session_combo()
win.bb_tag_edit.setText("柳州试飞")
win.bb_session_combo.setCurrentIndex(0)
win._on_bb_tag_save()
assert win._log_tags["demo_flight.csv"] == "柳州试飞"
assert "🏷柳州试飞" in win.bb_session_combo.itemText(0), \
    win.bb_session_combo.itemText(0)
# 标签文件落盘
tags_file = Path(os.path.join(os.path.dirname(__file__), "..",
                              "logs", "log_tags.json"))
assert tags_file.exists()
data = json.loads(tags_file.read_text(encoding="utf-8"))
assert data["demo_flight.csv"] == "柳州试飞"
# 切到第二段：编辑框为空；清除第一段标签
win.bb_session_combo.setCurrentIndex(1)
win._sync_tag_edit()
assert win.bb_tag_edit.text() == ""
win.bb_session_combo.setCurrentIndex(0)
win._sync_tag_edit()
assert win.bb_tag_edit.text() == "柳州试飞"
win.bb_tag_edit.setText("")
win._on_bb_tag_save()
assert "demo_flight.csv" not in win._log_tags
tags_file.unlink(missing_ok=True)                # 清理测试痕迹
print("  日志标签读写/切换/清除全部正常")

print("EXTRAS_OK")
