# -*- coding: utf-8 -*-
"""v0.94 回归：AI 全自动调参对比对话框必须能构建
（v0.93 曾在此崩 NameError: QDialog is not defined）"""
import os
import sys

os.environ["QT_QPA_PLATFORM"] = "offscreen"
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from PyQt6.QtWidgets import QApplication, QDialog, QTableWidget
import main as m

app = QApplication(sys.argv)
win = m.MainWindow()

changes = [
    {"kind": "pid", "key": (0, 0), "label": "Roll（横滚） P", "old": 45, "new": 52},
    {"kind": "pid", "key": (0, 2), "label": "Roll（横滚） D", "old": 40, "new": 30},
    {"kind": "rate", "key": ("rate", 0), "label": "Rate 横滚", "old": 0.67, "new": 0.72},
    {"kind": "filter", "key": "gyro_dyn_min", "label": "陀螺仪动态低通·下限 (Hz)",
     "old": 250, "new": 200},
]
notes = ["AI 建议关闭某滤波，已拒绝（关闭滤波有烧电机风险）"]

# 补丁：exec 不阻塞，直接返回；捕获构建过程异常即视为失败
built = {}


class _SpyDialog(QDialog):
    def exec(self):
        built["table"] = self.findChild(QTableWidget)
        return QDialog.DialogCode.Rejected


m.QDialog = _SpyDialog            # main 模块内 QDialog 换成侦察版
win._show_autotune_dialog(changes, "测试思路说明", notes)
m.QDialog = QDialog               # 还原

table = built.get("table")
assert table is not None, "对话框未构建"
assert table.rowCount() == 4 and table.columnCount() == 4
assert table.item(0, 3).text().startswith("↑"), table.item(0, 3).text()
assert table.item(1, 3).text().startswith("↓"), table.item(1, 3).text()
print("AUTOTUNE_DIALOG_OK")
