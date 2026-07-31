# -*- coding: utf-8 -*-
"""v0.91 回归：启动即写 app.log；切页不重复连接日志总线。"""
import os, sys
os.environ["QT_QPA_PLATFORM"] = "offscreen"
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from PyQt6.QtWidgets import QApplication
import main as m

# 清空日志，只统计本次运行
log_file = m.LOGS_DIR / "app.log"
log_file.write_text("", encoding="utf-8")

app = QApplication(sys.argv)
win = m.MainWindow()

first = log_file.read_text(encoding="utf-8")
assert "已启动" in first, f"启动日志缺失: {first[-200:]}"
print("PASS 1 启动即写 app.log")

from main import app_logger
n0 = app_logger.receivers(app_logger.appended)
for i in range(5):
    win.sidebar.setCurrentRow(i % 10)
app.processEvents()
n1 = app_logger.receivers(app_logger.appended)
assert n0 == n1, f"日志总线连接数增长 {n0} -> {n1}"
txt = log_file.read_text(encoding="utf-8")
cnt = txt.count("已启动")
assert cnt == 1, f"已启动 出现 {cnt} 次"
print(f"PASS 2 切页无重复连接(receivers={n1})、启动日志仅 1 条")
print("STARTUP_LOG_OK")
