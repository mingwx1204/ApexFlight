# -*- coding: utf-8 -*-
"""应用日志页（v0.99 从 main.py 拆出）：事件查看/清空/另存/打开目录"""

from datetime import datetime
from pathlib import Path

from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QFileDialog, QHBoxLayout, QMessageBox, QPushButton,
    QTextEdit, QVBoxLayout, QWidget)

import apex_i18n as i18n
from apex_fc import LOGS_DIR
from apex_i18n import tr
from apex_log import _app_log_lines, _app_log_lock, log_event

class LogTabMixin:
    """应用日志页全部 UI 与处理器（self 即 MainWindow）"""
    def _build_log_tab(self) -> QWidget:
        """应用日志页：连接/写入/下载/错误等事件实时滚动显示，
        随时可回看，可清空、另存、打开日志文件夹"""
        tab = QWidget()
        layout = QVBoxLayout(tab)

        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setFont(QFont("Consolas", 9))
        with _app_log_lock:
            history = list(_app_log_lines)
        self.log_view.setPlainText(
            "\n".join(history) if history else tr("（暂无日志）"))
        layout.addWidget(self.log_view, 1)

        btn_row = QHBoxLayout()
        self.log_clear_btn = QPushButton(tr("清空"))
        self.log_clear_btn.clicked.connect(self._on_log_clear)
        btn_row.addWidget(self.log_clear_btn)
        self.log_save_btn = QPushButton(tr("另存为…"))
        self.log_save_btn.clicked.connect(self._on_log_save)
        btn_row.addWidget(self.log_save_btn)
        self.log_open_btn = QPushButton(tr("打开日志文件夹"))
        self.log_open_btn.clicked.connect(self._open_log_folder)
        btn_row.addWidget(self.log_open_btn)
        btn_row.addStretch()
        layout.addLayout(btn_row)
        return tab


    def _on_app_log(self, line: str):
        """日志总线信号：追加到日志页（启动提示占位行先清掉）"""
        if self.log_view.toPlainText() == tr("（暂无日志）"):
            self.log_view.clear()
        self.log_view.append(line)


    def _on_log_clear(self):
        with _app_log_lock:
            _app_log_lines.clear()
        self.log_view.clear()
        log_event("日志已清空")


    def _on_log_save(self):
        path, _ = QFileDialog.getSaveFileName(
            self, tr("另存为…"), f"apexflight_log_"
            f"{datetime.now():%Y%m%d_%H%M%S}.txt",
            "日志文件 (*.txt)")
        if not path:
            return
        try:
            with _app_log_lock:
                content = "\n".join(_app_log_lines)
            Path(path).write_text(content, encoding="utf-8")
            self.statusBar().showMessage(f"日志已保存：{path}")
        except Exception as e:
            QMessageBox.warning(self, i18n.APP_NAME, f"保存失败：{e}")


    def _open_log_folder(self):
        """用系统文件管理器打开 logs/ 目录（app.log / crash.log / 黑匣子）"""
        LOGS_DIR.mkdir(exist_ok=True)
        import os
        os.startfile(str(LOGS_DIR))

