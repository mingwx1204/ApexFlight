# -*- coding: utf-8 -*-
"""ApexFlight 应用日志总线（v0.99 从 main.py 独立，供各页签模块共用）。

- 会话内事件记录，参考 BF Configurator 的日志页
- 同时写入 logs/app.log（超 1MB 自动轮转为 app.log.1）
- 线程安全：后台线程直接调用 log_event 即可，界面经信号接收
"""

import threading
from datetime import datetime

from PyQt6.QtCore import QObject, pyqtSignal

from apex_fc import LOGS_DIR


class _AppLogger(QObject):
    """日志总线：后台线程 emit 信号，界面线程安全接收"""
    appended = pyqtSignal(str)


app_logger = _AppLogger()
_app_log_lock = threading.Lock()
_app_log_lines: list = []                     # 本次会话的全部日志行


def log_event(message: str):
    """记录一条应用日志（线程安全）：内存缓冲 + 追加 app.log + 通知界面"""
    line = f"{datetime.now():%Y-%m-%d %H:%M:%S}  {message}"
    with _app_log_lock:
        _app_log_lines.append(line)
        try:
            LOGS_DIR.mkdir(exist_ok=True)
            log_file = LOGS_DIR / "app.log"
            if log_file.exists() and log_file.stat().st_size > 1024 * 1024:
                log_file.replace(LOGS_DIR / "app.log.1")   # 轮转，保留上一份
            with open(log_file, "a", encoding="utf-8") as f:
                f.write(line + "\n")
        except Exception:
            pass
    app_logger.appended.emit(line)
