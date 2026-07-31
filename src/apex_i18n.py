# -*- coding: utf-8 -*-
"""ApexFlight - 全局版本、用户配置（config.json）与界面多语言（i18n）

语言方案：以中文原文为键的词典翻译（_EN）。界面代码用 tr("中文") 包裹，
当前语言为英文时返回译文，否则返回原文。未收录的字符串原样显示，
保证任何情况下界面不崩。
"""

import json
from pathlib import Path

APP_VERSION = "0.9"
APP_NAME = "ApexFlight"

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config.json"

_current_lang = "zh"

# 中文原文 → English（优先覆盖：侧栏 / 顶栏 / 设置 / 日志页 / 常用状态）
_EN = {
    # 侧栏
    "仪表盘": "Dashboard",
    "PID 调参": "PID Tuning",
    "Rates 调参": "Rates",
    "滤波器": "Filters",
    "电机测试": "Motors",
    "接收机": "Receiver",
    "黑匣子": "Blackbox",
    "调参方案": "Presets",
    "AI 助手": "AI Assistant",
    "日志": "Log",
    # 顶栏
    "串口": "Port",
    "刷新": "Refresh",
    "波特率": "Baud",
    "连接": "Connect",
    "断开": "Disconnect",
    "设置": "Settings",
    # 状态栏
    "就绪：请选择串口后点击「连接」":
        "Ready: select a port and click [Connect]",
    "正在扫描串口……": "Scanning ports...",
    "已连接": "Connected",
    "已断开连接，串口已释放": "Disconnected, port released",
    # 设置对话框
    "语言": "Language",
    "默认波特率": "Default baud rate",
    "打开日志文件夹": "Open log folder",
    "关于": "About",
    "确定": "OK",
    "取消": "Cancel",
    "应用": "Apply",
    "语言已切换：导航与顶栏立即生效，其余界面重启后完全生效。":
        "Language switched: navigation applies now; "
        "other pages take full effect after restart.",
    # 日志页
    "清空": "Clear",
    "另存为…": "Save as...",
    "应用日志": "Application log",
    "（暂无日志）": "(no log yet)",
}


def tr(text: str) -> str:
    """按当前语言翻译界面文本；未收录的字符串原样返回"""
    if _current_lang == "en":
        return _EN.get(text, text)
    return text


def get_language() -> str:
    return _current_lang


def set_language(lang: str):
    global _current_lang
    _current_lang = "en" if str(lang).lower().startswith("en") else "zh"


def load_config() -> dict:
    """读取 config.json；文件不存在或损坏时返回默认配置"""
    defaults = {"language": "zh", "baud": "115200"}
    try:
        cfg = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
        if isinstance(cfg, dict):
            defaults.update({k: v for k, v in cfg.items()
                             if isinstance(v, (str, int, float, bool))})
    except Exception:
        pass
    return defaults


def save_config(cfg: dict):
    try:
        CONFIG_PATH.write_text(
            json.dumps(cfg, ensure_ascii=False, indent=2), encoding="utf-8")
    except Exception:
        pass


def init_from_config(cfg: dict):
    """启动时按配置初始化语言"""
    set_language(cfg.get("language", "zh"))
