# -*- coding: utf-8 -*-
"""v0.99 重构：把 main.py 的四大页签方法块机械抽取为 mixin 模块。

抽取规则：定位 `    def NAME(`，向上收纳连续的 4 空格注释行，
向下吃到下一个 4 空格顶格行（def / 注释）或文件类边界。
生成的 tab_*.py 头部按各块实际依赖硬编码；main.py 同步改继承。
运行：python tools/split_mixins.py   （在仓库根目录）
"""

import re
from pathlib import Path

SRC = Path(__file__).resolve().parent.parent / "src"
MAIN = SRC / "main.py"

# ---- 各 mixin 文件配置 ----
PLANS = {
    "tab_map": {
        "doc": '"""适飞地图页（v0.99 从 main.py 拆出）：底图/标记/定位/搜索/UOM"""',
        "imports": (
            "from PyQt6.QtCore import QTimer\n"
            "from PyQt6.QtWidgets import (\n"
            "    QComboBox, QHBoxLayout, QLabel, QLineEdit, QMessageBox,\n"
            "    QPushButton, QVBoxLayout, QWidget)\n"
            "\n"
            "from apex_i18n import tr\n"
            "from apex_log import log_event\n"
        ),
        "cls": "MapTabMixin",
        "cls_doc": '    """适飞地图页全部 UI 与处理器（self 即 MainWindow）"""\n',
        "methods": [
            "_build_map_tab", "_on_map_marker", "_map_copy_coords",
            "_map_locate", "_on_geo_position", "_on_geo_error",
            "_geo_give_up", "_map_locate_ip_fallback", "_map_search",
            "_map_sync_uom",
        ],
    },
    "tab_sweep": {
        "doc": '"""扫频调参页（v0.99 从 main.py 拆出）：系统辨识 + 伯德图 + 建议"""',
        "imports": (
            "from pathlib import Path\n"
            "\n"
            "from PyQt6.QtGui import QColor\n"
            "from PyQt6.QtWidgets import (\n"
            "    QComboBox, QFileDialog, QGroupBox, QHBoxLayout, QHeaderView,\n"
            "    QLabel, QMessageBox, QPushButton, QTableWidget,\n"
            "    QTableWidgetItem, QVBoxLayout, QWidget)\n"
            "\n"
            "from apex_fc import LOGS_DIR\n"
            "from apex_i18n import tr\n"
            "from apex_log import log_event\n"
            "\n"
            "\n"
            "def _mpl():\n"
            '    """延迟取 main 模块的 matplotlib 加载器与类（避免循环导入）"""\n'
            "    import main as _m\n"
            "    _m.load_matplotlib()\n"
            "    return (_m.HAS_MPL, _m.Figure, _m.FigureCanvasQTAgg,\n"
            "            _m.NavigationToolbar2QT)\n"
        ),
        "cls": "SweepTabMixin",
        "cls_doc": '    """扫频调参页全部 UI 与处理器（self 即 MainWindow）"""\n',
        "methods": [
            "_build_sweep_tab", "_sweep_open_log", "_sweep_use_bb",
            "_sweep_analyze", "_sweep_show_result", "_sweep_axis_changed",
            "_sweep_on_hover", "_sweep_apply",
        ],
    },
    "tab_motor": {
        "doc": '"""电机测试页（v0.99 从 main.py 拆出）：对齐 BF 最新 MotorsTab"""',
        "imports": (
            "from PyQt6.QtCore import Qt, QTimer\n"
            "from PyQt6.QtWidgets import (\n"
            "    QCheckBox, QFrame, QGroupBox, QHBoxLayout, QLabel,\n"
            "    QProgressBar, QPushButton, QSlider, QVBoxLayout, QWidget)\n"
            "\n"
            "from apex_fc import set_motors\n"
            "from apex_i18n import tr\n"
            "from apex_log import log_event\n"
            "\n"
            "# DShot 停转 / 满油门（µs），对齐 BF MotorsTab\n"
            "MOTOR_MIN_US = 1000\n"
            "MOTOR_MAX_US = 2000\n"
        ),
        "cls": "MotorTabMixin",
        "cls_doc": '    """电机测试页全部 UI 与处理器（self 即 MainWindow）"""\n',
        "methods": [
            "_build_motor_tab", "on_motor_count", "_update_motor_lock",
            "_on_motor_slider", "_on_master_slider",
            "_current_motor_command", "_send_motor_values",
            "_send_motor_stop", "on_motor_stop", "on_motor_values",
            "_stop_motors_safely",
        ],
    },
    "tab_log": {
        "doc": '"""应用日志页（v0.99 从 main.py 拆出）：事件查看/清空/另存/打开目录"""',
        "imports": (
            "from datetime import datetime\n"
            "from pathlib import Path\n"
            "\n"
            "from PyQt6.QtGui import QFont\n"
            "from PyQt6.QtWidgets import (\n"
            "    QFileDialog, QHBoxLayout, QMessageBox, QPushButton,\n"
            "    QTextEdit, QVBoxLayout, QWidget)\n"
            "\n"
            "import apex_i18n as i18n\n"
            "from apex_fc import LOGS_DIR\n"
            "from apex_i18n import tr\n"
            "from apex_log import _app_log_lines, _app_log_lock, log_event\n"
        ),
        "cls": "LogTabMixin",
        "cls_doc": '    """应用日志页全部 UI 与处理器（self 即 MainWindow）"""\n',
        "methods": [
            "_build_log_tab", "_on_app_log", "_on_log_clear",
            "_on_log_save", "_open_log_folder",
        ],
    },
}

lines = MAIN.read_text(encoding="utf-8").splitlines(keepends=True)


def find_span(name):
    """返回 (start, end) 行号区间 [start, end)，含上方连续注释块"""
    pat = f"    def {name}("
    start = next(i for i, ln in enumerate(lines) if ln.startswith(pat))
    # 向上收纳连续注释行
    c = start
    while c - 1 >= 0 and lines[c - 1].startswith("    #"):
        c -= 1
    start = c
    # 向下吃到下一个 4 空格顶格的非空行
    end = start + 1
    while end < len(lines):
        ln = lines[end]
        if ln.strip() == "":
            end += 1
            continue
        if ln.startswith("    ") and not ln.startswith("     "):
            break                      # 同级 def / 注释
        if not ln.startswith(" "):
            break                      # 类外（def main() 等）
        end += 1
    # 去掉尾部纯空行，保留一个
    while end - 1 > start and lines[end - 1].strip() == "" \
            and lines[end - 2].strip() == "":
        end -= 1
    return start, end


# ---- 抽取所有方法（先按文档顺序收集） ----
extracts = {k: [] for k in PLANS}
spans = []
for key, plan in PLANS.items():
    for name in plan["methods"]:
        s, e = find_span(name)
        spans.append((s, e))
        extracts[key].append("".join(lines[s:e]))

# ---- 生成 mixin 文件 ----
for key, plan in PLANS.items():
    header = (
        "# -*- coding: utf-8 -*-\n"
        f"{plan['doc']}\n"
        "\n"
        f"{plan['imports']}"
        "\n"
        f"class {plan['cls']}:\n"
        f"{plan['cls_doc']}"
    )
    body = "\n".join(extracts[key])
    # 扫频块：matplotlib 走延迟访问器
    if key == "tab_sweep":
        body = body.replace(
            "        if load_matplotlib():",
            "        HAS_MPL, Figure, FigureCanvasQTAgg, \\\n"
            "            NavigationToolbar2QT = _mpl()\n"
            "        if HAS_MPL:")
    out = SRC / f"{key}.py"
    out.write_text(header + body, encoding="utf-8")
    print(f"  生成 {out.name}（{len(extracts[key])} 个方法）")

# ---- main.py：逆序删除已抽取片段 ----
for s, e in sorted(spans, reverse=True):
    del lines[s:e]
text = "".join(lines)

# 删除电机常量（已移入 tab_motor）
text = text.replace(
    "\n# 电机测试常量（对齐 BF MotorsTab：DShot 停转=1000，满油门=2000 µs）\n"
    "MOTOR_MIN_US = 1000\n"
    "MOTOR_MAX_US = 2000\n",
    "")

# 插入 mixin 导入 + 改继承
text = text.replace(
    "from apex_log import *        # noqa: F401,F403  app_logger / log_event 等\n",
    "from apex_log import *        # noqa: F401,F403  app_logger / log_event 等\n"
    "from tab_map import MapTabMixin          # 适飞地图页\n"
    "from tab_sweep import SweepTabMixin      # 扫频调参页\n"
    "from tab_motor import MotorTabMixin      # 电机测试页\n"
    "from tab_log import LogTabMixin          # 应用日志页\n")
text = text.replace(
    "class MainWindow(QMainWindow):",
    "class MainWindow(MapTabMixin, SweepTabMixin, MotorTabMixin,\n"
    "                LogTabMixin, QMainWindow):")

MAIN.write_text(text, encoding="utf-8")
print("  main.py 已重写（移除抽取片段，改 mixin 继承）")
