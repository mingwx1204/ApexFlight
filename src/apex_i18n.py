# -*- coding: utf-8 -*-
"""ApexFlight - 全局版本、用户配置（config.json）与界面多语言（i18n）

翻译机制：
- 以中文原文为键的词典（_EN）。界面代码构建时仍写中文（源语言），
  切换语言时由主窗口的控件树遍历器把所有控件的当前文本做双向转换
  （中→英 或 英→中），做到不重启立即生效。
- 未收录的字符串原样显示，保证任何情况下界面不崩。
- 含变量插值的动态字符串（状态栏提示、数值读数）不在翻译范围内。
"""

import json
import re
import sys
from pathlib import Path

APP_VERSION = "1.01"
APP_NAME = "ApexFlight"

# 配置文件位置：开发时在项目根目录；打包后在 exe 同级目录
if getattr(sys, "frozen", False):
    CONFIG_PATH = Path(sys.executable).resolve().parent / "config.json"
else:
    CONFIG_PATH = Path(__file__).resolve().parent.parent / "config.json"

_current_lang = "zh"

# 中文原文 → English
_EN = {
    # ---- 侧栏 / 顶栏 / 通用按钮 ----
    "欢迎": "Welcome",
    "仪表盘": "Dashboard", "PID 调参": "PID Tuning", "Rates 调参": "Rates",
    "滤波器": "Filters", "电机测试": "Motors", "接收机": "Receiver",
    "黑匣子": "Blackbox", "调参方案": "Presets", "AI 助手": "AI Assistant",
    "扫频调参": "Sweep Tune",
    "日志": "Log",
    "串口": "Port", "刷新": "Refresh", "波特率": "Baud",
    "连接": "Connect", "断开": "Disconnect", "设置": "Settings",
    "保存": "Save", "清空": "Clear", "取消": "Cancel", "确定": "OK",
    "全选": "Select all", "另存为…": "Save as...",
    # ---- 状态栏（静态部分）----
    "就绪：请选择串口后点击「连接」": "Ready: select a port and click [Connect]",
    "就绪": "Ready",
    "正在扫描串口……": "Scanning ports...",
    "已连接": "Connected", "已断开连接": "Disconnected",
    "已断开连接，串口已释放": "Disconnected, port released",
    "检测中……": "Detecting...",
    "未知": "Unknown", "未连接": "Not connected", "未连接飞控": "Not connected",
    "（未检测到串口，请插入飞控）": "(No port found - plug in the FC)",
    "（服务未运行）": "(service not running)",
    "（请先下载模型）": "(download a model first)",
    # ---- 设置对话框 ----
    "语言": "Language", "默认波特率": "Default baud rate",
    "打开日志文件夹": "Open log folder", "关于": "About",
    "意见反馈": "Feedback", "复制诊断信息": "Copy diagnostics",
    "诊断信息已复制，到 GitHub Issues 粘贴即可": 
        "Diagnostics copied - paste them into the GitHub issue",
    "在 GitHub 上提交建议/问题": "Report issues or suggestions on GitHub",
    # ---- 欢迎页（v0.92）----
    "开源 FPV 无人机调参软件": "Open-source FPV drone tuning tool",
    "全本地运行 · 零云端 · 保护飞手隐私":
        "Fully local, zero cloud - your flight data never leaves this PC",
    "快速上手": "Quick start",
    "① 插入飞控 USB，关闭 Betaflight Configurator（串口独占）":
        "1. Plug in the FC via USB; close Betaflight Configurator (port is exclusive)",
    "② 顶栏选择串口，点击「连接」":
        "2. Pick the port in the top bar, then click [Connect]",
    "③ 仪表盘查看姿态与状态，到 PID / Rates 页开始调参":
        "3. Check attitude on Dashboard, then tune in PID / Rates",
    "QQ 交流群": "QQ Group",
    "扫码或搜群号加入，一起交流调参心得":
        "Scan the code or search the group number to chat tuning with us",
    "群号：": "Group No.:",
    "复制群号": "Copy No.",
    "联系作者": "Contact the Author",
    "有建议或合作意向，欢迎来邮": "Suggestions or cooperation? Drop an email",
    "复制邮箱": "Copy email",
    "开源社区": "Open Source",
    "代码开源（GPL-3.0），欢迎 Star / Issue / PR":
        "Open source (GPL-3.0) - stars, issues and PRs are welcome",
    "打开 GitHub 仓库": "Open GitHub repo",
    "已复制到剪贴板": "Copied to clipboard",
    # ---- 虚拟连接 / 适飞地图（v0.93）----
    "虚拟连接（无飞控体验全部功能）": "Virtual FC (try all features, no hardware)",
    "适飞地图": "Fly Zones",
    "底图：": "Base map:",
    "放大": "Zoom in", "缩小": "Zoom out",
    "定位我的城市": "Locate my city",
    "搜索地点，如：柳州": "Search a place, e.g. Liuzhou",
    "搜索": "Search",
    "同步 UOM 适飞区": "Sync UOM fly zones",
    "用黑匣子页当前日志": "Use current Blackbox log",
    "开始分析": "Analyze", "未选择日志": "No log selected",
    "三轴辨识指标": "Per-axis metrics",
    "扫频精准调参（实测数据 + 数学推导，不用 AI 猜）":
        "Sweep tuning (measured data + math, no AI guessing)",
    "精准调参建议（每条都带数学依据）":
        "Precise suggestions (each with math rationale)",
    "应用 PID 建议到飞控": "Apply PID suggestions to FC",
    "UOM 适飞区同步": "UOM fly-zone sync",
    "单击地图放置起飞点标记，坐标供 UOM 空域申请填表使用":
        "Click the map to drop a takeoff marker; coordinates ready for UOM airspace forms",
    "滤波器类建议请到「滤波器」页按建议频率手动调整；首次应用后请拆桨低空试飞验证":
        "Apply filter suggestions manually on the Filters page; verify with a careful first flight",
    "复制坐标": "Copy coordinates",
    "参数": "Parameter", "当前值": "Current",
    "建议值": "Suggested", "变化": "Change",
    "语言已切换：界面已立即生效（状态栏等动态提示保持中文）。":
        "Language switched. (Dynamic status messages remain in Chinese.)",
    # ---- v0.99：单位 / 机型卡片 / 日志标签 / 扫频视图 / 电机页 ----
    "基础单位": "Units",
    "公制（米 / 公里）": "Metric (m / km)",
    "英制（英尺 / 英里）": "Imperial (ft / mi)",
    "机型信息备注": "Craft Info",
    "名称": "Name", "机架": "Frame", "电机": "Motors", "桨叶": "Props",
    "电池": "Battery", "备注": "Notes",
    "保存机型信息": "Save craft info",
    "如：5寸花飞机": "e.g. 5-inch freestyle",
    "如：Mark5": "e.g. Mark5",
    "如：2207 1950KV": "e.g. 2207 1950KV",
    "如：51466 三叶": "e.g. 51466 tri-blade",
    "如：6S 1300mAh": "e.g. 6S 1300mAh",
    "自由记录": "Free text",
    "日志备注标签": "Log note tag",
    "保存标签": "Save tag",
    "全部叠加": "All axes",
    "横滚": "Roll", "俯仰": "Pitch", "偏航": "Yaw",
    "查看轴：": "Axis:",
    "保存截图": "Save screenshot",
    "把当前伯德图存为 PNG，方便发到交流群讨论":
        "Save the Bode plot as PNG to share in your group",
    "切换 全部叠加/单轴 视图（有结果时立即重绘）":
        "Switch all-axes / single-axis view (redraws on results)",
    "打开日志": "Open log",
    "我已拆下螺旋桨，启用电机测试":
        "Props removed - enable motor test",
    "主控": "Master",
    "模型下载分片": "Model download segments",
    "{n} 线程分片": "{n} threads",
    "下载 AI 模型时的并发分片数（越多越快，但太大会被服务器限速）":
        "Concurrent download segments for AI models (more = faster, but too many gets throttled)",
    # ---- 日志页 ----
    "应用日志": "Application log", "（暂无日志）": "(no log yet)",
    "日志已清空": "Log cleared", "日志文件 (*.txt)": "Log files (*.txt)",
    # ---- 仪表盘 ----
    "飞控信息": "FC Info", "固件版本：": "Firmware:",
    "飞控型号：": "Board:", "机型/电机：": "Type/Motors:",
    "电源 / 链路": "Power / Link", "电池电压：": "Voltage:",
    "电流：": "Current:", "已耗电：": "Consumed:", "信号强度：": "RSSI:",
    "飞控状态": "FC Status", "CPU 负载：": "CPU load:",
    "循环时间：": "Cycle time:", "解锁禁用：": "Arming disabled:",
    "飞行姿态（拿起飞机转一转试试）": "Attitude (pick up and rotate the quad)",
    "无（可以解锁）": "None (ready to arm)",
    "横滚 — ｜ 俯仰 — ｜ 航向 —": "Roll — | Pitch — | Yaw —",
    # ---- PID 页 ----
    "备份当前参数": "Backup current", "从备份恢复": "Restore backup",
    "直接双击表格中的数值进行修改，改完点「写入飞控」。":
        "Double-click a cell to edit, then click [Save] to write to the FC.",
    # ---- Rates 页 ----
    "油门": "Throttle", "油门限制百分比：": "Throttle limit %:",
    "油门中点：": "Throttle mid:", "油门 Expo：": "Throttle expo:",
    "基本 / 手动 Rate": "Basic / Manual Rate", "满杆 deg/s": "Max deg/s",
    "Rates 预览": "Rates preview",
    "手感": "Feel",
    "（固件为非经典类型，仅供参考）": "(non-classic rates type, read-only)",
    "油门曲线预览": "Throttle curve preview",
    "摇杆偏转 (%)": "Stick deflection (%)", "角速度 (°/s)": "Rate (°/s)",
    "油门输入 (%)": "Throttle in (%)", "油门输出 (%)": "Throttle out (%)",
    # ---- 滤波器页（字段名来自数据表）----
    "陀螺仪（独立于 PID 配置文件）": "Gyro (profile independent)",
    "D Term / 偏航（PID 配置文件关联）": "D Term / Yaw (profile linked)",
    "陀螺仪低通滤波器 1": "Gyro lowpass 1", "陀螺仪低通滤波器 2": "Gyro lowpass 2",
    "陀螺仪陷波滤波器 1": "Gyro notch 1", "陀螺仪陷波滤波器 2": "Gyro notch 2",
    "陀螺仪 RPM 滤波器": "Gyro RPM filter",
    "动态陷波滤波器": "Dynamic notch",
    "D Term 低通滤波器 1": "D Term lowpass 1",
    "D Term 低通滤波器 2": "D Term lowpass 2",
    "D Term 陷波滤波器": "D Term notch",
    "偏航低通滤波器": "Yaw lowpass",
    "启用：": "Enabled:", "滤波器类型": "Filter type",
    "最低截止频率 (Hz)": "Min cutoff (Hz)",
    "最高截止频率 (Hz)": "Max cutoff (Hz)",
    "静态截止频率 (Hz)": "Static cutoff (Hz)",
    "最低频率 (Hz)": "Min freq (Hz)",
    "最高频率 (Hz)": "Max freq (Hz)",
    "频率 (Hz)": "Frequency (Hz)", "截止 (Hz)": "Cutoff (Hz)",
    "谐波数量": "Harmonics", "陷波数量": "Notch count",
    "Q 因子": "Q factor", "动态曲线 Expo": "Dyn curve expo",
    "建议先在「黑匣子」页做频谱分析找到噪声峰后再调整。":
        "Tip: run FFT on the Blackbox page first to find noise peaks.",
    "黑匣子噪声峰：尚未做频谱分析": "Noise peaks: no FFT yet",
    "黑匣子噪声峰：未找到明显噪声峰": "Noise peaks: none found",
    # ---- 电机测试页 ----
    "电机输出（未连接）": "Motor outputs (not connected)",
    "全部停止": "Stop all",
    "我已拆下所有螺旋桨": "All propellers are removed",
    "我了解风险，确认开始测试": "I understand the risk, enable testing",
    "危险：电机测试会让电机真实转动！":
        "DANGER: motor test spins real motors!",
    "使用前必须【拆下所有螺旋桨】，并确认飞机固定牢固、":
        "Remove ALL propellers first; secure the quad and keep",
    "周围没有人员和杂物。": "people and objects away.",
    "只有两个安全确认都勾选且已连接，滑块才能用":
        "Sliders unlock only when both boxes are checked and connected",
    "电机位置示意（编号与 BF 一致）": "Motor layout (numbering matches BF)",
    "转动中的电机随油门高亮": "Spinning motors highlight with throttle",
    "主控制": "Master", "停转电机": "Stop motors",
    # ---- 接收机页 ----
    "通道（未连接）": "Channels (not connected)",
    "实时显示接收机各通道数值（正常范围约 1000~2000，":
        "Live receiver channel values (normal range ~1000-2000,",
    "中位约 1500）。打开发射机并拨动摇杆，数值会跟着动。":
        "mid ~1500). Turn on your radio and move the sticks.",
    # ---- 黑匣子页 ----
    "打开日志文件": "Open log", "生成演示日志": "Demo log",
    "从飞控下载": "Download from FC", "取消下载": "Cancel",
    "日志信息": "Log info",
    "通道用途": "Channel help",
    "时间范围": "Time range", "起点：": "Start:", "终点：": "End:",
    "绘制曲线": "Plot", "频谱分析": "FFT",
    "AI 解读图表": "AI analysis",
    "双日志对比": "Dual-log compare",
    "加载对比日志": "Load compare log",
    "对比模式（当前 vs 对比日志）": "Compare mode (current vs compare)",
    "未加载对比日志": "No compare log loaded",
    "未加载日志（支持 .bbl / .bfl / .csv）": "No log (.bbl / .bfl / .csv)",
    "日志段：": "Session:", "切换日志段": "Switch session",
    "下完清空闪存": "Erase after download", "下载范围：": "Range:",
    "全部（慢）": "All (slow)",
    "最近 1 MB（约 25 秒）": "Last 1 MB (~25 s)",
    "最近 2 MB（约 45 秒）": "Last 2 MB (~45 s)",
    "最近 4 MB（约 1.5 分钟）": "Last 4 MB (~1.5 min)",
    "最近 8 MB（约 3 分钟）": "Last 8 MB (~3 min)",
    "归一化显示（比较形状）": "Normalize (compare shapes)",
    "黑匣子数据轨道": "Blackbox tracks",
    "建议用「清空」后只开需要的通道": "Tip: clear all, then enable only what you need",
    "移动鼠标到图上查看数值": "Hover over the plot to read values",
    "时间 (秒)": "Time (s)", "频率 (Hz)": "Frequency (Hz)", "幅度": "Amplitude",
    "频谱分析（噪声峰位置决定滤波器截止频率）":
        "FFT (noise peaks guide filter cutoffs)",
    "黑匣子日志 (*.bbl *.bfl *.csv);;所有文件 (*)":
        "Blackbox logs (*.bbl *.bfl *.csv);;All files (*)",
    "选择黑匣子日志": "Select blackbox log",
    "选择对比日志（如调参前的飞行）": "Select compare log (e.g. before tuning)",
    "当前日志": "Current", "空转": "Bench run", "静止": "Static",
    # ---- 调参方案页 ----
    "保存当前为预设": "Save current as preset",
    "应用选中预设": "Apply selected",
    "删除选中": "Delete selected",
    "刷新列表": "Refresh list",
    "预设名称：": "Preset name:",
    "例如：花飞手感 / 竞速稳拍": "e.g. freestyle feel / race stable",
    "预设 = 一整套调参状态（PID + Rates + 滤波器）。":
        "A preset = full tuning state (PID + Rates + Filters).",
    "把当前飞控状态保存为预设，之后可一键切换；":
        "Save current FC state as a preset for one-click switch;",
    "应用前会自动备份当前配置，随时可以调回来。":
        "Current config is auto-backed up before applying.",
    "选择备份文件": "Select backup file",
    "ApexFlight 备份 (*.json)": "ApexFlight backup (*.json)",
    # ---- AI 页 ----
    "本地 AI 服务（Ollama）": "Local AI service (Ollama)",
    "刷新状态": "Refresh", "安装指引": "Install guide",
    "模型：": "Model:",
    "综合调参建议": "Full tuning advice",
    "分析当前 PID": "Analyze current PID",
    "分析黑匣子统计": "Analyze blackbox stats",
    "清空对话": "Clear chat",
    "发送": "Send", "停止": "Stop",
    "AI 助手页：连接本机 Ollama 大模型，做调参问答与数据分析":
        "AI assistant: chat with a local Ollama model about tuning",
    "输入你的调参问题，例如：翻滚时感觉有点软，应该怎么调？":
        "Ask a tuning question, e.g.: rolls feel soft, what should I tune?",
    "提示：AI 在本地电脑上运行（Ollama），不会上传任何飞行数据。":
        "Note: AI runs locally (Ollama); no flight data is uploaded.",
    "综合调参建议：全部真实数据一次性发给 AI 出方案":
        "Full advice: send all real data to AI for a plan",
    "快捷按钮：把当前 PID 表格内容发给 AI": "Shortcut: send PID table to AI",
    "快捷按钮：把黑匣子统计结果发给 AI": "Shortcut: send blackbox stats to AI",
    "安装本地 AI（Ollama）": "Install local AI (Ollama)",
    "Ollama 运行中": "Ollama running",
    "未检测到 Ollama 服务": "Ollama not detected",
    "Ollama 运行中，但还没有安装模型": "Ollama running, no model installed",
    "性能匹配": "Match for my PC",
    "点「性能匹配」自动检测本机配置并推荐模型":
        "Click [Match] to detect your PC and pick the right model",
    "一键拉取推荐模型": "Pull recommended model",
    "一键拉取": "Pull",
    "重试拉取": "Retry pull",
    # ---- 常用确认对话框 ----
    "确认写入": "Confirm write", "确认恢复": "Confirm restore",
    "确认删除": "Confirm delete", "确认应用预设": "Confirm preset",
    "确定继续吗？": "Continue?",
    "写入前会自动备份当前参数到 backups/ 文件夹。":
        "Current params are auto-backed up to backups/ before writing.",
    "正在连接": "Connecting",
}

# 反向词典（英 → 中），用于从英文切回中文
_EN_REV = {v: k for k, v in _EN.items()}

# 控件文本前缀里常见的 emoji/符号（翻译时剥掉前缀，翻译后拼回）
_PREFIX_RE = re.compile(
    r"^[\s💾✅🗑️🔄📂🧪📥ℹ️❓🎨📶🤖🧠📈⏹🛑📦⚙️⚠️💬✍️🎛️🎯🌊📡📜📊️ ]*")


def tr(text: str) -> str:
    """按当前语言翻译界面文本；未收录的字符串原样返回"""
    if _current_lang == "en":
        return _EN.get(text, text)
    return text


def localize_text(text: str) -> str:
    """把控件上的文本转换成当前语言（双向）。

    流程：剥掉 emoji 前缀 → 若当前是英文文本则先反查回中文原文
    → 再按当前语言正向翻译 → 拼回前缀。
    查不到的一律原样返回。
    """
    if not text or "<" in text:
        return text
    m = _PREFIX_RE.match(text)
    prefix, core = m.group(0), text[m.end():]
    if not core:
        return text
    # 表单标签常带冒号后缀（"名称："），剥掉再查、翻译后拼回。
    # 词典里既有带冒号的键（"油门中点："）也有不带的（滤波器字段名），
    # 两种都试；若命中的是带冒号的译文则不再补冒号。
    had_colon = core.endswith(("：", ":"))
    lookup = core[:-1] if had_colon else core
    colon_out = ""
    if had_colon:
        colon_out = ":" if _current_lang == "en" else "："
    if _current_lang == "en":
        new_core = _EN.get(lookup)
        if new_core is None:
            new_core = _EN.get(core)
            colon_out = ""                 # 命中带冒号键，译文自带冒号
        if not new_core:
            return text
    else:
        new_core = _EN_REV.get(lookup)
        if new_core is None:
            new_core = _EN_REV.get(core)
            colon_out = ""
        if not new_core:
            return text
    return prefix + new_core + colon_out


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
            # 标量 + dict（机型信息备注等结构化配置）都保留
            defaults.update({k: v for k, v in cfg.items()
                             if isinstance(v, (str, int, float, bool, dict))})
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
