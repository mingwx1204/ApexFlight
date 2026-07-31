# -*- coding: utf-8 -*-
"""ApexFlight - AI 助手：Ollama 本地大模型通信（后台线程 + 流式信号）"""

import json
import threading
import urllib.error
import urllib.request

from PyQt6.QtCore import QObject, pyqtSignal

# ============================================================
# 第五部分（B）：AI 助手 —— Ollama 本地大模型通信
# ============================================================
# 使用 Ollama 的 HTTP API（http://localhost:11434）与本地大模型对话。
# 只用 Python 标准库 urllib，不需要额外安装依赖。
# 所有网络请求都在后台线程执行（由 MainWindow._run_in_thread 驱动），
# 通过 pyqtSignal 把流式生成的文字安全地送回界面线程。

OLLAMA_BASE_URL = "http://localhost:11434"
AI_RECOMMENDED_MODELS = ["qwen2.5:3b", "qwen2.5:1.5b"]   # v0.6 主推 3b 深度分析

AI_SYSTEM_PROMPT = (
    "你是 ApexFlight 内置的无人机调参专家助手，精通 Betaflight 固件、"
    "PID 调参、Rates/滤波设置和黑匣子日志分析。"
    "请始终使用简体中文回答，语言简洁、给出可操作建议。"
    "分析数据时请按这个结构回答：①总体状态判断 ②按优先级排列的具体修改建议"
    "（改哪项、建议改成多少、依据是什么）③需要提醒的风险。"
    "涉及电机测试、参数修改等操作时，务必先提醒用户卸下螺旋桨、注意人身安全。"
)


def ollama_status() -> tuple[bool, list]:
    """检测 Ollama 服务是否运行，返回 (是否运行, 已安装模型名列表)"""
    try:
        req = urllib.request.Request(OLLAMA_BASE_URL + "/api/tags")
        with urllib.request.urlopen(req, timeout=1) as resp:
            data = json.loads(resp.read().decode("utf-8"))
        models = [m.get("name", "") for m in data.get("models", [])]
        return True, models
    except Exception:
        return False, []


def chat_blocking(model: str, messages: list, timeout: int = 180,
                  json_mode: bool = False) -> str:
    """非流式对话（必须在后台线程调用），返回完整回答文本。

    json_mode=True 时使用 Ollama 的 format=json 强制合法 JSON 输出，
    用于 AI 全自动调参这类需要结构化结果的场景（v0.93）。
    """
    body_dict = {"model": model, "messages": messages, "stream": False}
    if json_mode:
        body_dict["format"] = "json"
    body = json.dumps(body_dict).encode("utf-8")
    req = urllib.request.Request(
        OLLAMA_BASE_URL + "/api/chat", data=body,
        headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        data = json.loads(resp.read().decode("utf-8"))
    return data.get("message", {}).get("content", "")


def extract_json(text: str) -> dict:
    """从 AI 回答里稳健地提取第一个 JSON 对象（允许前后带解释文字）"""
    start = text.find("{")
    end = text.rfind("}")
    if start < 0 or end <= start:
        raise ValueError("回答中没有 JSON 对象")
    return json.loads(text[start:end + 1])


# ------------------------------------------------------------
# v0.96：按电脑性能匹配最优本地调参模型
# ------------------------------------------------------------
def detect_hardware() -> dict:
    """检测本机硬件（全标准库 + 可选 PowerShell / nvidia-smi，任何一步
    失败都降级处理，绝不抛异常）：
    {"cpu": 线程数, "ram_gb": 内存GB, "gpu": 显卡名, "vram_gb": 显存GB}"""
    import os
    hw = {"cpu": os.cpu_count() or 0, "ram_gb": 0.0,
          "gpu": "", "vram_gb": 0.0}
    # 物理内存：Windows GlobalMemoryStatusEx
    try:
        import ctypes

        class _MEMSTAT(ctypes.Structure):
            _fields_ = [("dwLength", ctypes.c_ulong),
                        ("dwMemoryLoad", ctypes.c_ulong),
                        ("ullTotalPhys", ctypes.c_ulonglong),
                        ("ullAvailPhys", ctypes.c_ulonglong),
                        ("ullTotalPageFile", ctypes.c_ulonglong),
                        ("ullAvailPageFile", ctypes.c_ulonglong),
                        ("ullTotalVirtual", ctypes.c_ulonglong),
                        ("ullAvailVirtual", ctypes.c_ulonglong),
                        ("ullAvailExtendedVirtual", ctypes.c_ulonglong)]

        st = _MEMSTAT()
        st.dwLength = ctypes.sizeof(_MEMSTAT)
        if ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(st)):
            hw["ram_gb"] = round(st.ullTotalPhys / (1024 ** 3), 1)
    except Exception:
        pass
    # 显卡名：PowerShell WMI（独显优先），启动隐藏窗口。
    # 有些环境 PATH 不含 System32，先 which 再找系统目录绝对路径
    ps = None
    try:
        import shutil
        ps = shutil.which("powershell")
        if not ps:
            import ctypes
            buf = ctypes.create_unicode_buffer(260)
            ctypes.windll.kernel32.GetSystemDirectoryW(buf, 260)
            cand = (buf.value + "\\WindowsPowerShell\\v1.0\\powershell.exe")
            if os.path.exists(cand):
                ps = cand
    except Exception:
        pass
    if ps:
        try:
            import subprocess
            out = subprocess.run(
                [ps, "-NoProfile", "-Command",
                 "(Get-CimInstance Win32_VideoController | "
                 "Sort-Object -Descending AdapterRAM | "
                 "Select-Object -First 1 -ExpandProperty Name)"],
                capture_output=True, text=True, timeout=10,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
            hw["gpu"] = (out.stdout or "").strip().splitlines()[0] \
                if out.stdout and out.stdout.strip() else ""
        except Exception:
            pass
    # 显存：N 卡用 nvidia-smi 精确拿（WMI 的 AdapterRAM 对 >4GB 不可靠）。
    # nvidia-smi 默认装在 NVSMI 目录，常不在 PATH
    if "nvidia" in hw["gpu"].lower():
        try:
            import shutil
            smi = shutil.which("nvidia-smi")
            if not smi:
                for cand in (
                        os.path.join(
                            os.environ.get("SystemRoot", r"C:\Windows"),
                            "System32", "nvidia-smi.exe"),
                        os.path.join(
                            os.environ.get("ProgramFiles", r"C:\Program Files"),
                            "NVIDIA Corporation", "NVSMI", "nvidia-smi.exe")):
                    if os.path.exists(cand):
                        smi = cand
                        break
            if smi:
                import subprocess
                out = subprocess.run(
                    [smi, "--query-gpu=memory.total",
                     "--format=csv,noheader,nounits"],
                    capture_output=True, text=True, timeout=8,
                    creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
                hw["vram_gb"] = round(int(out.stdout.strip()) / 1024, 1)
        except Exception:
            pass
    return hw


# 推荐档位：(最小RAM GB, 模型, 理由)——从高到低匹配
_MODEL_TIERS = [
    (32, "qwen2.5:14b", "分析最全面，适合深度黑匣子诊断"),
    (14, "qwen2.5:7b", "质量与速度平衡，多数电脑的最佳选择"),
    (7, "qwen2.5:3b", "轻快省内存，日常使用足够"),
    (0, "qwen2.5:1.5b", "极低配置保底，回答较简略"),
]


def recommend_model(hw: dict) -> tuple:
    """按硬件推荐模型，返回 (模型名, 理由, 本机描述)。
    规则：按 RAM 分档；N 卡显存 ≥10GB 升一档（GPU 推理快得多）。"""
    ram = hw.get("ram_gb", 0.0)
    vram = hw.get("vram_gb", 0.0)
    idx = next((i for i, (need, _, _) in enumerate(_MODEL_TIERS)
                if ram >= need), len(_MODEL_TIERS) - 1)
    upgraded = False
    if vram >= 10 and idx > 0:
        idx -= 1
        upgraded = True
    _, model, reason = _MODEL_TIERS[idx]
    parts = []
    if hw.get("cpu"):
        parts.append(f"{hw['cpu']} 线程")
    if ram:
        parts.append(f"{ram:.0f}GB 内存")
    if hw.get("gpu"):
        gpu = hw["gpu"]
        if vram:
            gpu += f"（{vram:.0f}GB 显存）"
        parts.append(gpu)
    desc = " / ".join(parts) or "硬件信息读取失败"
    if upgraded:
        reason += f"；检测到 {vram:.0f}GB 显存，已上调一档"
    return model, reason, desc


class AIBridge(QObject):
    """AI 对话桥：在后台线程调用 Ollama，流式输出通过信号送回界面"""

    token = pyqtSignal(str)      # 每收到一小段生成文字就发一次
    done = pyqtSignal()          # 一轮回答完整结束
    failed = pyqtSignal(str)     # 调用失败（服务没开/模型不存在/网络错误）

    def __init__(self):
        super().__init__()
        self._cancel = threading.Event()

    def cancel(self):
        """请求中断当前回答（由界面线程调用，线程安全）"""
        self._cancel.set()

    def chat(self, model: str, messages: list):
        """
        阻塞式流式对话（必须在后台线程调用）。
        messages: [{"role": "system"/"user"/"assistant", "content": "..."}, ...]
        """
        self._cancel.clear()
        body = json.dumps({
            "model": model,
            "messages": messages,
            "stream": True,
        }).encode("utf-8")
        req = urllib.request.Request(
            OLLAMA_BASE_URL + "/api/chat", data=body,
            headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=120) as resp:
                # 响应是逐行的 JSON 流，每行形如：
                # {"message":{"role":"assistant","content":"文字片段"},"done":false}
                for raw_line in resp:
                    if self._cancel.is_set():
                        break
                    line = raw_line.decode("utf-8", "ignore").strip()
                    if not line:
                        continue
                    try:
                        chunk = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    piece = chunk.get("message", {}).get("content", "")
                    if piece:
                        self.token.emit(piece)
                    if chunk.get("done"):
                        break
            self.done.emit()
        except urllib.error.URLError as e:
            self.failed.emit(
                f"无法连接 Ollama 服务：{e}\n请确认 Ollama 已启动。")
        except Exception as e:
            self.failed.emit(f"AI 调用失败：{e}")


