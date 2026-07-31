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


