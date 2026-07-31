# -*- coding: utf-8 -*-
"""AI 确定性采样回归（v1.01）：json_mode 默认 temperature=0 + 固定 seed，
同样输入必须得到逐字节相同的输出——AI 全自动调参结果可复现、可对比。

依赖本机 Ollama + qwen2.5:7b；不满足时自动跳过（不判失败）。
运行：python tests/check_ai_determinism.py
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from apex_ai import chat_blocking, ollama_status               # noqa: E402

running, models = ollama_status()
if not running or "qwen2.5:7b" not in models:
    print("SKIP：本机没有 qwen2.5:7b（或 Ollama 未运行），跳过确定性实测")
    print("AI_DETERMINISM_SKIP")
    sys.exit(0)

msgs = [{"role": "system", "content": "你是调参专家，只输出JSON"},
        {"role": "user", "content":
         "输出一个JSON：{\"pid_roll\":[P,I,D]}，P/I/D各给50到80之间的数"}]

r1 = chat_blocking("qwen2.5:7b", msgs, timeout=120, json_mode=True)
r2 = chat_blocking("qwen2.5:7b", msgs, timeout=120, json_mode=True)
assert r1.strip(), "空回答"
assert r1 == r2, f"确定性模式两次结果不一致：\n{r1[:80]}\n{r2[:80]}"
print("  ✅ 两次调用输出逐字节一致")
print("AI_DETERMINISM_OK")
