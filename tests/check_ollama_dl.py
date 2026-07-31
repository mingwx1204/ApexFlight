# -*- coding: utf-8 -*-
"""多线程 Ollama 下载器回归：命名解析/进度文本/清单拉取/小 blob 全流程/
分片续传。真实模型端到端拉取用环境变量 FULL_PULL=模型名 触发（默认跳过）。

运行：python tests/check_ollama_dl.py
"""

import json
import os
import sys
import tempfile
import urllib.request
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import apex_ollama_dl as dl                                    # noqa: E402

# 1. 模型名解析
assert dl.parse_model_name("qwen2.5:7b") == ("library/qwen2.5", "7b")
assert dl.parse_model_name("qwen2.5") == ("library/qwen2.5", "latest")
assert dl.parse_model_name("user/foo:1b") == ("user/foo", "1b")
print("  ✅ 模型名解析")

# 2. 进度文本
p = dl.PullProgress(4_000_000_000, 16)
p.add(1_000_000_000)
txt = p.text("qwen2.5:7b")
assert "25%" in txt and "MB/s" in txt and "16 线程" in txt, txt
print(f"  ✅ 进度文本: {txt}")

# 3. 真实清单拉取（registry.ollama.ai）
raw = dl.fetch_manifest("library/qwen2.5", "1.5b")
mani = json.loads(raw)
assert mani["schemaVersion"] == 2 and mani["layers"], raw[:100]
print(f"  ✅ 清单拉取（{len(mani['layers'])} 层）")

# 4. 小 blob 全流程：config（487B）下载 + SHA256 校验
cfg_digest = mani["config"]["digest"]
cfg_size = int(mani["config"]["size"])
with tempfile.TemporaryDirectory() as td:
    dest = Path(td) / ("sha256-" + cfg_digest.split(":")[1])
    prog = dl.PullProgress(cfg_size, 4)
    dl.download_blob("library/qwen2.5", cfg_digest, cfg_size, dest, prog, 4)
    assert dest.exists() and dest.stat().st_size == cfg_size
print(f"  ✅ 小 blob 下载+校验（{cfg_size}B）")

# 5. 分片续传：先手工完成第一片，再跑全量应跳过该片
big = next(l for l in mani["layers"] if int(l["size"]) > 100_000_000)
with tempfile.TemporaryDirectory() as td:
    dest = Path(td) / ("sha256-" + big["digest"].split(":")[1])
    size = int(big["size"])
    with open(dest, "wb") as f:
        f.truncate(size)
    sidecar = dest.with_name(dest.name + ".chunks")
    sidecar.write_text(json.dumps([0]), encoding="utf-8")
    prog2 = dl.PullProgress(size, 8)
    # 只验证续传逻辑的前置状态识别（不真下载 986MB）
    done = dl._load_done_set(sidecar)
    assert done == {0}
print("  ✅ 分片续传状态识别")

# 6. 端到端真实拉取（默认跳过：FULL_PULL=qwen2.5:0.5b python tests/check_ollama_dl.py）
target = os.environ.get("FULL_PULL")
if target:
    msgs = []
    err = dl.pull_model_multithread(target, 16, msgs.append)
    assert err is None, err
    name, tag = dl.parse_model_name(target)
    mpath = (dl.ollama_models_dir() / "manifests"
             / "registry.ollama.ai" / name / tag)
    assert mpath.exists(), "manifest 未写入"
    with urllib.request.urlopen("http://localhost:11434/api/tags",
                                timeout=5) as r:
        tags = json.loads(r.read())
    names = [m["name"] for m in tags.get("models", [])]
    assert target in names, f"ollama 未识别: {names}"
    print(f"  ✅ 端到端拉取并被 ollama 识别: {target}")
    print(f"     末次进度: {msgs[-1] if msgs else '无'}")

print("OLLAMA_DL_OK")
