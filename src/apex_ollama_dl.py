# -*- coding: utf-8 -*-
"""ApexFlight - Motrix 风格多线程 Ollama 模型下载器

为什么不用 `ollama pull`：官方 CLI 每层单连接，国内到
registry.ollama.ai 的链路单连接常被限速到几百 KB/s。
这里直接走 Ollama 官方仓库的 OCI API：
  1. GET /v2/<name>/manifests/<tag> 拿到层清单
  2. 每个 blob 用 HTTP Range 切成 8MiB 分片，N 个线程并发下载
     （分片完成状态写 sidecar 文件，中断后重开自动续传）
  3. SHA256 校验后放入 Ollama 本地模型库（~/.ollama/models/blobs），
     最后写 manifest —— ollama 服务实时识别，无需重启
任何一步失败都可回退到官方 `ollama pull`（由调用方决定）。
"""

import hashlib
import json
import os
import threading
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

REGISTRY = "https://registry.ollama.ai"
CHUNK_SIZE = 8 * 1024 * 1024          # 固定 8MiB 分片：重试代价小、续传粒度细
_UA = {"User-Agent": "ApexFlight/1.0 (ollama-mt-pull)"}


class PullError(Exception):
    """多线程拉取失败（调用方应回退官方 ollama pull）"""


def parse_model_name(model: str) -> tuple[str, str]:
    """qwen2.5:7b → (library/qwen2.5, 7b)；缺省命名空间 library，缺省 tag latest"""
    name, _, tag = model.partition(":")
    tag = tag or "latest"
    if "/" not in name:
        name = "library/" + name
    return name, tag


def ollama_models_dir() -> Path:
    """Ollama 模型库目录（尊重 OLLAMA_MODELS 环境变量）"""
    env = os.environ.get("OLLAMA_MODELS")
    if env:
        return Path(env)
    return Path.home() / ".ollama" / "models"


def _http_get(url: str, headers: dict | None = None, timeout: int = 30) -> bytes:
    req = urllib.request.Request(url, headers={**_UA, **(headers or {})})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return resp.read()


def fetch_manifest(name: str, tag: str, timeout: int = 30) -> bytes:
    """拉取模型清单（原始字节，写库时要原样保存）"""
    url = f"{REGISTRY}/v2/{name}/manifests/{tag}"
    return _http_get(url, {
        "Accept": "application/vnd.oci.image.manifest.v1+json"}, timeout)


class PullProgress:
    """跨线程字节计数器，生成 Motrix 风格进度文本"""

    def __init__(self, total_bytes: int, segments: int):
        self.total = max(total_bytes, 1)
        self.segments = segments
        self._done = 0
        self._lock = threading.Lock()
        self._t0 = time.time()

    def add(self, n: int):
        with self._lock:
            self._done += n

    def text(self, model: str) -> str:
        with self._lock:
            done = self._done
        dt = max(time.time() - self._t0, 0.1)
        speed = done / dt
        pct = min(done * 100 // self.total, 100)
        eta = int((self.total - done) / speed) if speed > 0 else 0
        return (f"{model} {pct}%｜{done / 2 ** 30:.2f}/"
                f"{self.total / 2 ** 30:.2f} GB｜{speed / 2 ** 20:.1f} MB/s"
                f"｜剩 {eta // 60}m{eta % 60:02d}s｜{self.segments} 线程")


def _load_done_set(sidecar: Path) -> set:
    try:
        return set(json.loads(sidecar.read_text(encoding="utf-8")))
    except Exception:                                           # noqa: BLE001
        return set()


def download_blob(name: str, digest: str, size: int, dest: Path,
                  prog: PullProgress, segments: int = 16):
    """分片并发下载一个 blob 到 dest，带分片级断点续传 + SHA256 校验"""
    url = f"{REGISTRY}/v2/{name}/blobs/{digest}"
    dest.parent.mkdir(parents=True, exist_ok=True)
    sidecar = dest.with_name(dest.name + ".chunks")
    ranges = [(s, min(s + CHUNK_SIZE, size) - 1)
              for s in range(0, size, CHUNK_SIZE)]

    done_set = set()
    if sidecar.exists() and dest.exists() and dest.stat().st_size == size:
        done_set = _load_done_set(sidecar)          # 续传：复用已完成分片
    if not done_set:
        with open(dest, "wb") as f:                 # 重新预分配
            f.truncate(size)

    finished = sorted(i for i in done_set if 0 <= i < len(ranges))
    if finished:
        prog.add(sum(ranges[i][1] - ranges[i][0] + 1 for i in finished))
    done_set = set(finished)
    todo = [i for i in range(len(ranges)) if i not in done_set]

    lock = threading.Lock()

    def work(i: int):
        s, e = ranges[i]
        want = e - s + 1
        last = None
        for attempt in range(4):
            try:
                data = _http_get(url, {"Range": f"bytes={s}-{e}"}, 120)
                if len(data) != want:
                    raise PullError(f"分片大小不符 {len(data)} != {want}")
                break
            except Exception as e_:                  # noqa: BLE001
                last = e_
                time.sleep(1.2 * (attempt + 1))
        else:
            raise PullError(f"分片 @{s} 多次重试仍失败：{last}")
        with open(dest, "r+b") as f:
            f.seek(s)
            f.write(data)
        with lock:
            done_set.add(i)
            try:
                sidecar.write_text(json.dumps(sorted(done_set)),
                                   encoding="utf-8")
            except OSError:
                pass
        prog.add(want)

    if todo:
        with ThreadPoolExecutor(max_workers=max(1, segments)) as ex:
            list(ex.map(work, todo))
    sidecar.unlink(missing_ok=True)

    expect = digest.split(":", 1)[1]
    if _sha256_of(dest) != expect:
        dest.unlink(missing_ok=True)
        raise PullError(f"SHA256 校验失败：{digest[:19]}…（已删除坏文件）")


def _sha256_of(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for blk in iter(lambda: f.read(4 * 1024 * 1024), b""):
            h.update(blk)
    return h.hexdigest()


def _try_reuse_official_partial(blob_dir: Path, digest: str, size: int,
                                final: Path, on_progress=None) -> bool:
    """官方 `ollama pull` 的 -partial 文件若已下满且校验通过，直接转正入库，
    避免用户之前白等了几十分钟的下载被浪费（v1.00）"""
    base = "sha256-" + digest.split(":", 1)[1]
    partial = blob_dir / (base + "-partial")
    if not (partial.exists() and partial.stat().st_size == size):
        return False
    if on_progress:
        on_progress("检测到官方下载残留的完整分片，校验后直接入库…")
    if _sha256_of(partial) != digest.split(":", 1)[1]:
        return False                                # 内容坏：不用，重下
    final.parent.mkdir(parents=True, exist_ok=True)
    if final.exists():
        final.unlink()
    os.replace(partial, final)
    return True


def pull_model_multithread(model: str, segments: int = 16,
                           on_progress=None) -> str | None:
    """多线程拉取模型进 Ollama 本地库。

    on_progress(text) 约每 0.6s 回调一次（后台线程，勿直接碰 UI）。
    成功返回 None；失败返回错误描述（调用方回退官方 pull）。
    """
    try:
        name, tag = parse_model_name(model)
        store = ollama_models_dir()
        blob_dir = store / "blobs"
        tmp_dir = blob_dir / ".apexflight_tmp"

        manifest_raw = fetch_manifest(name, tag)
        manifest = json.loads(manifest_raw)
        items = [manifest["config"], *manifest.get("layers", [])]

        # 计划：已在库中且尺寸吻合的 blob 直接跳过（幂等）；
        # 官方 pull 残留的完整 -partial 校验后转正（v1.00）
        plan = []
        for it in items:
            digest, size = it["digest"], int(it["size"])
            final = blob_dir / ("sha256-" + digest.split(":", 1)[1])
            if final.exists() and final.stat().st_size == size:
                continue
            if _try_reuse_official_partial(blob_dir, digest, size, final,
                                           on_progress):
                continue
            if final.exists() and final.stat().st_size == size:
                continue  # 竞态兜底：校验期间 ollama 服务自己完成了入库
            plan.append((digest, size, final))

        prog = PullProgress(sum(s for _, s, _ in plan), segments)
        stop = threading.Event()

        def _tick():
            while not stop.wait(0.6):
                if on_progress:
                    on_progress(prog.text(model))

        if on_progress:
            on_progress(prog.text(model))
        t = threading.Thread(target=_tick, daemon=True)
        t.start()
        try:
            for digest, size, final in plan:
                tmpf = tmp_dir / ("sha256-" + digest.split(":", 1)[1])
                download_blob(name, digest, size, tmpf, prog, segments)
                final.parent.mkdir(parents=True, exist_ok=True)
                os.replace(tmpf, final)
        finally:
            stop.set()

        # 最后写清单：ollama 服务扫描到清单即识别模型
        mpath = store / "manifests" / "registry.ollama.ai" / name / tag
        mpath.parent.mkdir(parents=True, exist_ok=True)
        mpath.write_bytes(manifest_raw)
        if on_progress:
            on_progress(f"{model} 100%｜校验完成，已入库")
        return None
    except Exception as e:                                      # noqa: BLE001
        return str(e)
