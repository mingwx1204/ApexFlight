# -*- coding: utf-8 -*-
"""ApexFlight 扫频调参引擎（v0.98）：从黑匣子日志做系统辨识（无 AI）。

原理（与 BF2026 Chirp Autotune 同源的教科书方法）：
- 输入 x = setpoint[axis]（目标角速度 °/s），输出 y = gyroADC[axis]（实测）
- Welch 互谱法估计频率响应：H(f) = Pxy / Pxx（Hann 窗 + 50% 重叠平均）
- 相干性 γ²(f) = |Pxy|²/(Pxx·Pyy)：激励是否充分的质量门（BF 同款 coherence）
- 由闭环 H 推导开环 L = H/(1-H) 算相位裕度；灵敏度 S = 1 - H（>6dB 危险）
- 指标：低频跟踪平台、-3dB 带宽、相位裕度、灵敏度峰值/频率、谐振峰
- 建议引擎：纯规则 + 公式推导（确定性、可复现），绝不"猜"

与 BF Chirp 的关系：BF 靠固件注入扫频激励；本模块用普通飞行日志里
翻滚/甩杆的宽频激励做同样的辨识。日志激励不足时相干性会低，
此时如实提示重飞一段"油门斜坡 + 单轴激进动作"再来分析。
"""

import csv
import math

import numpy as np

# 分析频段：低于 1Hz 是姿态/漂移区，高于 600Hz 超出调参意义
F_MIN, F_MAX = 1.0, 600.0
# 数据质量门：功率加权相干性低于此值时建议重飞（BF 经验值 80%+）
COHERENCE_GATE = 0.78
# 灵敏度峰值危险线（dB）：BF Autotune 讨论的不稳定阈值约 6dB
SENSITIVITY_DANGER_DB = 6.0

AXES = (("横滚", 0), ("俯仰", 1), ("偏航", 2))


class SweepError(Exception):
    """扫频分析的可预期错误（数据不足/缺通道等），界面直接展示"""


# ------------------------------------------------------------
# 日志读取（全采样率，只取需要的列；与绘图的抽稀加载分离）
# ------------------------------------------------------------
def load_log_series(csv_path, needed=("gyroADC[0]", "gyroADC[1]", "gyroADC[2]",
                                      "setpoint[0]", "setpoint[1]",
                                      "setpoint[2]"),
                    max_rows: int = 1_200_000):
    """读取黑匣子 CSV 中分析所需的列，返回 (采样率Hz, {列名: np.ndarray})。

    - 采样率由时间列的中位间隔推算（抗抖动）
    - 行数超过 max_rows 时等间隔抽稀并同步折算采样率（防内存爆炸），
      抽稀会压低奈奎斯特频率，调用方需知晓
    """
    with open(csv_path, "r", encoding="utf-8", errors="replace") as f:
        reader = csv.reader(f)
        header = [name.strip() for name in next(reader)]
        time_idx = next((i for i, n in enumerate(header)
                         if n.lower().startswith("time")), None)
        if time_idx is None:
            raise SweepError("CSV 中没有 time 列，不是标准黑匣子日志")
        col_idx = {}
        for name in needed:
            if name in header:
                col_idx[name] = header.index(name)
        if len(col_idx) < 2:
            raise SweepError("日志缺少 gyroADC/setpoint 通道，无法做扫频分析")

        keep = sorted([time_idx] + list(col_idx.values()))
        rows = []
        for row in reader:
            rows.append(row)
            if len(rows) > max_rows * 2:      # 先粗读，超限再抽稀
                break

    total = len(rows)
    stride = max(1, total // max_rows)
    idx_map = {v: k for k, v in col_idx.items()}
    data = {name: [] for name in col_idx}
    times = []
    for row in rows[::stride]:
        try:
            times.append(float(row[time_idx]))
            for ci in col_idx.values():
                data[idx_map[ci]].append(float(row[ci]) if ci < len(row)
                                         else float("nan"))
        except (ValueError, IndexError):
            continue
    if len(times) < 2000:
        raise SweepError(f"有效数据太少（{len(times)} 行），"
                         f"需要至少几秒的全速率日志才能辨识")
    dt = np.median(np.diff(times))            # 微秒
    if dt <= 0:
        raise SweepError("时间列异常（间隔非正），日志损坏？")
    fs = 1_000_000.0 / dt
    series = {name: np.asarray(vals, dtype=float) for name, vals in data.items()}
    return fs, series


# ------------------------------------------------------------
# 频率响应估计（Welch 互谱法）
# ------------------------------------------------------------
def estimate_response(x: np.ndarray, y: np.ndarray, fs: float,
                      window_s: float = 4.0):
    """估计 x→y 的频率响应。
    返回 (freqs, mag_db, phase_deg, coherence)，无效频段填 NaN。"""
    n = len(x)
    nperseg = int(window_s * fs)
    nperseg = min(nperseg, n)
    if nperseg < 256:
        raise SweepError(f"数据窗口太短（{nperseg} 点），无法估计频率响应")
    window = np.hanning(nperseg)
    step = nperseg // 2
    # 去均值，抑制直流分量
    x = x - np.mean(x)
    y = y - np.mean(y)

    sxx = np.zeros(nperseg // 2 + 1)
    syy = np.zeros_like(sxx)
    sxy = np.zeros_like(sxx, dtype=complex)
    count = 0
    for start in range(0, n - nperseg + 1, step):
        xs = x[start:start + nperseg] * window
        ys = y[start:start + nperseg] * window
        xf = np.fft.rfft(xs)
        yf = np.fft.rfft(ys)
        sxx += np.abs(xf) ** 2
        syy += np.abs(yf) ** 2
        sxy += np.conj(xf) * yf
        count += 1
    if count < 2:
        raise SweepError("可分段数不足（日志太短），无法平均估计")

    freqs = np.fft.rfftfreq(nperseg, 1.0 / fs)
    with np.errstate(divide="ignore", invalid="ignore"):
        h = np.where(sxx > 0, sxy / sxx, np.nan)
        coh = np.where((sxx > 0) & (syy > 0),
                       np.abs(sxy) ** 2 / (sxx * syy), np.nan)
    mag_db = 20 * np.log10(np.abs(h))
    phase_deg = np.degrees(np.angle(h))
    coh = np.clip(np.real(coh), 0.0, 1.0)
    return freqs, mag_db, phase_deg, coh, sxx


# ------------------------------------------------------------
# 单轴指标：带宽 / 相位裕度 / 灵敏度峰值 / 谐振峰 / 相干性
# ------------------------------------------------------------
def analyze_axis(x: np.ndarray, y: np.ndarray, fs: float) -> dict:
    """对单轴做完整系统辨识，返回指标字典（含绘图数据）"""
    freqs, mag_db, phase_deg, coh, sxx = estimate_response(x, y, fs)
    nyq = fs / 2.0
    f_hi = min(F_MAX, nyq * 0.9)
    band = (freqs >= F_MIN) & (freqs <= f_hi)

    # ---- 激励区：输入功率不低于峰值 1% 的频段。
    # 相干性在强泄漏处会虚高，功率门才是"真的激励过"的判据 ----
    sxx_band = np.where(band, sxx, 0.0)
    sxx_max = np.nanmax(sxx_band) if np.any(sxx_band > 0) else np.nan
    excited = band & np.isfinite(sxx) & (sxx > 0.01 * sxx_max) \
        if sxx_max == sxx_max else band & False

    # ---- 低频跟踪平台（理想≈0dB）：激励区最低 10% 频点的中位幅值 ----
    plateau = float("nan")
    if np.any(excited):
        ex_f = freqs[excited]
        f_cap = ex_f[max(1, len(ex_f) // 10)]
        low = excited & (freqs <= f_cap)
        plateau = float(np.nanmedian(mag_db[low]))

    # ---- 带宽：幅值跌到平台-3dB 以下的最低频率（保持 3 个频点防毛刺）----
    bandwidth = float("nan")
    if not math.isnan(plateau):
        below = excited & (mag_db < (plateau - 3.0))
        idxs = np.where(below)[0]
        for i in idxs:
            if i + 2 < len(below) and np.all(below[i:i + 3]):
                bandwidth = float(freqs[i])
                break

    # ---- 相位裕度：开环 L = H/(1-H)，找 |L|=0dB 穿越点 ----
    h = 10 ** (mag_db / 20) * np.exp(1j * np.radians(phase_deg))
    denom = 1 - h
    valid = excited & (np.abs(denom) > 0.05) & np.isfinite(mag_db)
    pm, f_gc = float("nan"), float("nan")
    if np.any(valid):
        L = np.full_like(h, np.nan, dtype=complex)
        L[valid] = h[valid] / denom[valid]
        l_db = 20 * np.log10(np.abs(L))
        # 从低频向高频找 0dB 穿越
        cross = np.where((l_db[:-1] >= 0) & (l_db[1:] < 0)
                         & valid[:-1] & valid[1:])[0]
        if len(cross):
            i = cross[0]
            f_gc = float(freqs[i])
            pm = float(180 + np.degrees(np.angle(L[i])))

    # ---- 灵敏度 S = 1-H：峰值越大越接近振荡（>6dB 危险）。
    # 只看激励区 + 相干性合格的频段——没有激励的地方幅值估计是噪声 ----
    s_db = 20 * np.log10(np.maximum(np.abs(denom), 1e-9))
    s_region = excited & np.isfinite(s_db) & (coh > 0.3)
    sens_peak_db, sens_peak_hz = float("nan"), float("nan")
    if np.any(s_region):
        i = int(np.nanargmax(np.where(s_region, s_db, -np.inf)))
        sens_peak_db = float(s_db[i])
        sens_peak_hz = float(freqs[i])

    # ---- 谐振峰：80Hz 以上、高出平台 3dB 且激励/相干性合格的局部极大 ----
    resonances = []
    if not math.isnan(plateau):
        res_region = (freqs >= 80) & (freqs <= f_hi) & excited & (coh > 0.5)
        m = np.where(res_region, mag_db, -np.inf)
        for i in range(1, len(m) - 1):
            if (m[i] > m[i - 1] and m[i] >= m[i + 1]
                    and m[i] > plateau + 3.0 and np.isfinite(m[i])):
                resonances.append((round(float(freqs[i]), 1),
                                   round(float(mag_db[i]), 1)))
    resonances = resonances[:4]

    # ---- 相干性：激励区 2~150Hz 的中位数（质量门，BF 经验 80%+ 可信）----
    gate_band = excited & (freqs >= 2.0) & (freqs <= min(150.0, f_hi))
    coherence = float("nan")
    if np.any(gate_band):
        coherence = float(np.nanmedian(coh[gate_band]))

    return {
        "freqs": freqs, "mag_db": mag_db, "phase_deg": phase_deg, "coh": coh,
        "plateau_db": round(plateau, 2),
        "bandwidth_hz": round(bandwidth, 1) if bandwidth == bandwidth else None,
        "phase_margin_deg": round(pm, 1) if pm == pm else None,
        "gain_crossover_hz": round(f_gc, 1) if f_gc == f_gc else None,
        "sensitivity_peak_db": (round(sens_peak_db, 1)
                                if sens_peak_db == sens_peak_db else None),
        "sensitivity_peak_hz": (round(sens_peak_hz, 1)
                                if sens_peak_hz == sens_peak_hz else None),
        "resonances": resonances,
        "coherence": round(coherence, 3) if coherence == coherence else None,
        "fs": round(fs, 1),
    }


# ------------------------------------------------------------
# 建议引擎：纯规则 + 公式（每条都带依据），绝不臆测
# ------------------------------------------------------------
def recommend(metrics_by_axis: dict, current_pids: dict | None = None) -> list:
    """根据三轴辨识指标生成精准参数建议。

    metrics_by_axis: {"横滚": analyze_axis(...), ...}
    current_pids: 可选 {"roll": (P,I,D), "pitch": (P,I,D), "yaw": (P,I,D)}
    返回 [{param, axis, current, suggested, change, reason, level}]，
    level: info（参考）/ action（建议修改）/ danger（优先处理）
    """
    out = []
    names = list(metrics_by_axis.keys())
    # 数据质量门：任一轴相干性低 → 只给重飞建议
    cohs = {a: m.get("coherence") for a, m in metrics_by_axis.items()}
    low_coh = [a for a, c in cohs.items() if c is not None
               and c < COHERENCE_GATE]
    if low_coh:
        out.append({
            "param": "数据质量", "axis": "、".join(low_coh),
            "current": "—", "suggested": "重新采集", "change": "—",
            "level": "danger",
            "reason": (f"相干性 {min(c for c in cohs.values() if c is not None):.0%}"
                       f" < {COHERENCE_GATE:.0%}：激励不充分，辨识结果不可信。"
                       f"请飞一段「3 次油门斜坡 + 各轴 2~3 次快速翻滚/甩杆」"
                       f"约 20 秒再分析（或刷 BF2026 用 Chirp 模式采集）"),
        })
        return out

    # ---- 规则 1：轴间带宽失衡 → 慢轴 P/D 按比例上调 ----
    bws = {a: m.get("bandwidth_hz") for a, m in metrics_by_axis.items()}
    valid_bw = {a: b for a, b in bws.items() if b}
    if len(valid_bw) >= 2:
        bw_max = max(valid_bw.values())
        fastest = max(valid_bw, key=valid_bw.get)
        for a, b in valid_bw.items():
            if a != fastest and b < 0.8 * bw_max:
                pct = min(30, max(5, round((bw_max / b - 1) * 50)))
                out.append({
                    "param": "PID", "axis": a,
                    "current": _cur_pid(current_pids, a),
                    "suggested": f"P/D × {1 + pct / 100:.2f}",
                    "change": f"+{pct}%", "level": "action",
                    "reason": (f"{a}带宽 {b:.0f}Hz 仅为{fastest} {bw_max:.0f}Hz 的"
                               f" {b / bw_max:.0%}，带宽差 >20% 说明该轴增益偏低。"
                               f"上调幅度 = (带宽比-1)×50% = {pct}%"),
                })

    # ---- 规则 2：相位裕度 ----
    for a in names:
        pm_val = metrics_by_axis[a].get("phase_margin_deg")
        if pm_val is None:
            continue
        if pm_val < 45:
            out.append({
                "param": "D 项 / 阻尼", "axis": a,
                "current": _cur_pid(current_pids, a),
                "suggested": "D × 0.90 或 D 滤波 -10%",
                "change": "-10%", "level": "danger",
                "reason": (f"相位裕度 {pm_val:.0f}° < 45°，已接近振荡边缘"
                           f"（BF 经验：>60° 稳健）。降低 D 或加强 D 项滤波"
                           f"可换回相位裕度"),
            })
        elif pm_val > 100:
            out.append({
                "param": "增益余量", "axis": a,
                "current": "—", "suggested": "可适度加 P 或放松滤波",
                "change": "余量", "level": "info",
                "reason": (f"相位裕度 {pm_val:.0f}° > 100°，系统很保守，"
                           f"还有提速空间（每次 +5~10% 试飞验证）"),
            })

    # ---- 规则 3：灵敏度峰值 >6dB → 接近失稳 ----
    for a in names:
        sp_db = metrics_by_axis[a].get("sensitivity_peak_db")
        sp_hz = metrics_by_axis[a].get("sensitivity_peak_hz")
        if sp_db is None or sp_hz is None:
            continue
        if sp_db > SENSITIVITY_DANGER_DB:
            out.append({
                "param": "滤波器", "axis": a,
                "current": "—",
                "suggested": f"检查 {sp_hz:.0f}Hz 附近谐振；动态陷波覆盖之",
                "change": "优先", "level": "danger",
                "reason": (f"灵敏度峰值 {sp_db:.1f}dB @ {sp_hz:.0f}Hz 超过"
                           f" {SENSITIVITY_DANGER_DB:.0f}dB 危险线：外部扰动在"
                           f"该频率会被放大 {10 ** (sp_db / 20):.1f} 倍，"
                           f"洗桨/阵风时会振荡"),
            })

    # ---- 规则 4：明显谐振峰 → 陷波/低通建议 ----
    for a in names:
        for f_r, m_r in metrics_by_axis[a].get("resonances", []):
            out.append({
                "param": "陷波滤波", "axis": a,
                "current": "—",
                "suggested": f"陷波中心 {f_r:.0f}Hz（峰 {m_r:.1f}dB）",
                "change": f"{f_r:.0f}Hz", "level": "action",
                "reason": (f"{a}在 {f_r:.0f}Hz 存在高出低频平台 "
                           f"{m_r:.1f}dB 的谐振峰，通常是机架/电机共振；"
                           f"动态陷波范围应覆盖它，或检查该频段低通是否太松"),
            })

    # ---- 规则 5：低频跟踪不足 ----
    for a in names:
        pl = metrics_by_axis[a].get("plateau_db")
        if pl is not None and pl < -1.5:
            out.append({
                "param": "I 项 / 跟踪", "axis": a,
                "current": _cur_pid(current_pids, a),
                "suggested": "I × 1.10 或检查陀螺仪低通过重",
                "change": "+10%", "level": "action",
                "reason": (f"1~5Hz 跟踪平台 {pl:.1f}dB（理想≈0dB）："
                           f"低频都跟不住，常见于 I 不足或陀螺仪滤波过重"),
            })

    if not out:
        out.append({
            "param": "综合", "axis": "三轴", "current": "—",
            "suggested": "维持当前参数", "change": "0", "level": "info",
            "reason": ("带宽均衡、相位裕度健康、灵敏度峰值在安全线内、"
                       "无突出谐振——当前调参状态良好"),
        })
    return out


def _cur_pid(current_pids: dict | None, axis_cn: str) -> str:
    if not current_pids:
        return "—"
    key = {"横滚": "roll", "俯仰": "pitch", "偏航": "yaw"}.get(axis_cn, "")
    v = current_pids.get(key)
    return f"P{v[0]}/I{v[1]}/D{v[2]}" if v else "—"


# ------------------------------------------------------------
# 顶层入口：一段日志 → 三轴指标 + 建议
# ------------------------------------------------------------
def analyze_log(csv_path, current_pids: dict | None = None) -> dict:
    """分析一段黑匣子日志（CSV），返回绘图与建议所需的全部数据"""
    fs, series = load_log_series(csv_path)
    metrics = {}
    for name, axis in AXES:
        sp, gy = f"setpoint[{axis}]", f"gyroADC[{axis}]"
        if sp not in series or gy not in series:
            continue
        x = np.nan_to_num(series[sp])
        y = np.nan_to_num(series[gy])
        if np.std(x) < 1.0:
            # 该轴设定值几乎没动 → 无激励，跳过并如实说明
            metrics[name] = {"error": f"{name}设定值几乎无变化（激励不足）",
                             "coherence": 0.0}
            continue
        metrics[name] = analyze_axis(x, y, fs)
    if not any("bandwidth_hz" in m or "mag_db" in m for m in metrics.values()):
        raise SweepError("三个轴都缺少有效激励（日志太平静），"
                         "请用包含翻滚/甩杆动作的飞行日志")
    good = {a: m for a, m in metrics.items() if "error" not in m}
    suggestions = recommend(good, current_pids)
    return {"fs": fs, "axes": metrics, "suggestions": suggestions}
