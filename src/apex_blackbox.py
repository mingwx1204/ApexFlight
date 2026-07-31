# -*- coding: utf-8 -*-
"""ApexFlight - 黑匣子日志解码、统计分析与飞行/空转日志类型判别"""

import csv
import subprocess
from pathlib import Path

from apex_fc import BLACKBOX_DECODER, LOGS_DIR  # noqa: F401

# ============================================================
# 第四部分：黑匣子日志分析
# ============================================================
# 支持两类文件：
#   1. .bbl / .bfl 二进制日志 —— 用官方 blackbox_decode.exe 转成 CSV 再解析
#   2. .csv（blackbox_decode 的输出）—— 直接解析
# CSV 首行是列名，常见列：
#   time（微秒）、gyroADC[0..2]（陀螺仪三轴）、setpoint[0..3]（设定值）、
#   rcCommand[0..3]（遥控器指令）、axisP/I/D[0..2]（PID 各项输出）、
#   motor[0..7]（电机输出）、vbatLatest（电压）、amperageLatest（电流）

# 通道中文名对照（未列出的通道直接显示原始列名）
CHANNEL_NAMES = {
    "gyroADC[0]": "陀螺仪·横滚 (°/s)",
    "gyroADC[1]": "陀螺仪·俯仰 (°/s)",
    "gyroADC[2]": "陀螺仪·偏航 (°/s)",
    "setpoint[0]": "设定值·横滚",
    "setpoint[1]": "设定值·俯仰",
    "setpoint[2]": "设定值·偏航",
    "setpoint[3]": "设定值·油门",
    "rcCommand[0]": "遥控指令·横滚",
    "rcCommand[1]": "遥控指令·俯仰",
    "rcCommand[2]": "遥控指令·偏航",
    "rcCommand[3]": "遥控指令·油门",
    "axisP[0]": "P 项·横滚", "axisP[1]": "P 项·俯仰", "axisP[2]": "P 项·偏航",
    "axisI[0]": "I 项·横滚", "axisI[1]": "I 项·俯仰", "axisI[2]": "I 项·偏航",
    "axisD[0]": "D 项·横滚", "axisD[1]": "D 项·俯仰", "axisD[2]": "D 项·偏航",
    "motor[0]": "电机 1 输出", "motor[1]": "电机 2 输出",
    "motor[2]": "电机 3 输出", "motor[3]": "电机 4 输出",
    "vbatLatest": "电池电压 (0.01V)",
    "amperageLatest": "电流 (0.01A)",
    "rssi": "信号强度 RSSI",
}

# 通道用途说明（鼠标悬停 / 「通道用途」按钮都会用到）
CHANNEL_HELP = {
    "gyroADC[0]": "飞机实际的横滚角速度（°/s）。调参最核心的通道：和设定值对比看"
                  "跟踪效果；波形毛刺多 = 滤波不够，来回振荡 = P/D 过高。",
    "gyroADC[1]": "飞机实际的俯仰角速度（°/s）。用途同横滚：看响应是否跟手、"
                  "有没有振动噪声。",
    "gyroADC[2]": "飞机实际的偏航角速度（°/s）。偏航振荡常见于机架共振或"
                  "偏航 PID 过激。",
    "setpoint[0]": "飞控根据打杆算出的横滚目标角速度。和 gyroADC[0] 对比："
                   "滞后大 → 可加 P 或 FeedForward；超调多 → 加 D 或减 P。",
    "setpoint[1]": "俯仰目标角速度，用途同上。",
    "setpoint[2]": "偏航目标角速度，用途同上。",
    "setpoint[3]": "目标油门。看油门突变时其他通道是否被干扰（掉压导致抖动）。",
    "rcCommand[0]": "遥控器横滚原始指令。排查打杆没反应、通道反向、"
                    "接收机信号抖动时用。",
    "rcCommand[1]": "遥控器俯仰原始指令，用途同上。",
    "rcCommand[2]": "遥控器偏航原始指令，用途同上。",
    "rcCommand[3]": "遥控器油门原始指令，用途同上。",
    "axisP[0]": "P 项（比例）横滚输出。波形高频振荡 → P 太高；"
                "跟踪缓慢无力 → P 偏低。",
    "axisP[1]": "P 项俯仰输出，用途同上。",
    "axisP[2]": "P 项偏航输出，用途同上。",
    "axisI[0]": "I 项（积分）横滚输出，负责消除持续误差（风阻、重心偏移）。"
                "长期偏离零属正常；机身缓慢来回摆动 → I 太高。",
    "axisI[1]": "I 项俯仰输出，用途同上。",
    "axisI[2]": "I 项偏航输出，用途同上。",
    "axisD[0]": "D 项（微分）横滚输出，抑制过冲和回弹。D 太大 → 电机发热、"
                "放大高频噪声；洗桨（急转后抖动）明显 → 可适当加 D。",
    "axisD[1]": "D 项俯仰输出，用途同上。",
    "axisD[2]": "D 项偏航输出，用途同上。",
    "motor[0]": "电机 1 输出（1000~2000）。长期贴顶（≈2000 饱和）→ 重心偏、"
                "机架损伤或该轴 PID 过激；几个电机差值大 → 机架不对称。",
    "motor[1]": "电机 2 输出，用途同上。",
    "motor[2]": "电机 3 输出，用途同上。",
    "motor[3]": "电机 4 输出，用途同上。",
    "vbatLatest": "电池电压。满油门时掉压厉害 → 电池老化或放电倍率不够；"
                  "松油门后回升缓慢 → 内阻偏大。",
    "amperageLatest": "瞬时电流。看功耗峰值、排查异常耗电。",
    "rssi": "链路信号强度（0~99 或百分比）。定位失控、信号弱发生在什么时间。",
}
CHANNEL_HELP_DEFAULT = ("该通道的详细含义可对照 Betaflight 黑匣子文档。"
                        "一般来说： gyro 类看噪声与跟踪，setpoint/rcCommand 类"
                        "看输入，axisP/I/D 类看 PID 各项出力，motor 类看饱和。")


class BlackboxError(Exception):
    """黑匣子日志处理错误"""
    pass


def decode_blackbox(log_path: Path) -> list:
    """
    用官方 blackbox_decode.exe 把 .bbl/.bfl 二进制日志解码成 CSV。
    一个 .bbl 文件通常包含多段飞行记录（每次解锁一段），
    解码器会生成 "文件名.01.csv"、"文件名.02.csv"……
    返回：全部生成的 CSV 路径列表（按段号排序）。
    """
    if not BLACKBOX_DECODER.exists():
        raise BlackboxError(f"未找到解码器：{BLACKBOX_DECODER}")
    before = set(log_path.parent.glob("*.csv"))
    result = subprocess.run(
        [str(BLACKBOX_DECODER), str(log_path)],
        cwd=str(BLACKBOX_DECODER.parent),     # DLL 在 tools 目录里
        capture_output=True, timeout=300,
    )
    if result.returncode != 0:
        raise BlackboxError(
            f"解码失败（可能不是有效的黑匣子日志）：\n"
            f"{result.stderr.decode('gbk', errors='replace')[:300]}")
    new_csvs = sorted(set(log_path.parent.glob("*.csv")) - before)
    if not new_csvs:
        raise BlackboxError("解码器没有产出 CSV 文件")
    return new_csvs


def parse_bbl_header(log_path: Path) -> dict:
    """
    直接解析 .bbl/.bfl 文件的头部信息（头部是纯文本行，以 "H " 开头）。
    返回关键信息字典：固件版本、机名、板子、循环频率、日期等。
    解析不到时返回空字典（例如直接打开 CSV 的情况）。
    """
    info = {}
    # 我们关心的头部字段 -> 中文显示名
    wanted = {
        "Firmware type": "固件类型",
        "Firmware revision": "固件版本",
        "Firmware date": "固件日期",
        "Craft name": "机名",
        "Board information": "板子信息",
        "looptime": "PID 循环周期 (µs)",
        "gyro_scale": "陀螺仪量程",
        "acc_1G": "加速度计 1G 值",
        "vbatref": "电压基准",
        "currentMeter": "电流计",
        "motorOutput": "电机输出范围",
        "rc_rate": "RC 速率",
        "rc_expo": "RC  expo",
        "rates": " Rates",
        "rollPID": "Roll PID",
        "pitchPID": "Pitch PID",
        "yawPID": "Yaw PID",
        "dterm_filter_type": "D 项滤波类型",
        "gyro_lowpass_hz": "陀螺仪低通 (Hz)",
        "dterm_lowpass_hz": "D 项低通 (Hz)",
    }
    try:
        # 头部在文件开头，读前 64KB 足够
        with open(log_path, "rb") as f:
            raw = f.read(65536)
        text = raw.decode("latin-1", errors="replace")
        for line in text.split("\n"):
            line = line.strip("\x00").strip()
            if not line.startswith("H "):
                continue
            body = line[2:]
            if ":" not in body:
                continue
            key, _, value = body.partition(":")
            key, value = key.strip(), value.strip()
            if key in wanted and key not in info:
                info[key] = value
    except OSError:
        pass
    # 转换成 {中文名: 值} 返回，保持 wanted 中的顺序
    return {wanted[k]: v for k, v in info.items() if k in wanted}


def load_blackbox_csv(csv_path: Path) -> tuple:
    """
    读取黑匣子 CSV 文件。
    返回：(时间轴[秒] 列表, {列名: 数值列表}, 原始列名列表)
    大文件自动抽稀到约 10 万点：绘图流畅，同时保留足够采样率做频谱分析。
    """
    with open(csv_path, "r", encoding="utf-8", errors="replace") as f:
        reader = csv.reader(f)
        # blackbox_decode 生成的表头形如 "loopIteration, time (us), axisP[0], ..."
        # 列名带前导空格，且时间列叫 "time (us)" 而不是 "time"，
        # 这里统一去掉空格，并兼容两种时间列名
        header = [name.strip() for name in next(reader)]
        columns = {name: [] for name in header}
        for row in reader:
            for name, cell in zip(header, row):
                try:
                    columns[name].append(float(cell))
                except ValueError:
                    columns[name].append(float("nan"))

    time_name = next((n for n in columns if n.lower().startswith("time")), None)
    if time_name is None or not columns[time_name]:
        raise BlackboxError("CSV 中没有 time 列，不是标准的黑匣子日志")
    time_col = columns[time_name]

    count = len(time_col)
    stride = max(1, count // 100000)
    time_axis = [t / 1_000_000 for t in time_col[::stride]]   # 微秒 → 秒
    data = {name: vals[::stride] for name, vals in columns.items()
            if name != time_name}
    kept = [name for name in header if name != time_name]
    return time_axis, data, kept


def generate_demo_log() -> Path:
    """
    生成一段 10 秒的演示日志（CSV 格式），让没有黑匣子日志的
    用户也能立即体验曲线分析功能。内容模拟一次翻滚动作。
    """
    import math
    import random

    LOGS_DIR.mkdir(exist_ok=True)
    path = LOGS_DIR / "demo_flight.csv"
    random.seed(42)
    header = ["time", "gyroADC[0]", "gyroADC[1]", "gyroADC[2]",
              "setpoint[0]", "setpoint[1]", "setpoint[2]",
              "motor[0]", "motor[1]", "motor[2]", "motor[3]",
              "vbatLatest"]
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for i in range(5000):                 # 10 秒 × 500Hz
            t_us = i * 2000
            t = t_us / 1_000_000
            # 3~4 秒时打一次满杆横滚（设定值阶跃 + 陀螺仪跟随响应）
            sp = 500 if 3.0 <= t < 4.0 else 0
            gyro = sp * (1 - math.exp(-max(0, t - 3.0) * 8)) \
                if t >= 3.0 else 0
            if t >= 4.0:
                gyro = 500 * math.exp(-(t - 4.0) * 8)
            gyro += random.gauss(0, 25)       # 叠加噪声（模拟真实陀螺仪）
            writer.writerow([
                t_us,
                round(gyro, 1),               # gyroADC[0] 横滚
                round(random.gauss(0, 15), 1),
                round(random.gauss(0, 15), 1),
                sp, 0, 0,                     # setpoint
                round(1400 + sp * 0.4 + random.gauss(0, 30)),   # motor 1
                round(1400 + sp * 0.4 + random.gauss(0, 30)),   # motor 2
                round(1400 - sp * 0.4 + random.gauss(0, 30)),   # motor 3
                round(1400 - sp * 0.4 + random.gauss(0, 30)),   # motor 4
                1680 - int(t * 2),            # 电压缓降
            ])
    return path


def analyze_blackbox_stats(time_axis: list, data: dict, columns: list) -> dict:
    """
    黑匣子结构化分析（v0.6）：把日志算成一组有调参意义的指标，
    供「AI 调参建议」和「AI 解读图表」使用。
    返回指标字典：时长/采样率、各轴陀螺仪 RMS 噪声与 FFT 主峰、
    设定值→陀螺仪跟踪滞后（互相关）、电机饱和占比、最低电压。
    """
    import numpy as np

    stats = {}
    if not time_axis:
        return stats
    stats["时长(秒)"] = round(time_axis[-1], 1)
    t = np.array(time_axis)
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0
    stats["采样率(Hz)"] = round(1 / dt) if dt > 0 else 0

    def arr(name):
        return np.array(data.get(name, []), dtype=float)

    axis_names = {0: "横滚", 1: "俯仰", 2: "偏航"}
    # ---- 各轴陀螺仪：RMS 噪声 + 前 3 个频谱峰 ----
    for axis in range(3):
        name = f"gyroADC[{axis}]"
        if name not in data or dt <= 0:
            continue
        y = arr(name)
        y = y[~np.isnan(y)]
        if len(y) < 64:
            continue
        y = y - y.mean()
        rms = float(np.sqrt(np.mean(y * y)))
        windowed = y * np.hanning(len(y))
        spec = np.abs(np.fft.rfft(windowed)) / len(y) * 2
        freqs = np.fft.rfftfreq(len(y), dt)
        mask = freqs > 20                       # 忽略机身运动频率
        peaks = []
        if mask.any():
            vf, vs = freqs[mask], spec[mask]
            top = np.argsort(vs)[-3:]
            peaks = sorted(round(float(vf[i])) for i in top)
        stats[f"陀螺仪·{axis_names[axis]} RMS噪声(°/s)"] = round(rms, 1)
        stats[f"陀螺仪·{axis_names[axis]} 噪声峰(Hz)"] = peaks

    # ---- 跟踪质量：setpoint → gyro 的互相关滞后（正 = 陀螺仪慢）----
    for axis in range(3):
        sp, gy = f"setpoint[{axis}]", f"gyroADC[{axis}]"
        if sp not in data or gy not in data or dt <= 0:
            continue
        s, g = arr(sp), arr(gy)
        n = min(len(s), len(g))
        s = np.nan_to_num(s[:n] - np.nanmean(s[:n]))
        g = np.nan_to_num(g[:n] - np.nanmean(g[:n]))
        if np.std(s) < 1e-6 or np.std(g) < 1e-6:
            continue
        corr = np.correlate(g, s, mode="full")
        lag = int(np.argmax(corr) - (n - 1))
        stats[f"跟踪滞后·{axis_names[axis]}(ms)"] = round(lag * dt * 1000, 1)

    # ---- 电机饱和（输出 >1950 的时间占比，取最高的一只电机）----
    motors = [c for c in columns if c.startswith("motor[")]
    sat = []
    for mn in motors:
        mv = arr(mn)
        mv = mv[~np.isnan(mv)]
        if len(mv):
            sat.append(float(np.mean(mv > 1950) * 100))
    if sat:
        stats["电机饱和时间占比(%)"] = round(max(sat), 1)

    # ---- 电压（vbatLatest 单位 0.01V）----
    if "vbatLatest" in data:
        v = arr("vbatLatest") / 100.0
        v = v[~np.isnan(v)]
        if len(v):
            stats["最低电压(V)"] = round(float(v.min()), 2)
    return stats


def classify_log_type(time_axis: list, data: dict, columns: list) -> dict:
    """
    判别日志类型（v0.7）：真实飞行 vs 地面通电空转（未装桨）。
    用确定性特征打分，结论连同证据一起给界面显示和 AI 解读：
      ① 电流：未装桨空转时负载极小（平均通常 < 3A），真实飞行大得多
      ② 陀螺仪 RMS：未装桨没有桨叶气动载荷，噪声很小
      ③ 姿态设定活动度：真实飞行三轴 setpoint 活动明显
      ④ 电机是否在转：区分"空转"与"静止"
    返回：{"verdict": 结论, "confidence": 置信度%, "reasons": [证据],
           "features": {特征值}}
    """
    import numpy as np

    result = {"verdict": "数据不足", "confidence": 0,
              "reasons": [], "features": {}}
    if not time_axis:
        return result

    def arr(name):
        return np.array(data.get(name, []), dtype=float)

    feats = {}
    motors = [c for c in columns if c.startswith("motor[")]
    if motors:
        feats["电机平均输出"] = round(float(np.nanmean(
            [np.nanmean(arr(c)) for c in motors])))
    if "amperageLatest" in data:
        amps = arr("amperageLatest") / 100.0
        feats["平均电流(A)"] = round(float(np.nanmean(amps)), 2)
        feats["峰值电流(A)"] = round(float(np.nanmax(amps)), 2)
    gyro_cols = [c for c in columns if c.startswith("gyroADC[")]
    rms_all = []
    for c in gyro_cols:
        y = arr(c)
        y = y[~np.isnan(y)]
        if len(y):
            rms_all.append(float(np.sqrt(np.mean((y - y.mean()) ** 2))))
    if rms_all:
        feats["陀螺仪平均RMS(°/s)"] = round(float(np.mean(rms_all)), 1)
    sp_cols = [c for c in columns if c.startswith("setpoint[")][:3]
    if sp_cols:
        feats["姿态设定活动度"] = round(float(np.mean(
            [np.nanstd(arr(c)) for c in sp_cols])), 1)
    result["features"] = feats

    score, reasons = 0, []
    amps = feats.get("平均电流(A)")
    if amps is not None:
        if amps < 3:
            score += 2
            reasons.append(f"平均电流仅 {amps}A（装桨飞行通常远超 3A）")
        elif amps > 6:
            score -= 2
            reasons.append(f"平均电流 {amps}A，有明显动力负载")
    rms = feats.get("陀螺仪平均RMS(°/s)")
    if rms is not None:
        if rms < 25:
            score += 1
            reasons.append(f"陀螺仪噪声 RMS 仅 {rms}°/s，"
                           "几乎没有桨叶气动载荷")
        elif rms > 60:
            score -= 1
            reasons.append(f"陀螺仪 RMS {rms}°/s，有明显飞行振动")
    act = feats.get("姿态设定活动度")
    if act is not None:
        if act < 30:
            score += 1
            reasons.append("三轴姿态设定几乎没动（不像在做飞行动作）")
        else:
            score -= 1
            reasons.append("有明显的姿态控制活动")
    motor_avg = feats.get("电机平均输出", 0)
    spinning = motor_avg > 1100
    reasons.append(f"电机平均输出 {motor_avg}"
                   + ("，电机在转" if spinning else "，电机基本停转"))

    if score >= 2:
        verdict = ("疑似地面空转（通电未装桨）" if spinning
                   else "地面静止数据（电机未转）")
        confidence = min(95, 60 + score * 10)
    elif score <= -2:
        verdict = "正常飞行数据"
        confidence = min(95, 60 - score * 10)
    else:
        verdict = "无法确定（特征不明显）"
        confidence = 50
    result.update(verdict=verdict, confidence=confidence, reasons=reasons)
    return result

