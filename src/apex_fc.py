# -*- coding: utf-8 -*-
"""ApexFlight - 飞控数据查询与写入、Rates/滤波 read-modify-write、
调参快照与 PID 备份恢复。"""

import json
import time
from datetime import datetime
from pathlib import Path

import serial

from apex_msp import *  # noqa: F401,F403

# 项目根目录（src 的上一级）与备份文件夹。
# 打包适配（v0.9）：PyInstaller 冻结后，用户数据（备份/日志/预设/配置）
# 放在 exe 同级目录；程序资源（图标/解码器）在 PyInstaller 解压的临时目录。
import sys as _sys
if getattr(_sys, "frozen", False):
    PROJECT_ROOT = Path(_sys.executable).resolve().parent
    _BUNDLE_ROOT = Path(getattr(_sys, "_MEIPASS", PROJECT_ROOT))
else:
    PROJECT_ROOT = Path(__file__).resolve().parent.parent
    _BUNDLE_ROOT = PROJECT_ROOT
BACKUP_DIR = PROJECT_ROOT / "backups"
ICON_PATH = _BUNDLE_ROOT / "assets" / "icon.png"
# QQ 交流群二维码（欢迎页展示）
QQ_QR_PATH = _BUNDLE_ROOT / "assets" / "qq_group_qr.png"
# 官方黑匣子解码器（cleanflight/blackbox-tools，可把 .bbl/.bfl 转成 CSV）
BLACKBOX_DECODER = _BUNDLE_ROOT / "tools" / "blackbox_decode.exe"
# 演示日志存放目录
LOGS_DIR = PROJECT_ROOT / "logs"


# ============================================================
# 第二部分：飞控数据查询与写入
# ============================================================

def query_flight_controller(ser: serial.Serial) -> dict:
    """查询飞控基本信息：固件版本、板子型号、机型/电机。
    额外附带结构化字段：variant（固件代号如 BTFL）、version_tuple（版本元组），
    供 compatibility_report 做适配判断。"""
    info = {"firmware": "未知", "board": "未知", "motors": "未知",
            "variant": "", "version_tuple": None}

    # 固件版本（3 字节：主.次.修订）
    try:
        data = msp_request(ser, MSP_FC_VERSION)
        if len(data) >= 3:
            info["version_tuple"] = (data[0], data[1], data[2])
            info["firmware"] = f"Betaflight {data[0]}.{data[1]}.{data[2]}"
    except MspError:
        pass

    # 固件名称确认（应为 "BTFL"）
    try:
        data = msp_request(ser, MSP_FC_VARIANT)
        variant = data[:4].decode("ascii", errors="replace").strip("\x00 ")
        info["variant"] = variant
        if variant != "BTFL":
            info["firmware"] += f"（注意：检测到固件为 {variant}）"
    except MspError:
        pass

    # 飞控板型号（MSP_BOARD_INFO，Betaflight 4.x 格式）
    #   前 4 字节 = 短代号（如 SH74）；第 8 字节起 = 两个长度前缀字符串：
    #   MCU 目标名（如 STM32H743）和板子完整名（如 DAKEFPVH743）
    try:
        data = msp_request(ser, MSP_BOARD_INFO)
        if len(data) >= 4:
            short_id = data[:4].decode("ascii", errors="replace")
            mcu_name, board_name = "", ""
            if len(data) >= 9:
                n1 = data[8]
                if 9 + n1 <= len(data):
                    mcu_name = data[9:9 + n1].decode("ascii", errors="replace")
                    pos = 9 + n1
                    if pos + 1 <= len(data):
                        n2 = data[pos]
                        if n2 > 0 and pos + 1 + n2 <= len(data):
                            board_name = data[pos + 1:pos + 1 + n2].decode(
                                "ascii", errors="replace")
            if board_name:
                info["board"] = f"{board_name}（{mcu_name}，代号 {short_id}）"
            elif mcu_name:
                info["board"] = f"{mcu_name}（板子代号 {short_id}）"
            else:
                info["board"] = short_id
    except MspError:
        pass

    # 机型/电机：先尝试老命令 MSP_IDENT，失败则用 MSP_MOTOR 数电机通道
    try:
        data = msp_request(ser, MSP_IDENT)
        if len(data) >= 2:
            info["motors"] = MULTITYPE_NAMES.get(data[1], f"机型代码 {data[1]}")
    except MspError:
        try:
            data = msp_request(ser, MSP_MOTOR)
            info["motors"] = f"{len(data) // 2} 个电机通道"
        except MspError:
            pass
    return info


# ------------------------------------------------------------
# 兼容性评估（v0.8）：不同固件 / 不同版本的适配策略
# ------------------------------------------------------------
# 已知固件代号（MSP_FC_VARIANT 返回的 4 字符）
KNOWN_VARIANTS = {
    "BTFL": "Betaflight",
    "INAV": "INAV",
    "CLFL": "Cleanflight",
    "QUIC": "QUIC",
    "EMUF": "EmuFlight",
    "RHFL": "Rotorflight",
}


def parse_fc_version(text: str) -> tuple | None:
    """从 'Betaflight 4.5.2' 之类的字符串解析版本元组 (4, 5, 2)，失败返回 None"""
    import re
    m = re.search(r"(\d+)\.(\d+)(?:\.(\d+))?", text)
    if not m:
        return None
    return (int(m.group(1)), int(m.group(2)),
            int(m.group(3)) if m.group(3) else 0)


def compatibility_report(info: dict) -> dict:
    """根据固件变体和版本评估本软件与这块飞控的兼容程度。

    返回 {
        level: "full"（完全支持）/ "limited"（受限）/ "unknown"（未知）,
        block_writes: 是否锁定 BF 专属布局的写入（Rates/滤波/预设应用）,
        messages: 给用户看的中文提示列表,
        features: 各功能可用性,
    }

    适配原则：
    - 非 Betaflight 固件（INAV/Cleanflight/QUIC 等）：MSP 核心命令通用，
      PID/电机/仪表可用；但 RC_TUNING 23 字节、FILTER_CONFIG 49 字节是
      Betaflight 私有布局，写入可能破坏配置 → 只读保护。
    - Betaflight 4.4+：布局已逐字段验证，完全支持。
    - Betaflight 4.2/4.3：布局接近但未完整验证，允许写入但给出提醒。
    - Betaflight < 4.2：布局不同，只读保护。
    """
    variant = info.get("variant", "") or parse_fc_variant_from_text(
        info.get("firmware", ""))
    version = info.get("version_tuple") or parse_fc_version(
        info.get("firmware", ""))
    features = {"pid": True, "rates": True, "filter": True,
                "blackbox": True, "motors": True, "presets": True}

    if variant and variant != "BTFL":
        name = KNOWN_VARIANTS.get(variant, variant)
        features.update({"rates": False, "filter": False, "presets": False})
        return {
            "level": "limited", "block_writes": True,
            "messages": [
                f"检测到 {name} 固件：本软件按 Betaflight 协议开发，"
                "仪表盘、PID、电机测试可用；",
                "Rates 与滤波器的字节布局是该固件私有格式，"
                "已切换为只读保护（可查看，保存按钮已锁定）。",
            ],
            "features": features,
        }

    if version is None:
        return {
            "level": "unknown", "block_writes": False,
            "messages": ["未能识别固件版本：功能将照常尝试，"
                         "执行写入操作前建议先做备份。"],
            "features": features,
        }

    major, minor = version[0], version[1]
    if (major, minor) >= (4, 4):
        return {"level": "full", "block_writes": False,
                "messages": [], "features": features}
    if (major, minor) >= (4, 2):
        return {
            "level": "limited", "block_writes": False,
            "messages": [f"Betaflight {major}.{minor} 未经完整验证"
                         "（本软件按 4.4+ 协议逐字段核对），"
                         "如写入后手感异常请用自动备份恢复。"],
            "features": features,
        }
    features.update({"rates": False, "filter": False, "presets": False})
    return {
        "level": "limited", "block_writes": True,
        "messages": [f"Betaflight {major}.{minor} 版本较旧：Rates/滤波器布局"
                     "与 4.4+ 不同，已切换为只读保护。"],
        "features": features,
    }


def parse_fc_variant_from_text(text: str) -> str:
    """从显示文本（如 'Betaflight 4.5.2（注意：检测到固件为 INAV）'）提取固件代号"""
    for code in KNOWN_VARIANTS:
        if code in text:
            return code
    return ""


def query_pid(ser: serial.Serial) -> tuple:
    """
    读取全部 PID 参数（MSP_PID）。
    每组 3 字节（P、I、D）。新固件返回 5 组（15 字节），老固件 10 组（30 字节）。
    返回：(名称列表, 数值列表)，数值列表元素为 (P, I, D) 元组。
    """
    data = msp_request(ser, MSP_PID)
    if len(data) < 9 or len(data) % 3 != 0:
        raise MspError(f"PID 数据长度异常：实际 {len(data)} 字节")

    count = len(data) // 3
    if count == 5:
        names = PID_NAMES_MODERN
    elif count == 10:
        names = PID_NAMES_LEGACY
    else:                                     # 未知固件：用通用名称
        names = [f"PID {i + 1}" for i in range(count)]

    values = []
    for i in range(count):
        values.append((data[i * 3], data[i * 3 + 1], data[i * 3 + 2]))
    return names, values


def write_pid(ser: serial.Serial, values: list, save_eeprom: bool = True):
    """
    把 PID 参数写回飞控（MSP_SET_PID），然后保存到闪存（MSP_EEPROM_WRITE）。
    参数：values = [(P, I, D), ...]，与读取时的组数一致
    """
    payload = bytes(v for triple in values for v in triple)
    msp_request(ser, MSP_SET_PID, payload)
    if save_eeprom:
        # 保存到闪存耗时较长（飞控要写 Flash），超时放宽到 5 秒
        msp_request(ser, MSP_EEPROM_WRITE, b"", timeout=5.0)


# ------------------------------------------------------------
# Rates / 摇杆调参（MSP_RC_TUNING / MSP_SET_RC_TUNING）
# ------------------------------------------------------------
# Betaflight 4.5 的 MSP_RC_TUNING 响应布局（23 字节，与固件 msp.c 一致）：
#   0  rcRates[横滚]      1  rcExpo[横滚]      2~4 rates[横滚/俯仰/偏航]
#   5  (旧 tpa_rate，已废弃) 6 油门中点  7 油门 expo  8~9 (旧 tpa 断点 u16)
#   10 rcExpo[偏航]  11 rcRates[偏航]  12 rcRates[俯仰]  13 rcExpo[俯仰]
#   14 油门限幅类型  15 油门限幅百分比
#   16~21 三轴角速度上限（u16 × 3）  22 Rates 类型（0=Betaflight 经典）
# 所有比例值存储为 百分数整数（150 = 1.50）。
# 写入策略：先完整读取 23 字节，只修改我们理解的字节，原样写回其余字节
# （read-modify-write），未知/废弃字段保持不变，兼容性最好。

def query_rc_tuning(ser: serial.Serial) -> bytes:
    """读取 Rates 调参原始数据（MSP_RC_TUNING），返回原始字节串"""
    return msp_request(ser, MSP_RC_TUNING)


def write_rc_tuning(ser: serial.Serial, raw: bytes, save_eeprom: bool = True):
    """把修改后的 23 字节 Rates 数据写回飞控"""
    msp_request(ser, MSP_SET_RC_TUNING, bytes(raw))
    if save_eeprom:
        msp_request(ser, MSP_EEPROM_WRITE, b"", timeout=5.0)


# 三个可调字段 × 三轴（横滚/俯仰/偏航）在 23 字节中的偏移
RC_FIELD_OFFSETS = {
    "rc_rate": [0, 12, 11],     # 中位灵敏度（RC Rate）
    "expo":    [1, 13, 10],     # 中位指数（Expo）
    "rate":    [2, 3, 4],       # 满杆速率（Super Rate）
}


def parse_rc_tuning(raw: bytes) -> dict:
    """把 RC_TUNING 原始数据解析成可读字典（比例值已除以 100）。

    适配策略（v0.8）：Betaflight 4.4+ 返回 23 字节；更老/其他固件可能更短。
    短数据不再直接报错，而是按可得字节解析、缺失字段填安全默认值，
    并标记 "partial": True 让界面提示"该固件数据不完整，仅供参考"。"""
    if len(raw) < 8:
        raise MspError(f"Rates 数据长度异常：实际 {len(raw)} 字节（至少需 8）")
    partial = len(raw) < 23

    def b(i: int, default: int = 0) -> int:
        return raw[i] if i < len(raw) else default

    def w(i: int, default: int = 0) -> int:
        return u16(raw, i) if i + 2 <= len(raw) else default

    result = {
        "rc_rate": [b(0, 100) / 100, b(12, 100) / 100, b(11, 100) / 100],
        "expo":    [b(1) / 100, b(13) / 100, b(10) / 100],
        "rate":    [b(2, 70) / 100, b(3, 70) / 100, b(4, 70) / 100],
        "thr_mid": b(6, 50) / 100,
        "thr_expo": b(7) / 100,
        "thr_limit_pct": b(15, 100),
        "rate_limit": [w(16, 1998), w(18, 1998), w(20, 1998)],
        "rates_type": b(22, 0),
    }
    if partial:
        result["partial"] = True
    return result


def set_rc_value(raw: bytearray, field: str, axis: int, value: float):
    """修改某轴某字段（value 为浮点比例值，如 1.50），写回 bytearray"""
    raw[RC_FIELD_OFFSETS[field][axis]] = max(0, min(255, round(value * 100)))


def bf_rate_curve(stick: float, rc_rate: float, super_rate: float,
                  expo: float) -> float:
    """
    Betaflight 经典 Rates 公式（与固件 fc/rc.c applyBetaflightRates 一致）：
    摇杆偏转 0~1 → 角速度（°/s）。
    注意：expo 只弯曲线性部分；满杆拉升系数用的是 expo 之前的原始杆量；
    rc_rate 超过 2.0 时固件还有额外的线性增益（14.54 倍斜率）。
    """
    xe = stick * (1 - expo) + stick ** 3 * expo
    if rc_rate > 2.0:
        rc_rate += 14.54 * (rc_rate - 2.0)
    angle = 200 * rc_rate * xe
    if super_rate:
        angle /= max(0.01, 1 - abs(stick) * super_rate)
    return angle


def bf_throttle_curve(x: float, mid: float, expo: float) -> float:
    """
    Betaflight 油门曲线（与固件 fc/rc.c lookupThrottleRC 一致）：
    输入油门 0~1 → 输出 0~1，曲线锚定中点 (mid, mid)，expo 控制弯曲程度。
    """
    if mid <= 0 or mid >= 1:
        mid = 0.5
    scale = (1 - mid) if x > mid else mid
    if scale <= 0:
        return mid
    t = (x - mid) / scale
    return mid + t * (1 - expo + expo * t * t) * scale


# ------------------------------------------------------------
# 滤波器配置（MSP_FILTER_CONFIG / MSP_SET_FILTER_CONFIG）
# ------------------------------------------------------------
# Betaflight 4.5 的 MSP_FILTER_CONFIG 响应布局（49 字节，与固件 msp.c 一致）。
# 同样采用 read-modify-write：只改下表列出的字节，其余原样写回。
#   字段定义：（键名, 中文显示名, 字节偏移, 类型, 最小值, 最大值）
FILTER_FIELDS = [
    ("gyro_lpf1_hz",  "陀螺仪低通 1 截止频率 (Hz)", 20, "u16", 0, 1000),
    ("gyro_lpf2_hz",  "陀螺仪低通 2 截止频率 (Hz)", 22, "u16", 0, 1000),
    ("dterm_lpf1_hz", "D 项低通 1 截止频率 (Hz)",   1,  "u16", 0, 500),
    ("dterm_lpf2_hz", "D 项低通 2 截止频率 (Hz)",   26, "u16", 0, 500),
    ("yaw_lpf_hz",    "偏航低通截止频率 (Hz)",       3,  "u16", 0, 500),
    ("gyro_notch1_hz",     "陀螺仪陷波 1 频率 (Hz)",  5,  "u16", 0, 1000),
    ("gyro_notch1_cutoff", "陀螺仪陷波 1 截止 (Hz)",  7,  "u16", 0, 1000),
    ("dterm_notch_hz",     "D 项陷波频率 (Hz)",       9,  "u16", 0, 500),
    ("dterm_notch_cutoff", "D 项陷波截止 (Hz)",       11, "u16", 0, 500),
    ("gyro_notch2_hz",     "陀螺仪陷波 2 频率 (Hz)",  13, "u16", 0, 1000),
    ("gyro_notch2_cutoff", "陀螺仪陷波 2 截止 (Hz)",  15, "u16", 0, 1000),
    ("gyro_dyn_min",  "陀螺仪动态低通·下限 (Hz)",   29, "u16", 0, 1000),
    ("gyro_dyn_max",  "陀螺仪动态低通·上限 (Hz)",   31, "u16", 0, 1000),
    ("dterm_dyn_min", "D 项动态低通·下限 (Hz)",     33, "u16", 0, 500),
    ("dterm_dyn_max", "D 项动态低通·上限 (Hz)",     35, "u16", 0, 500),
    ("notch_q",       "动态陷波 Q 值",              39, "u16", 1, 1000),
    ("notch_min",     "动态陷波·最低频率 (Hz)",     41, "u16", 0, 1000),
    ("notch_max",     "动态陷波·最高频率 (Hz)",     45, "u16", 0, 1000),
    ("notch_count",   "动态陷波·数量",              48, "u8",  0, 5),
    ("dyn_expo",      "D 项动态低通 expo",          47, "u8",  0, 10),
    ("rpm_harmonics", "RPM 滤波·谐波数量",          43, "u8",  0, 3),
    ("rpm_min_hz",    "RPM 滤波·最低频率 (Hz)",     44, "u8",  0, 255),
]

# 滤波器类型字段（下拉选择）：键名 → 字节偏移
FILTER_TYPE_FIELDS = {
    "gyro_lpf1_type": 24, "gyro_lpf2_type": 25,
    "dterm_lpf1_type": 17, "dterm_lpf2_type": 28,
}
FILTER_TYPES = ["PT1", "Biquad", "PT2", "PT3"]


def query_filter_config(ser: serial.Serial) -> bytes:
    """读取滤波器配置原始数据（MSP_FILTER_CONFIG）"""
    return msp_request(ser, MSP_FILTER_CONFIG)


def write_filter_config(ser: serial.Serial, raw: bytes,
                        save_eeprom: bool = True):
    """把修改后的滤波器配置写回飞控"""
    msp_request(ser, MSP_SET_FILTER_CONFIG, bytes(raw))
    if save_eeprom:
        msp_request(ser, MSP_EEPROM_WRITE, b"", timeout=5.0)


def parse_filter_config(raw: bytes) -> dict:
    """把滤波器原始数据解析成 {键名: 整数值}。

    适配策略（v0.8）：Betaflight 4.4+ 返回 49 字节；短数据容错解析——
    超出长度的字段填 0 并标记 "partial": True，不再整体报错。"""
    if len(raw) < 8:
        raise MspError(f"滤波器数据长度异常：实际 {len(raw)} 字节（至少需 8）")
    result = {}
    for key, _name, offset, kind, _lo, _hi in FILTER_FIELDS:
        if kind == "u16":
            result[key] = u16(raw, offset) if offset + 2 <= len(raw) else 0
        else:
            result[key] = raw[offset] if offset < len(raw) else 0
    if len(raw) < 49:
        result["partial"] = True
    return result


def set_filter_value(raw: bytearray, key: str, value: int):
    """修改某个滤波器字段，写回 bytearray"""
    for k, _name, offset, kind, lo, hi in FILTER_FIELDS:
        if k == key:
            value = max(lo, min(hi, int(value)))
            if kind == "u16":
                raw[offset] = value & 0xFF
                raw[offset + 1] = (value >> 8) & 0xFF
            else:
                raw[offset] = value & 0xFF
            if key == "gyro_lpf1_hz":
                # 偏移 0 处还有一个旧版单字节副本，保持同步
                raw[0] = value & 0xFF
            return
    raise KeyError(f"未知滤波器字段：{key}")


# ------------------------------------------------------------
# 调参方案（预设）：PID + Rates + 滤波器 整体快照
# ------------------------------------------------------------
PRESETS_DIR = PROJECT_ROOT / "presets"


def tuning_snapshot(info: dict, pid_names: list, pid_values: list,
                    rc_raw: bytes, filter_raw: bytes,
                    name: str = "") -> dict:
    """把当前全部调参状态打包成一个字典（用于保存预设或写入前备份）"""
    return {
        "software": "ApexFlight",
        "version": "0.5",
        "name": name,
        "saved_time": datetime.now().isoformat(timespec="seconds"),
        "firmware": info.get("firmware", "未知"),
        "board": info.get("board", "未知"),
        "pid_names": pid_names,
        "pid_values": [list(v) for v in pid_values],
        "rc_tuning_raw": list(rc_raw),
        "filter_raw": list(filter_raw),
    }


def save_preset_file(directory: Path, filename: str, snapshot: dict) -> Path:
    """把调参快照保存为 JSON 文件，返回路径"""
    directory.mkdir(exist_ok=True)
    path = directory / filename
    path.write_text(json.dumps(snapshot, ensure_ascii=False, indent=2),
                    encoding="utf-8")
    return path


def load_preset_file(path: Path) -> dict:
    """读取预设 JSON 文件"""
    return json.loads(path.read_text(encoding="utf-8"))


def query_status_ex(ser: serial.Serial) -> dict:
    """
    查询扩展状态（MSP_STATUS_EX）：循环时间、CPU 负载、解锁禁用标志。
    数据格式：循环时间(2) I2C错误(2) 传感器(2) 飞行模式(4) 当前配置(1)
              CPU负载(2) 解锁禁用标志数量(1) 解锁禁用标志(4) 配置状态(1)
    """
    data = msp_request(ser, MSP_STATUS_EX)
    result = {"cycle_us": 0, "cpu_load": 0, "arming_disabled": []}
    if len(data) >= 11:
        result["cycle_us"] = u16(data, 0)
        result["cpu_load"] = u16(data, 11) if len(data) >= 13 else 0
        # 解锁禁用标志是 4 字节位图，逐个位翻译成中文原因
        if len(data) >= 18:
            flags = (data[14] | (data[15] << 8)
                     | (data[16] << 16) | (data[17] << 24))
            for bit, reason in ARMING_DISABLE_FLAGS.items():
                if flags & (1 << bit):
                    result["arming_disabled"].append(reason)
    return result


def query_analog(ser: serial.Serial) -> dict:
    """
    查询模拟量（MSP_ANALOG）：电压、耗电、RSSI、电流。
    数据格式：电压(1, 0.1V) 耗电mAh(2) RSSI(2, 0-1023) 电流(2, 0.01A) 电压(2, 0.01V)
    """
    data = msp_request(ser, MSP_ANALOG)
    result = {"voltage": 0.0, "mah": 0, "rssi": 0, "amps": 0.0}
    if len(data) >= 1:
        result["voltage"] = data[0] / 10.0    # 老格式，精度低
    if len(data) >= 3:
        result["mah"] = u16(data, 1)
    if len(data) >= 5:
        result["rssi"] = round(u16(data, 3) / 1023 * 100)  # 转成百分比
    if len(data) >= 7:
        result["amps"] = u16(data, 5) / 100.0
    if len(data) >= 9:
        result["voltage"] = u16(data, 7) / 100.0  # 新格式，精度高
    return result


def query_attitude(ser: serial.Serial) -> tuple:
    """
    查询姿态角（MSP_ATTITUDE）。
    数据格式：横滚(2, 0.1度) 俯仰(2, 0.1度) 偏航(2, 1度)
    返回：(roll_deg, pitch_deg, yaw_deg)
    """
    data = msp_request(ser, MSP_ATTITUDE)
    if len(data) < 6:
        raise MspError("姿态数据长度异常")
    return s16(data, 0) / 10.0, s16(data, 2) / 10.0, s16(data, 4)


def query_rc(ser: serial.Serial) -> list:
    """查询接收机通道值（MSP_RC），每个通道 2 字节，通常 1000~2000。"""
    data = msp_request(ser, MSP_RC)
    return [u16(data, i) for i in range(0, len(data) - 1, 2)]


def query_motor_count(ser: serial.Serial) -> int:
    """查询电机输出通道数量（MSP_MOTOR，每个电机 2 字节）。"""
    data = msp_request(ser, MSP_MOTOR)
    return len(data) // 2


def set_motors(ser: serial.Serial, values: list):
    """
    直接设置电机输出（MSP_SET_MOTOR），电机测试用。
    参数：values = 8 个 0~2000 的整数（0 表示停转）
    ⚠️ 危险操作：调用前必须确认已拆下螺旋桨！
    """
    payload = bytearray()
    for v in values:
        payload += int(v).to_bytes(2, "little")
    msp_request(ser, MSP_SET_MOTOR, bytes(payload))


def query_dataflash_summary(ser: serial.Serial) -> dict:
    """
    查询板载闪存信息（MSP_DATAFLASH_SUMMARY，命令码 70）。
    数据格式：是否支持(1) 扇区数(4) 总容量(4) 已用空间(4)
    返回：{"supported": bool, "total_mb": float, "used_bytes": int}
    """
    data = msp_request(ser, MSP_DATAFLASH_SUMMARY)
    if len(data) < 13:
        raise MspError("闪存信息数据长度异常")
    supported = data[0] != 0
    used = int.from_bytes(data[9:13], "little")
    total = int.from_bytes(data[5:9], "little")
    return {"supported": supported,
            "total_mb": total / 1048576,
            "used_bytes": used}


def download_dataflash(ser: serial.Serial, used_bytes: int,
                       start_address: int = 0,
                       progress_cb=None, cancel_flag=None) -> bytes:
    """
    从板载闪存下载黑匣子日志原始数据（MSP_DATAFLASH_READ，命令码 71）。
    请求格式：起始地址(4字节) + 读取长度(2字节)。
    响应格式：地址回显(4字节) + 数据。

    参数：used_bytes = 已用字节数（从 summary 获得，即下载终点）
          start_address = 起始地址（>0 时只下载尾部 = 最新的飞行记录，
                          因为 Betaflight 的闪存日志是顺序追加写的）
          progress_cb = 进度回调（已下载字节数）
          cancel_flag = 可取消标志（带 is_set() 方法的对象，如 threading.Event）
    返回：下载区间的字节串（.bbl 文件内容，可能从某段日志中间开始，
          解码器会自动跳过不完整的开头）
    """
    if start_address >= used_bytes:
        raise MspError("下载区间为空")
    # 首次读取：探测协议版本（v2 大帧 / v1 小帧）与响应头格式
    probe_len = min(4096, used_bytes - start_address)
    probe_payload = start_address.to_bytes(4, "little") + probe_len.to_bytes(2, "little")
    hdr = 4                       # 响应头长度（数据起始偏移），默认老格式
    try:
        probe = msp2_request(ser, MSP_DATAFLASH_READ, probe_payload)
        use_v2, chunk = True, 4096
    except MspError:
        probe_len = min(240, used_bytes - start_address)
        probe_payload = start_address.to_bytes(4, "little") + probe_len.to_bytes(2, "little")
        probe = msp_request(ser, MSP_DATAFLASH_READ, probe_payload)
        use_v2, chunk = False, 240
    if len(probe) >= 7 + probe_len and probe[4:6] == probe_len.to_bytes(2, "little"):
        if probe[6] != 0:
            raise MspError("飞控返回了压缩格式的闪存数据，暂不支持解析")
        hdr = 7                   # BF 4.x 新格式：4+2+1 字节响应头
    buffer = bytearray(probe[hdr:hdr + probe_len])

    request = msp2_request if use_v2 else msp_request
    address = start_address + len(buffer)
    while address < used_bytes:
        if cancel_flag is not None and cancel_flag.is_set():
            raise MspError("下载已取消")
        length = min(chunk, used_bytes - address)
        payload = address.to_bytes(4, "little") + length.to_bytes(2, "little")
        data = request(ser, MSP_DATAFLASH_READ, payload)
        if len(data) < hdr + length:
            raise MspError("闪存读取响应长度异常")
        echo = int.from_bytes(data[:4], "little")
        if echo != address:
            raise MspError(f"闪存地址不匹配：期望 {address}，实际 {echo}")
        buffer += data[hdr:hdr + length]
        address += length
        if progress_cb and address % 65536 < chunk:   # 每 64KB 报一次进度
            progress_cb(address - start_address)
    return bytes(buffer[:used_bytes - start_address])


def find_last_log_start(ser: serial.Serial, used_bytes: int,
                        progress_cb=None, cancel_flag=None) -> int:
    """
    从闪存末尾向前搜索最近一次飞行日志的起点地址。
    黑匣子每段日志以 "H Product:Blackbox" 文本头开头。尾部下载时如果
    区间内没有段头（说明最近一次飞行很长，跨越了区间边界），解码器
    将无法识别数据，需要把下载起点扩展到该段头位置。
    返回：最后一个段头的绝对地址；整片闪存都没有段头时返回 0。
    """
    HEADER = b"H Product"
    STEP = 512 * 1024            # 每块 512KB（读取约 11 秒）
    OVERLAP = 64                 # 块间重叠，防止段头正好落在块边界上
    pos = used_bytes
    while pos > 0:
        if cancel_flag is not None and cancel_flag.is_set():
            raise MspError("下载已取消")
        c0 = max(0, pos - STEP)
        c1 = min(used_bytes, pos + OVERLAP)
        if progress_cb:
            progress_cb(-c0)              # 负数 = 正在扫描定位阶段
        # MSP 长度字段是 2 字节，单帧最多 65535，分小帧读取本块
        data = bytearray()
        addr = c0
        while addr < c1:
            length = min(60000, c1 - addr)
            payload = addr.to_bytes(4, "little") + length.to_bytes(2, "little")
            data += msp2_request(ser, MSP_DATAFLASH_READ, payload)[7:]
            addr += length
        idx = bytes(data).rfind(HEADER)   # 块内最后一个段头
        if idx >= 0:
            return c0 + idx
        pos = c0
    return 0


def erase_dataflash(ser: serial.Serial, timeout: float = 180.0):
    """
    清空飞控板载闪存（MSP_DATAFLASH_ERASE，命令码 72）。
    擦除整颗芯片需要几十秒，飞控在后台异步执行；
    本函数轮询 summary 直到已用空间归零（超时抛 MspError）。
    ⚠️ 不可恢复：调用前必须先完成下载！
    """
    msp_request(ser, MSP_DATAFLASH_ERASE)         # 触发擦除
    deadline = time.time() + timeout
    while time.time() < deadline:
        time.sleep(2)
        summary = query_dataflash_summary(ser)    # 轮询擦除进度
        if summary["used_bytes"] == 0:
            return
    raise MspError("等待闪存擦除完成超时")


# ============================================================
# 第三部分：PID 备份 / 恢复
# ============================================================

def save_backup(info: dict, names: list, values: list) -> Path:
    """
    把当前 PID 参数备份成 JSON 文件，存到 backups/ 文件夹。
    文件名带时间戳，方便区分多次备份。返回文件路径。
    """
    BACKUP_DIR.mkdir(exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = {
        "software": "ApexFlight",
        "backup_time": datetime.now().isoformat(timespec="seconds"),
        "firmware": info.get("firmware", "未知"),
        "board": info.get("board", "未知"),
        "pid_names": names,
        "pid_values": [list(v) for v in values],
    }
    path = BACKUP_DIR / f"apex_backup_{timestamp}.json"
    path.write_text(json.dumps(backup, ensure_ascii=False, indent=2),
                    encoding="utf-8")
    return path


def load_backup(path: Path) -> tuple:
    """读取备份 JSON 文件，返回 (名称列表, 数值列表)。"""
    data = json.loads(path.read_text(encoding="utf-8"))
    names = data["pid_names"]
    values = [tuple(v) for v in data["pid_values"]]
    return names, values


