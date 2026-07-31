# -*- coding: utf-8 -*-
"""
ApexFlight —— 开源无人机调参软件
v0.5：实时仪表盘 + PID 在线调参（写入/备份/恢复）+ Rates 调参（曲线可视化）
      + 滤波器设置（与黑匣子频谱联动）+ 调参方案管理（预设保存/一键切换）
      + 电机测试 + 接收机通道监视 + 黑匣子分析（文件/闪存下载/频谱）
      + 本地 AI 助手（Ollama）

运行方式：
    1. 安装依赖：  pip install -r requirements.txt
    2. 运行程序：  python src/main.py   （或双击 启动ApexFlight.bat）

技术栈：
    - 界面：PyQt6
    - 串口：pyserial
    - 协议：MSP v1（MultiWii Serial Protocol）
    - 所有串口操作在后台线程执行，界面不卡死
"""

import csv
import json
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from datetime import datetime
from pathlib import Path

import serial
from serial.tools import list_ports

from PyQt6.QtCore import QObject, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QColor, QFont, QIcon, QPainter, QPainterPath, QPen, QPixmap
from PyQt6.QtWidgets import (
    QAbstractButton, QApplication, QCheckBox, QComboBox, QDoubleSpinBox,
    QFileDialog, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout,
    QHeaderView, QLabel, QListWidget, QListWidgetItem, QMainWindow,
    QMessageBox, QPushButton, QScrollArea, QSlider, QSpinBox, QStackedWidget,
    QTableWidget, QTableWidgetItem, QTextEdit, QLineEdit, QVBoxLayout, QWidget,
)

# matplotlib 用于黑匣子曲线绘制（嵌入式画布）；未安装时黑匣子页给出提示
try:
    import matplotlib
    matplotlib.use("QtAgg")
    # 中文显示配置（微软雅黑），并修复负号显示
    matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
    matplotlib.rcParams["axes.unicode_minus"] = False
    from matplotlib.backends.backend_qtagg import (
        FigureCanvasQTAgg, NavigationToolbar2QT)
    from matplotlib.figure import Figure
    HAS_MPL = True
except ImportError:
    HAS_MPL = False

# ============================================================
# 第一部分：MSP 协议（MultiWii Serial Protocol v1）
# ============================================================
# MSP v1 数据帧格式：
#   请求帧：  '$' 'M' '<'  数据长度(1字节)  命令码(1字节)  [数据...]  校验和(1字节)
#   响应帧：  '$' 'M' '>'  数据长度(1字节)  命令码(1字节)  [数据...]  校验和(1字节)
#   错误帧：  '$' 'M' '!'  ...（飞控拒绝该命令）
#   校验和 = 数据长度、命令码、所有数据字节的异或（XOR）

# 本程序用到的 MSP 命令码
MSP_API_VERSION = 1      # MSP 协议版本
MSP_FC_VARIANT = 2       # 固件名称（如 "BTFL" = Betaflight）
MSP_FC_VERSION = 3       # 固件版本号（3 字节：主.次.修订）
MSP_BOARD_INFO = 4       # 飞控板信息
MSP_IDENT = 100          # 老版识别命令（新固件已废弃，代码做了兼容）
MSP_MOTOR = 104          # 电机输出值（8 个电机，每个 2 字节）
MSP_RC = 105             # 接收机通道值（N 个通道，每个 2 字节）
MSP_ATTITUDE = 108       # 姿态角：横滚/俯仰/偏航
MSP_ANALOG = 110         # 电压、电流、RSSI 等模拟量
MSP_PID = 112            # 读取 PID 参数
MSP_STATUS_EX = 150      # 扩展状态：CPU 负载、解锁禁用标志等
MSP_SET_PID = 202        # 写入 PID 参数
MSP_SET_MOTOR = 214      # 直接控制电机输出（电机测试用）
MSP_EEPROM_WRITE = 250   # 把当前配置保存到飞控闪存（断电不丢失）
MSP_DATAFLASH_SUMMARY = 70   # 板载闪存信息（黑匣子存储芯片）
MSP_DATAFLASH_READ = 71      # 读取板载闪存数据（黑匣子日志原始字节）
MSP_DATAFLASH_ERASE = 72     # 清空板载闪存（擦除整颗芯片，耗时几十秒）
MSP_FILTER_CONFIG = 92       # 读取滤波器配置（低通/陷波/RPM 滤波）
MSP_SET_FILTER_CONFIG = 93   # 写入滤波器配置
MSP_RC_TUNING = 111          # 读取 Rates/Expo/油门曲线等摇杆调参
MSP_SET_RC_TUNING = 204      # 写入摇杆调参

# MSP_IDENT 返回的机型代码对照表（老固件用）
MULTITYPE_NAMES = {
    1: "三轴 (Tri)", 2: "四轴 + (Quad +)", 3: "四轴 X (Quad X)",
    4: "双轴 (Bi)", 5: "云台 (Gimbal)", 6: "Y6", 7: "六轴 + (Hex 6 +)",
    8: "飞翼 (Flying Wing)", 9: "Y4", 10: "六轴 X (Hex 6 X)",
    11: "八轴 X8 (Octo X8)", 12: "八轴扁平 + (Octo Flat +)",
    13: "八轴扁平 X (Octo Flat X)", 14: "飞机 (Airplane)",
    15: "直升机 120 (Heli 120)", 16: "直升机 90 (Heli 90)",
    17: "垂直起降 (VTail)", 18: "四轴 H (Quad H)",
}

# Betaflight 4.4+ 精简后的 5 组 PID 名称；老固件 10 组用后面的旧表
PID_NAMES_MODERN = ["Roll（横滚）", "Pitch（俯仰）", "Yaw（偏航）",
                    "Level（自稳强度）", "Mag（磁航向保持）"]
PID_NAMES_LEGACY = ["Roll（横滚）", "Pitch（俯仰）", "Yaw（偏航）",
                    "Alt（定高）", "Pos（定点）", "PosR（位置速率）",
                    "NavR（导航速率）", "Level（自稳）", "Mag（磁航向）",
                    "Vel（速度）"]

# 解锁禁用标志（arming disable flags）位含义
# 与 Betaflight 4.5 源码 fc/runtime_config.h 中 armingDisableFlags_e 一致
ARMING_DISABLE_FLAGS = {
    0: "NOGYRO（未检测到陀螺仪）",
    1: "FAILSAFE（失控保护激活）",
    2: "RX_FAILSAFE（接收机失控）",
    3: "NOT_DISARMED（接收机恢复时解锁开关未复位）",
    4: "BOXFAILSAFE（失控保护开关打开）",
    5: "RUNAWAY（起飞保护触发）",
    6: "CRASH（摔机检测触发）",
    7: "THROTTLE（油门过高）",
    8: "ANGLE（机身倾斜角度过大）",
    9: "BOOTGRACE（开机保护时间内）",
    10: "NOPREARM（预解锁未开启）",
    11: "LOAD（CPU 负载过高）",
    12: "CALIB（传感器校准中）",
    13: "CLI（命令行模式激活）",
    14: "CMS（OSD 菜单打开）",
    15: "BST（黑羊设备阻止解锁）",
    16: "MSP（调参连接占用中）",
    17: "PARALYZE（瘫痪模式）",
    18: "GPS（GPS 救援卫星不足）",
    19: "RESCUE_SW（GPS 救援开关打开）",
    20: "RPMFILTER（电机 RPM 滤波无数据）",
    21: "REBOOT_REQD（需要重启生效）",
    22: "DSHOT_BBANG（DSHOT 位带模式故障）",
    23: "NO_ACC_CAL（加速度计未校准）",
    24: "MOTOR_PROTO（电调协议未配置）",
    25: "ARMSWITCH（解锁开关位置不安全）",
    26: "DSHOT_TELEM（DSHOT 遥测无数据）",
}

# 项目根目录（src 的上一级）与备份文件夹
PROJECT_ROOT = Path(__file__).resolve().parent.parent
BACKUP_DIR = PROJECT_ROOT / "backups"
ICON_PATH = PROJECT_ROOT / "assets" / "icon.png"
# 官方黑匣子解码器（cleanflight/blackbox-tools，可把 .bbl/.bfl 转成 CSV）
BLACKBOX_DECODER = PROJECT_ROOT / "tools" / "blackbox_decode.exe"
# 演示日志存放目录
LOGS_DIR = PROJECT_ROOT / "logs"


class MspError(Exception):
    """MSP 通信错误（校验失败、飞控拒绝、超时等）"""
    pass


def build_msp_request(cmd: int, payload: bytes = b"") -> bytes:
    """
    构造一个 MSP v1 请求帧。
    参数：cmd = 命令码，payload = 附加数据（查询类命令为空）
    返回：可直接写入串口的完整字节串
    """
    size = len(payload)
    checksum = size ^ cmd                     # 校验和 = 长度 XOR 命令码 XOR 数据
    for byte in payload:
        checksum ^= byte
    return b"$M<" + bytes([size, cmd]) + payload + bytes([checksum])


def read_msp_response(ser: serial.Serial, expected_cmd: int,
                      timeout: float = 2.0) -> bytes:
    """从串口读取并解析一帧 MSP 响应，失败抛出 MspError。"""
    deadline = time.time() + timeout

    def read_n(n: int) -> bytes:
        """读取 n 个字节，超时抛异常"""
        buf = b""
        while len(buf) < n:
            if time.time() > deadline:
                raise MspError("读取超时：飞控无响应（检查串口选择、飞控供电、"
                               "是否有其他程序占用串口）")
            chunk = ser.read(n - len(buf))
            if chunk:
                buf += chunk
        return buf

    # 定位帧头 '$M'
    while True:
        if read_n(1) == b"$" and read_n(1) == b"M":
            break

    direction = read_n(1)
    if direction == b"!":                     # 飞控拒绝此命令
        size = read_n(1)[0]
        cmd = read_n(1)[0]
        read_n(size + 1)
        raise MspError(f"飞控拒绝了命令 {cmd}（该固件可能不支持）")
    if direction != b">":
        raise MspError("收到无法识别的 MSP 数据帧")

    size = read_n(1)[0]
    cmd = read_n(1)[0]
    payload = read_n(size)
    checksum_byte = read_n(1)[0]

    checksum = size ^ cmd                     # XOR 校验
    for byte in payload:
        checksum ^= byte
    if checksum != checksum_byte:
        raise MspError("数据校验失败（XOR 校验和不匹配）")
    if cmd != expected_cmd:
        raise MspError(f"命令码不匹配：期望 {expected_cmd}，实际 {cmd}")
    return payload


# 串口访问锁：所有 MSP 请求共用。
# 快通道（100ms）和慢通道（700ms）轮询运行在不同线程中，
# 如果不加锁，两个线程会同时读写同一个串口，字节流交错导致
# 双方解析到损坏的数据帧，进而超时卡死。
# 每条 MSP 请求是"清空缓冲区 → 发送 → 读完整个响应"的原子操作，
# 在请求级别加锁即可杜绝交错。
_MSP_LOCK = threading.Lock()


def msp_request(ser: serial.Serial, cmd: int, payload: bytes = b"",
                timeout: float = 2.0) -> bytes:
    """发送一条 MSP 命令并等待响应（线程安全，全程持锁）。"""
    with _MSP_LOCK:
        ser.reset_input_buffer()              # 丢弃缓冲区里的旧数据
        ser.write(build_msp_request(cmd, payload))
        ser.flush()
        return read_msp_response(ser, cmd, timeout)


# ============================================================
# MSP v2（"$X" 帧）：支持超长数据（16 位长度 + CRC8 校验）
# ============================================================
# v2 帧格式：
#   '$' 'X' 方向  标志(1字节)  命令码(2字节小端)  长度(2字节小端)  [数据...]  CRC8(1字节)
#   CRC 算法：CRC8-DVB-S2（多项式 0xD5，初值 0x00），覆盖 标志~数据 全部字节
# 用途：MSP_DATAFLASH_READ 用 v2 单帧最多可返回约 4KB 数据，
#       实测下载速度从 v1 的 6KB/s 提升到约 47KB/s（7~8 倍）。

def crc8_dvb_s2(data: bytes) -> int:
    """CRC8-DVB-S2 校验（MSPv2 帧的校验算法）"""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def build_msp2_request(cmd: int, payload: bytes = b"") -> bytes:
    """构造一个 MSP v2 请求帧（支持最长 65535 字节数据）"""
    body = (bytes([0])                                # 标志位：请求固定为 0
            + cmd.to_bytes(2, "little")
            + len(payload).to_bytes(2, "little")
            + payload)
    return b"$X<" + body + bytes([crc8_dvb_s2(body)])


def read_msp2_response(ser: serial.Serial, expected_cmd: int,
                       timeout: float = 5.0) -> bytes:
    """从串口读取并解析一帧 MSP v2 响应，失败抛出 MspError。"""
    deadline = time.time() + timeout

    def read_n(n: int) -> bytes:
        buf = b""
        while len(buf) < n:
            if time.time() > deadline:
                raise MspError("读取超时：飞控无响应")
            chunk = ser.read(n - len(buf))
            if chunk:
                buf += chunk
        return buf

    # 定位帧头 '$X'
    while True:
        if read_n(1) == b"$" and read_n(1) == b"X":
            break

    direction = read_n(1)
    flag = read_n(1)[0]
    cmd = int.from_bytes(read_n(2), "little")
    size = int.from_bytes(read_n(2), "little")
    payload = read_n(size)
    crc_byte = read_n(1)[0]

    if direction == b"!":                     # 飞控拒绝此命令
        raise MspError(f"飞控拒绝了命令 {cmd}（该固件可能不支持）")
    if direction != b">":
        raise MspError("收到无法识别的 MSPv2 数据帧")

    body = bytes([flag]) + cmd.to_bytes(2, "little") \
        + size.to_bytes(2, "little") + payload
    if crc8_dvb_s2(body) != crc_byte:
        raise MspError("数据校验失败（CRC8 不匹配）")
    if cmd != expected_cmd:
        raise MspError(f"命令码不匹配：期望 {expected_cmd}，实际 {cmd}")
    return payload


def msp2_request(ser: serial.Serial, cmd: int, payload: bytes = b"",
                 timeout: float = 5.0) -> bytes:
    """发送一条 MSP v2 命令并等待响应（与 v1 共用同一把串口锁）。"""
    with _MSP_LOCK:
        ser.reset_input_buffer()
        ser.write(build_msp2_request(cmd, payload))
        ser.flush()
        return read_msp2_response(ser, cmd, timeout)


def u16(data: bytes, offset: int) -> int:
    """从字节串中按小端序读取 2 字节无符号整数"""
    return data[offset] | (data[offset + 1] << 8)


def s16(data: bytes, offset: int) -> int:
    """从字节串中按小端序读取 2 字节有符号整数"""
    value = u16(data, offset)
    return value - 65536 if value >= 32768 else value


# ============================================================
# 第二部分：飞控数据查询与写入
# ============================================================

def query_flight_controller(ser: serial.Serial) -> dict:
    """查询飞控基本信息：固件版本、板子型号、机型/电机。"""
    info = {"firmware": "未知", "board": "未知", "motors": "未知"}

    # 固件版本（3 字节：主.次.修订）
    try:
        data = msp_request(ser, MSP_FC_VERSION)
        if len(data) >= 3:
            info["firmware"] = f"Betaflight {data[0]}.{data[1]}.{data[2]}"
    except MspError:
        pass

    # 固件名称确认（应为 "BTFL"）
    try:
        data = msp_request(ser, MSP_FC_VARIANT)
        variant = data[:4].decode("ascii", errors="replace")
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
    """把 23 字节原始数据解析成可读字典（比例值已除以 100）"""
    if len(raw) < 23:
        raise MspError(f"Rates 数据长度异常：实际 {len(raw)} 字节（需 23）")
    return {
        "rc_rate": [raw[0] / 100, raw[12] / 100, raw[11] / 100],  # R, P, Y
        "expo":    [raw[1] / 100, raw[13] / 100, raw[10] / 100],
        "rate":    [raw[2] / 100, raw[3] / 100, raw[4] / 100],
        "thr_mid": raw[6] / 100,
        "thr_expo": raw[7] / 100,
        "thr_limit_pct": raw[15],
        "rate_limit": [u16(raw, 16), u16(raw, 18), u16(raw, 20)],
        "rates_type": raw[22],
    }


def set_rc_value(raw: bytearray, field: str, axis: int, value: float):
    """修改某轴某字段（value 为浮点比例值，如 1.50），写回 bytearray"""
    raw[RC_FIELD_OFFSETS[field][axis]] = max(0, min(255, round(value * 100)))


def bf_rate_curve(stick: float, rc_rate: float, super_rate: float,
                  expo: float) -> float:
    """
    Betaflight 经典 Rates 公式：摇杆偏转 0~1 → 角速度（°/s）。
    中位附近由 rc_rate/expo 决定，满杆由 super_rate 拉升。
    """
    xe = stick * (1 - expo) + stick ** 3 * expo
    denom = max(0.01, 1 - super_rate * abs(xe))
    return 200 * rc_rate * xe / denom


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
    """把滤波器原始数据解析成 {键名: 整数值}"""
    if len(raw) < 49:
        raise MspError(f"滤波器数据长度异常：实际 {len(raw)} 字节（需 49）")
    result = {}
    for key, _name, offset, kind, _lo, _hi in FILTER_FIELDS:
        result[key] = u16(raw, offset) if kind == "u16" else raw[offset]
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


# ============================================================
# 第五部分：后台工作线程（防止界面卡死）
# ============================================================

class SerialWorker(QObject):
    """后台串口工作对象：所有耗时的串口操作都在这里执行"""

    # 信号定义（PyQt 信号是线程安全的，后台线程发、界面线程收）
    connected = pyqtSignal(dict)              # 连接成功（飞控信息）
    pid_ready = pyqtSignal(list, list)        # PID 读取成功（名称, 数值）
    status_ready = pyqtSignal(dict)           # 慢通道：电压/CPU/解锁标志
    fast_ready = pyqtSignal(dict)             # 快通道：姿态角/RC 通道
    write_done = pyqtSignal(str)              # 写入完成
    backup_done = pyqtSignal(str)             # 备份完成（文件路径）
    motor_count_ready = pyqtSignal(int)       # 电机通道数
    flash_progress = pyqtSignal(str)          # 闪存下载进度提示
    flash_done = pyqtSignal(str)              # 闪存黑匣子下载完成（文件路径）
    tuning_ready = pyqtSignal(dict)           # Rates/滤波器读取成功
                                            # {"rc_raw": [23字节], "filter_raw": [49字节]}
    error = pyqtSignal(str)                   # 错误信息
    status = pyqtSignal(str)                  # 状态栏提示

    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.fc_info = {}                     # 缓存飞控信息（备份时用）

    # ---------- 连接与信息查询 ----------

    def connect_and_query(self, port: str, baudrate: int):
        """打开串口 → 查询飞控信息 → 读取 PID → 查询电机通道数"""
        try:
            self.status.emit("正在打开串口……")
            self.close_port()
            self.serial_port = serial.Serial(
                port=port, baudrate=baudrate,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.2, write_timeout=2,
            )
            time.sleep(0.5)                   # 等待飞控串口稳定

            self.status.emit("正在获取飞控信息……")
            self.fc_info = query_flight_controller(self.serial_port)
            self.connected.emit(self.fc_info)

            self.status.emit("正在读取 PID 参数……")
            names, values = query_pid(self.serial_port)
            self.pid_ready.emit(names, values)

            try:
                count = query_motor_count(self.serial_port)
                self.motor_count_ready.emit(count)
            except MspError:
                pass

            # 连接后顺带读取 Rates / 滤波器配置（v0.5）
            try:
                self.status.emit("正在读取 Rates 与滤波器配置……")
                rc_raw = query_rc_tuning(self.serial_port)
                filter_raw = query_filter_config(self.serial_port)
                self.tuning_ready.emit({"rc_raw": list(rc_raw),
                                        "filter_raw": list(filter_raw)})
            except MspError:
                pass                          # 个别固件不支持则跳过

            self.status.emit("就绪")

        except serial.SerialException as e:
            self.error.emit(f"无法打开串口 {port}：{e}\n请确认没有其他程序"
                            "（如 Betaflight Configurator）占用该串口。")
            self.close_port()
        except MspError as e:
            self.error.emit(f"MSP 通信失败：{e}")
            self.close_port()
        except Exception as e:
            self.error.emit(f"发生未知错误：{e}")
            self.close_port()

    # ---------- 实时状态轮询（快慢双通道） ----------
    # 快通道（100ms）：姿态角 + RC 通道 —— 影响"手感"，需要高刷新率
    # 慢通道（700ms）：电压/CPU/解锁标志 —— 变化慢，低频即可，减轻飞控负担

    def poll_fast(self):
        """快通道：只读姿态角和 RC 通道（2 条 MSP 请求，约 20ms）"""
        if not self.is_connected:
            return
        try:
            data = {"attitude": query_attitude(self.serial_port)}
            try:
                data["rc"] = query_rc(self.serial_port)
            except MspError:
                data["rc"] = []
            self.fast_ready.emit(data)
        except (MspError, serial.SerialException) as e:
            self.error.emit(f"实时数据读取失败：{e}")

    def poll_status(self):
        """慢通道：读取模拟量 + 扩展状态（仪表盘电源/CPU 区域用）"""
        if not self.is_connected:
            return
        try:
            data = {}
            data.update(query_analog(self.serial_port))
            try:
                data.update(query_status_ex(self.serial_port))
            except MspError:
                pass                          # 个别固件不支持 STATUS_EX
            self.status_ready.emit(data)
        except (MspError, serial.SerialException) as e:
            self.error.emit(f"实时数据读取失败：{e}")

    # ---------- PID 写入 / 备份 / 恢复 ----------

    def write_pids(self, names: list, values: list, backup_first: bool):
        """
        写入 PID 到飞控并保存到闪存。
        backup_first=True 时先自动备份当前参数，防止调乱后无法恢复。
        """
        if not self.is_connected:
            self.error.emit("未连接飞控")
            return
        try:
            if backup_first:
                self.status.emit("正在备份当前参数……")
                _, current = query_pid(self.serial_port)
                path = save_backup(self.fc_info, names, current)
                self.backup_done.emit(str(path))

            self.status.emit("正在写入 PID 并保存到闪存……")
            write_pid(self.serial_port, values, save_eeprom=True)

            # 重新读取一遍，确认写入生效
            names2, values2 = query_pid(self.serial_port)
            self.pid_ready.emit(names2, values2)
            self.write_done.emit("PID 已写入飞控并保存到闪存 ✅")

        except (MspError, serial.SerialException) as e:
            self.error.emit(f"写入失败：{e}")
        except Exception as e:
            self.error.emit(f"写入时发生未知错误：{e}")

    def restore_pids(self, file_path: str):
        """从备份 JSON 文件恢复 PID 并写入飞控"""
        if not self.is_connected:
            self.error.emit("未连接飞控")
            return
        try:
            self.status.emit("正在读取备份文件……")
            names, values = load_backup(Path(file_path))
            self.status.emit("正在恢复参数到飞控……")
            write_pid(self.serial_port, values, save_eeprom=True)
            names2, values2 = query_pid(self.serial_port)
            self.pid_ready.emit(names2, values2)
            self.write_done.emit(f"已从备份恢复 ✅\n{file_path}")
        except (MspError, serial.SerialException) as e:
            self.error.emit(f"恢复失败：{e}")
        except Exception as e:
            self.error.emit(f"备份文件读取失败：{e}")

    def backup_now(self, names: list):
        """手动备份当前飞控参数（不写入任何东西）"""
        if not self.is_connected:
            self.error.emit("未连接飞控")
            return
        try:
            self.status.emit("正在备份当前参数……")
            _, values = query_pid(self.serial_port)
            path = save_backup(self.fc_info, names, values)
            self.backup_done.emit(str(path))
            self.write_done.emit(f"备份完成 ✅\n{path}")
        except Exception as e:
            self.error.emit(f"备份失败：{e}")

    # ---------- 电机测试 ----------

    def set_motor_values(self, values: list):
        """发送电机输出值（电机测试页用）。调用方必须已完成安全确认。"""
        if not self.is_connected:
            return
        try:
            set_motors(self.serial_port, values)
        except (MspError, serial.SerialException) as e:
            self.error.emit(f"电机控制失败：{e}")

    # ---------- 闪存黑匣子下载 ----------

    def download_blackbox_flash(self, cancel_event,
                                tail_bytes: int = 0, erase_after: bool = False):
        """
        从飞控板载闪存下载黑匣子日志，保存为 .bbl 文件。
        流程：查询闪存信息 → 分块下载 → 存到 logs/ 目录 → （可选）清空闪存。
        参数：tail_bytes > 0 时只下载最后 tail_bytes 字节（最新的一次飞行
              记录在闪存末尾，只下尾巴可以快很多）；
              erase_after = 下载成功后清空闪存（下次只积累新日志）。
        """
        if not self.is_connected:
            self.error.emit("未连接飞控")
            return
        try:
            self.flash_progress.emit("正在查询飞控闪存……")
            summary = query_dataflash_summary(self.serial_port)
            if not summary["supported"] or summary["used_bytes"] == 0:
                self.error.emit(
                    "飞控上没有可下载的黑匣子数据。\n"
                    "可能原因：板子没有板载闪存芯片，或黑匣子记录未开启。\n"
                    "请在 Betaflight Configurator 的「黑盒子」页确认"
                    "存储设备可用并开启记录，飞一次后再下载。")
                return

            used = summary["used_bytes"]
            start = max(0, used - tail_bytes) if tail_bytes > 0 else 0

            # 尾部下载时：确认区间内有日志段头，否则向前扩展起点
            # （最近一次飞行可能很长，段头在区间之外，没有段头解码器无法识别）
            if tail_bytes > 0:
                def scan_progress(neg_pos):
                    scanned_mb = (used + neg_pos) / 1048576   # neg_pos 是负数
                    self.flash_progress.emit(
                        f"正在定位最近一次飞行的日志起点"
                        f"（已扫描 {scanned_mb:.1f} MB）……")

                try:
                    last_hdr = find_last_log_start(
                        self.serial_port, used,
                        progress_cb=scan_progress,
                        cancel_flag=cancel_event)
                    if last_hdr < start:
                        start = last_hdr
                except MspError as e:
                    if "取消" in str(e):
                        raise
                    # 扫描失败则按原范围下载（可能解不出，但不影响全量下载）

            total = used - start

            def on_progress(done_bytes):
                pct = done_bytes / total * 100
                self.flash_progress.emit(
                    f"下载中 {done_bytes / 1024:.0f} / "
                    f"{total / 1024:.0f} KB（{pct:.0f}%）")

            raw = download_dataflash(self.serial_port, used,
                                     start_address=start,
                                     progress_cb=on_progress,
                                     cancel_flag=cancel_event)
            LOGS_DIR.mkdir(exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = LOGS_DIR / f"flash_download_{timestamp}.bbl"
            path.write_bytes(raw)

            if erase_after:
                self.flash_progress.emit("下载完成，正在清空飞控闪存……")
                erase_dataflash(self.serial_port)
                self.flash_progress.emit("闪存已清空 ✅")

            self.flash_done.emit(str(path))

        except (MspError, serial.SerialException) as e:
            self.error.emit(f"闪存下载失败：{e}")
        except Exception as e:
            self.error.emit(f"闪存下载发生未知错误：{e}")

    # ---------- Rates / 滤波器读写（v0.5） ----------

    def read_tuning(self):
        """重新读取 Rates 与滤波器配置"""
        if not self.is_connected:
            self.error.emit("未连接飞控")
            return
        try:
            self.status.emit("正在读取 Rates 与滤波器配置……")
            rc_raw = query_rc_tuning(self.serial_port)
            filter_raw = query_filter_config(self.serial_port)
            self.tuning_ready.emit({"rc_raw": list(rc_raw),
                                    "filter_raw": list(filter_raw)})
            self.status.emit("Rates / 滤波器已读取")
        except (MspError, serial.SerialException) as e:
            self.error.emit(f"读取调参配置失败：{e}")

    def write_tuning(self, rc_raw: list, filter_raw: list):
        """写入 Rates + 滤波器配置（写入前自动做全量快照备份）"""
        if not self.is_connected:
            self.error.emit("未连接飞控")
            return
        try:
            self._snapshot_backup()
            self.status.emit("正在写入 Rates / 滤波器配置……")
            write_rc_tuning(self.serial_port, bytes(rc_raw), save_eeprom=False)
            write_filter_config(self.serial_port, bytes(filter_raw),
                                save_eeprom=False)
            msp_request(self.serial_port, MSP_EEPROM_WRITE, b"", timeout=5.0)
            # 重新读取确认
            rc2 = query_rc_tuning(self.serial_port)
            filt2 = query_filter_config(self.serial_port)
            self.tuning_ready.emit({"rc_raw": list(rc2),
                                    "filter_raw": list(filt2)})
            self.write_done.emit("Rates / 滤波器配置已写入并保存 ✅")
        except (MspError, serial.SerialException) as e:
            self.error.emit(f"写入失败：{e}")
        except Exception as e:
            self.error.emit(f"写入时发生未知错误：{e}")

    # ---------- 调参方案（预设） ----------

    def _snapshot_backup(self) -> Path:
        """把当前 PID+Rates+滤波器 整体快照存到 backups/（写入类操作前调用）"""
        names, values = query_pid(self.serial_port)
        rc_raw = query_rc_tuning(self.serial_port)
        filter_raw = query_filter_config(self.serial_port)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        snap = tuning_snapshot(self.fc_info, names, values,
                               rc_raw, filter_raw,
                               name=f"写入前自动备份 {timestamp}")
        path = save_preset_file(BACKUP_DIR,
                                f"full_backup_{timestamp}.json", snap)
        self.backup_done.emit(str(path))
        return path

    def capture_preset(self, name: str):
        """读取飞控当前全部调参状态，保存为预设文件"""
        if not self.is_connected:
            self.error.emit("未连接飞控")
            return
        try:
            self.status.emit("正在读取飞控当前配置……")
            names, values = query_pid(self.serial_port)
            rc_raw = query_rc_tuning(self.serial_port)
            filter_raw = query_filter_config(self.serial_port)
            snap = tuning_snapshot(self.fc_info, names, values,
                                   rc_raw, filter_raw, name=name)
            safe = "".join(c for c in name
                           if c not in '\\/:*?"<>|').strip() or "preset"
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = save_preset_file(PRESETS_DIR,
                                    f"{safe}_{timestamp}.json", snap)
            self.write_done.emit(f"预设已保存 ✅\n{path}")
        except (MspError, serial.SerialException) as e:
            self.error.emit(f"保存预设失败：{e}")
        except Exception as e:
            self.error.emit(f"保存预设时发生未知错误：{e}")

    def apply_preset(self, preset: dict):
        """把预设完整写回飞控（PID + Rates + 滤波器），写入前自动备份"""
        if not self.is_connected:
            self.error.emit("未连接飞控")
            return
        try:
            self._snapshot_backup()
            self.status.emit("正在写入预设……")
            pid_values = [tuple(v) for v in preset["pid_values"]]
            write_pid(self.serial_port, pid_values, save_eeprom=False)
            write_rc_tuning(self.serial_port,
                            bytes(preset["rc_tuning_raw"]), save_eeprom=False)
            write_filter_config(self.serial_port,
                                bytes(preset["filter_raw"]), save_eeprom=False)
            msp_request(self.serial_port, MSP_EEPROM_WRITE, b"", timeout=5.0)
            # 重新读取，刷新界面
            names2, values2 = query_pid(self.serial_port)
            self.pid_ready.emit(names2, values2)
            rc2 = query_rc_tuning(self.serial_port)
            filt2 = query_filter_config(self.serial_port)
            self.tuning_ready.emit({"rc_raw": list(rc2),
                                    "filter_raw": list(filt2)})
            self.write_done.emit(
                f"预设「{preset.get('name', '')}」已应用并保存到闪存 ✅")
        except KeyError as e:
            self.error.emit(f"预设文件缺少字段：{e}")
        except (MspError, serial.SerialException) as e:
            self.error.emit(f"应用预设失败：{e}")
        except Exception as e:
            self.error.emit(f"应用预设时发生未知错误：{e}")

    # ---------- 断开 ----------

    def close_port(self):
        """安全关闭串口"""
        if self.serial_port is not None:
            try:
                if self.serial_port.is_open:
                    self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None

    @property
    def is_connected(self) -> bool:
        return self.serial_port is not None and self.serial_port.is_open


# ============================================================
# 第五部分（B）：AI 助手 —— Ollama 本地大模型通信
# ============================================================
# 使用 Ollama 的 HTTP API（http://localhost:11434）与本地大模型对话。
# 只用 Python 标准库 urllib，不需要额外安装依赖。
# 所有网络请求都在后台线程执行（由 MainWindow._run_in_thread 驱动），
# 通过 pyqtSignal 把流式生成的文字安全地送回界面线程。

OLLAMA_BASE_URL = "http://localhost:11434"
AI_RECOMMENDED_MODELS = ["qwen2.5:1.5b", "qwen2.5:3b"]   # RTX 2060 6GB 适用

AI_SYSTEM_PROMPT = (
    "你是 ApexFlight 内置的无人机调参专家助手，精通 Betaflight 固件、"
    "PID 调参、滤波设置和黑匣子日志分析。"
    "请始终使用简体中文回答，语言简洁、给出可操作建议。"
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


# ============================================================
# 第五部分：自定义控件
# ============================================================

class ToggleSwitch(QAbstractButton):
    """
    胶囊开关（仿 Betaflight Configurator 的 toggle 样式）：
    关 = 灰色胶囊 + 白色圆点在左；开 = 橙色胶囊 + 白色圆点在右。
    用法和 QCheckBox 一样：isChecked() / setChecked() / toggled 信号。
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setCheckable(True)
        self.setFixedSize(38, 20)
        self.setCursor(Qt.CursorShape.PointingHandCursor)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        rect = self.rect().adjusted(1, 1, -1, -1)
        on = self.isChecked()

        # 胶囊背景（开 = 图标橙，关 = 深灰）
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(245, 168, 61) if on else QColor(54, 60, 68))
        painter.drawRoundedRect(rect, rect.height() / 2, rect.height() / 2)

        # 白色圆形滑块
        d = rect.height() - 4
        x = rect.right() - d - 2 if on else rect.left() + 2
        painter.setBrush(QColor(255, 255, 255))
        painter.drawEllipse(int(x), int(rect.top() + 2), int(d), int(d))
        painter.end()

    def sizeHint(self):
        return self.minimumSizeHint()


class AttitudeIndicator(QWidget):
    """
    人工地平线仪表：模拟真实飞行仪表，显示横滚和俯仰。
    蓝色 = 天空，棕色 = 地面，中间的线 = 地平线。
    """

    def __init__(self):
        super().__init__()
        self._roll = 0.0
        self._pitch = 0.0
        self.setMinimumSize(180, 180)

    def set_attitude(self, roll: float, pitch: float):
        """更新姿态角（单位：度）并重绘"""
        self._roll, self._pitch = roll, pitch
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        size = int(min(self.width(), self.height()))
        painter.translate(self.width() / 2, self.height() / 2)

        # 圆形裁剪，画出仪表外形
        # 注意 1：QPainter 没有 setClipEllipse 方法（Qt4 老 API），
        #         Qt6 必须用 QPainterPath + setClipPath 实现椭圆裁剪
        # 注意 2：fillRect 等只接受整数坐标，浮点数会抛 TypeError
        radius = int(size / 2 - 4)
        clip_path = QPainterPath()
        clip_path.addEllipse(-radius, -radius, radius * 2, radius * 2)
        painter.setClipPath(clip_path)

        # 按横滚角旋转、按俯仰角上下平移整个天地
        # Betaflight 的角度约定与航空仪表相反：
        #   俯仰值为正 = 机头下压 → 应看到更多地面 → 地平线上移（取负号）
        #   横滚旋转方向同理取反，与 Configurator 的 3D 模型保持一致
        painter.save()
        painter.rotate(self._roll)
        pitch_pixels = int(max(-radius, min(radius, -self._pitch * 2)))

        # 天/地配色与 ApexFlight 图标一致：青色天空 + 深棕地面
        painter.fillRect(-size, -size * 2 + pitch_pixels,
                         size * 2, size * 2, QColor(62, 198, 232))   # 天空
        painter.fillRect(-size, pitch_pixels,
                         size * 2, size * 2, QColor(74, 56, 38))     # 地面
        # 地平线
        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.drawLine(-size, int(pitch_pixels), size, int(pitch_pixels))
        painter.restore()

        # 中央固定的飞机符号（图标橙色，不随姿态转动）
        painter.setPen(QPen(QColor(245, 168, 61), 4))
        painter.drawLine(-30, 0, -10, 0)
        painter.drawLine(10, 0, 30, 0)
        painter.drawLine(0, 0, 0, 6)

        # 外圈边框
        painter.setClipping(False)
        painter.setPen(QPen(QColor(120, 120, 120), 2))
        painter.drawEllipse(int(-radius), int(-radius),
                            int(radius * 2), int(radius * 2))
        painter.end()


# ============================================================
# 第六部分：GUI 主窗口
# ============================================================

class MainWindow(QMainWindow):
    """ApexFlight 主窗口（6 个功能页签）"""

    # AI 探测结果信号（后台线程探测 Ollama → 界面线程刷新显示）
    ai_probe_done = pyqtSignal(bool, list)

    def __init__(self):
        super().__init__()
        self.worker = SerialWorker()
        self.ai = AIBridge()                  # AI 对话桥（Ollama）
        self._ai_messages = []                # 对话历史（发给模型的上下文）
        self._ai_reply_buffer = ""            # 当前这一轮回答的累积文字
        self._ai_busy = False                 # AI 是否正在回答
        self._threads = []                    # 持有线程引用，防止被回收
        self._polling = False                 # 慢通道轮询线程是否正在跑
        self._polling_fast = False            # 快通道轮询线程是否正在跑
        self._pid_names = []                  # 当前 PID 名称列表
        self._motor_sliders = []              # 电机滑块列表
        self._motor_count = 0
        self._rc_bars = []                    # RC 通道显示条

        self.setWindowTitle("ApexFlight")
        self.resize(960, 680)
        self._center_on_screen()
        if ICON_PATH.exists():
            self.setWindowIcon(QIcon(str(ICON_PATH)))

        self._build_ui()
        self._apply_stylesheet()
        self._connect_signals()

        # 慢通道定时器：700ms 刷一次电压/CPU/解锁标志
        self.poll_timer = QTimer(self)
        self.poll_timer.setInterval(700)
        self.poll_timer.timeout.connect(self._poll_once)

        # 快通道定时器：100ms 刷一次姿态角和 RC 通道（约 10 帧/秒）
        self.fast_timer = QTimer(self)
        self.fast_timer.setInterval(100)
        self.fast_timer.timeout.connect(self._poll_fast_once)

        self.refresh_ports()

        # 界面和信号都就绪后，后台探测一次 Ollama 服务状态
        QTimer.singleShot(300, self.on_ai_refresh)

    def _apply_stylesheet(self):
        """应用全局暗色主题（配色取自 ApexFlight 图标：青 #3EC6E8 + 橙 #F5A83D）"""
        self.setStyleSheet("""
            /* 全局底色与字体 */
            QMainWindow, QWidget {
                background: #1B1E23;
                color: #E8E8E8;
                font-family: "Microsoft YaHei", "Segoe UI";
                font-size: 13px;
            }
            /* 顶栏 */
            QWidget#topbar {
                background: #14161A;
                border-bottom: 1px solid #363C44;
            }
            QLabel#titleLabel {
                color: #3EC6E8;
                font-size: 20px;
                font-weight: bold;
                padding-left: 4px;
            }
            QLabel#subtitleLabel { color: #9AA0A6; padding-left: 8px; }
            /* 左侧导航栏（仿 BF 侧边菜单） */
            QListWidget#sidebar {
                background: #14161A;
                border: none;
                border-right: 1px solid #363C44;
                outline: none;
            }
            QListWidget#sidebar::item {
                padding: 14px 12px;
                color: #9AA0A6;
                border-left: 3px solid transparent;
            }
            QListWidget#sidebar::item:selected {
                background: #23272E;
                color: #3EC6E8;
                border-left: 3px solid #3EC6E8;
            }
            QListWidget#sidebar::item:hover { background: #1F2329; }
            /* 分组框 */
            QGroupBox {
                border: 1px solid #363C44;
                border-radius: 8px;
                margin-top: 14px;
                padding-top: 12px;
                background: #23272E;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 6px;
                color: #3EC6E8;
                font-weight: bold;
            }
            /* 按钮 */
            QPushButton {
                background: #2E333B;
                border: 1px solid #454C55;
                border-radius: 6px;
                padding: 6px 14px;
                color: #E8E8E8;
            }
            QPushButton:hover { background: #383F48; border-color: #3EC6E8; }
            QPushButton:pressed { background: #262B32; }
            QPushButton:disabled { color: #5A6068; background: #24282E; }
            QPushButton#connectBtn {
                background: #3EC6E8;
                color: #10222A;
                font-weight: bold;
                border: none;
            }
            QPushButton#connectBtn:hover { background: #5BD2EE; }
            QPushButton#connectBtn:disabled { background: #27505C; color: #7FA5AF; }
            QPushButton#disconnectBtn {
                background: transparent;
                border: 1px solid #F5A83D;
                color: #F5A83D;
            }
            QPushButton#disconnectBtn:hover { background: #3A2E1A; }
            QPushButton#dangerBtn {
                background: #E04545;
                color: white;
                font-weight: bold;
                border: none;
                padding: 8px 20px;
            }
            QPushButton#dangerBtn:hover { background: #F05858; }
            QPushButton#dangerBtn:disabled { background: #5C2B2B; color: #9A7A7A; }
            /* 下拉框 */
            QComboBox {
                background: #2E333B;
                border: 1px solid #454C55;
                border-radius: 6px;
                padding: 5px 8px;
            }
            QComboBox:hover { border-color: #3EC6E8; }
            QComboBox QAbstractItemView {
                background: #2E333B;
                selection-background-color: #3EC6E8;
                selection-color: #10222A;
            }
            /* 表格 */
            QTableWidget {
                background: #1F2329;
                gridline-color: #363C44;
                border: 1px solid #363C44;
                border-radius: 6px;
            }
            QHeaderView::section {
                background: #2E333B;
                color: #3EC6E8;
                padding: 6px;
                border: none;
                font-weight: bold;
            }
            QTableWidget::item:selected {
                background: #3EC6E8;
                color: #10222A;
            }
            QLineEdit {
                background: #2E333B;
                border: 1px solid #3EC6E8;
                border-radius: 4px;
                padding: 2px 4px;
            }
            /* 滑块 */
            QSlider::groove:horizontal {
                height: 6px;
                background: #363C44;
                border-radius: 3px;
            }
            QSlider::sub-page:horizontal {
                background: #3EC6E8;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                width: 16px;
                height: 16px;
                margin: -6px 0;
                border-radius: 8px;
                background: #3EC6E8;
            }
            QSlider::handle:horizontal:disabled { background: #5A6068; }
            QSlider::sub-page:horizontal:disabled { background: #3A4048; }
            /* 复选框 */
            QCheckBox { spacing: 6px; }
            QCheckBox::indicator {
                width: 16px;
                height: 16px;
                border: 1px solid #454C55;
                border-radius: 3px;
                background: #2E333B;
            }
            QCheckBox::indicator:checked {
                background: #F5A83D;
                border-color: #F5A83D;
            }
            /* 状态栏 */
            QStatusBar {
                background: #14161A;
                color: #9AA0A6;
                border-top: 1px solid #363C44;
            }
            /* 滚动区（通道开关列表） */
            QScrollArea {
                border: 1px solid #363C44;
                border-radius: 6px;
                background: #1F2329;
            }
            /* 滚动条 */
            QScrollBar:vertical {
                background: #1B1E23;
                width: 10px;
            }
            QScrollBar::handle:vertical {
                background: #3A4048;
                border-radius: 5px;
                min-height: 30px;
            }
            QScrollBar::handle:vertical:hover { background: #3EC6E8; }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
        """)

    # ---------- 窗口初始化 ----------

    def _center_on_screen(self):
        """窗口居中显示"""
        screen = QApplication.primaryScreen().availableGeometry()
        self.move((screen.width() - self.width()) // 2,
                  (screen.height() - self.height()) // 2)

    def _build_ui(self):
        """构建主界面（仿 Betaflight Configurator 布局）：
        顶栏（图标 + 标题 + 连接控件）+ 左侧导航栏 + 右侧页面区"""
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # ---- 顶栏：图标 + 标题 + 连接控件（连接按钮在最右侧，同 BF）----
        topbar = QWidget()
        topbar.setObjectName("topbar")
        top = QHBoxLayout(topbar)
        top.setContentsMargins(14, 8, 14, 8)

        if ICON_PATH.exists():
            icon_label = QLabel()
            icon_label.setPixmap(QPixmap(str(ICON_PATH)).scaled(
                36, 36, Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation))
            top.addWidget(icon_label)

        title = QLabel("ApexFlight")
        title.setObjectName("titleLabel")
        top.addWidget(title)
        top.addStretch()

        top.addWidget(QLabel("串口"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(210)
        top.addWidget(self.port_combo)
        self.refresh_button = QPushButton("刷新")
        self.refresh_button.clicked.connect(self.refresh_ports)
        top.addWidget(self.refresh_button)
        top.addWidget(QLabel("波特率"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["57600", "115200", "230400", "460800"])
        self.baud_combo.setCurrentText("115200")
        top.addWidget(self.baud_combo)
        self.connect_button = QPushButton("连接")
        self.connect_button.setObjectName("connectBtn")
        self.connect_button.setMinimumWidth(90)
        self.connect_button.clicked.connect(self.on_connect_clicked)
        top.addWidget(self.connect_button)
        self.disconnect_button = QPushButton("断开")
        self.disconnect_button.setObjectName("disconnectBtn")
        self.disconnect_button.setEnabled(False)
        self.disconnect_button.clicked.connect(self.on_disconnect_clicked)
        top.addWidget(self.disconnect_button)
        layout.addWidget(topbar)

        # ---- 主体：左侧导航 + 右侧页面堆栈 ----
        body = QHBoxLayout()
        body.setContentsMargins(0, 0, 0, 0)
        body.setSpacing(0)

        self.sidebar = QListWidget()
        self.sidebar.setObjectName("sidebar")
        self.sidebar.setFixedWidth(150)
        self.sidebar.setHorizontalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff)   # 隐藏横向滚动条
        self.sidebar.addItems(["📊  仪表盘", "🎛️  PID 调参", "🎯  Rates 调参",
                               "🌊  滤波器", "⚙️  电机测试", "📡  接收机",
                               "📈  黑匣子", "💾  调参方案", "🤖  AI 助手"])
        self.sidebar.setCurrentRow(0)
        body.addWidget(self.sidebar)

        self.pages = QStackedWidget()
        self.pages.addWidget(self._build_dashboard_tab())
        self.pages.addWidget(self._build_pid_tab())
        self.pages.addWidget(self._build_rates_tab())
        self.pages.addWidget(self._build_filter_tab())
        self.pages.addWidget(self._build_motor_tab())
        self.pages.addWidget(self._build_rc_tab())
        self.pages.addWidget(self._build_blackbox_tab())
        self.pages.addWidget(self._build_preset_tab())
        self.pages.addWidget(self._build_ai_tab())
        body.addWidget(self.pages, 1)

        body_widget = QWidget()
        body_widget.setLayout(body)
        layout.addWidget(body_widget, 1)

        # 导航点击切换页面
        self.sidebar.currentRowChanged.connect(self.pages.setCurrentIndex)

        self.statusBar().showMessage("就绪：请选择串口后点击「连接」")

    # ---------- 页签 1：仪表盘 ----------

    def _build_dashboard_tab(self) -> QWidget:
        tab = QWidget()
        layout = QHBoxLayout(tab)

        # 左列：飞控信息 + 电源信息
        left = QVBoxLayout()

        info_box = QGroupBox("飞控信息")
        form = QFormLayout(info_box)
        self.firmware_label = QLabel("未连接")
        self.board_label = QLabel("未连接")
        self.motors_label = QLabel("未连接")
        form.addRow("固件版本：", self.firmware_label)
        form.addRow("飞控型号：", self.board_label)
        form.addRow("机型/电机：", self.motors_label)
        left.addWidget(info_box)

        power_box = QGroupBox("电源 / 链路")
        form2 = QFormLayout(power_box)
        self.voltage_label = QLabel("—")
        self.amps_label = QLabel("—")
        self.mah_label = QLabel("—")
        self.rssi_label = QLabel("—")
        form2.addRow("电池电压：", self.voltage_label)
        form2.addRow("电流：", self.amps_label)
        form2.addRow("已耗电：", self.mah_label)
        form2.addRow("信号强度：", self.rssi_label)
        left.addWidget(power_box)

        fc_box = QGroupBox("飞控状态")
        form3 = QFormLayout(fc_box)
        self.cpu_label = QLabel("—")
        self.cycle_label = QLabel("—")
        self.arming_label = QLabel("—")
        self.arming_label.setWordWrap(True)
        form3.addRow("CPU 负载：", self.cpu_label)
        form3.addRow("循环时间：", self.cycle_label)
        form3.addRow("解锁禁用：", self.arming_label)
        left.addWidget(fc_box)
        left.addStretch()
        layout.addLayout(left, 1)

        # 右列：姿态指示器 + 角度数值
        right = QVBoxLayout()
        attitude_box = QGroupBox("飞行姿态（拿起飞机转一转试试）")
        att_layout = QVBoxLayout(attitude_box)
        self.horizon = AttitudeIndicator()
        att_layout.addWidget(self.horizon,
                             alignment=Qt.AlignmentFlag.AlignCenter)
        self.attitude_label = QLabel("横滚 — ｜ 俯仰 — ｜ 航向 —")
        self.attitude_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        att_layout.addWidget(self.attitude_label)
        right.addWidget(attitude_box)
        layout.addLayout(right, 1)
        return tab

    # ---------- 页签 2：PID 调参 ----------

    def _build_pid_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        hint = QLabel("直接双击表格中的数值进行修改，改完点「写入飞控」。"
                      "写入前会自动备份当前参数到 backups/ 文件夹。")
        hint.setWordWrap(True)
        layout.addWidget(hint)

        self.pid_table = QTableWidget(0, 3)
        self.pid_table.setHorizontalHeaderLabels(["P", "I", "D"])
        self.pid_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch)
        layout.addWidget(self.pid_table)

        buttons = QHBoxLayout()
        self.pid_reload_btn = QPushButton("重新读取")
        self.pid_reload_btn.clicked.connect(self.on_pid_reload)
        self.pid_reload_btn.setEnabled(False)
        buttons.addWidget(self.pid_reload_btn)

        self.pid_write_btn = QPushButton("写入飞控")
        self.pid_write_btn.clicked.connect(self.on_pid_write)
        self.pid_write_btn.setEnabled(False)
        buttons.addWidget(self.pid_write_btn)

        self.pid_backup_btn = QPushButton("备份当前参数")
        self.pid_backup_btn.clicked.connect(self.on_pid_backup)
        self.pid_backup_btn.setEnabled(False)
        buttons.addWidget(self.pid_backup_btn)

        self.pid_restore_btn = QPushButton("从备份恢复")
        self.pid_restore_btn.clicked.connect(self.on_pid_restore)
        self.pid_restore_btn.setEnabled(False)
        buttons.addWidget(self.pid_restore_btn)
        buttons.addStretch()
        layout.addLayout(buttons)
        return tab

    # ---------- 页签 3：电机测试 ----------

    def _build_motor_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        # 安全警告区（红字）
        warning = QLabel("⚠️ 危险：电机测试会让电机真实转动！\n"
                         "使用前必须【拆下所有螺旋桨】，并确认飞机固定牢固、"
                         "周围没有人员和杂物。")
        warning.setStyleSheet("color: #E04545; font-weight: bold;")
        warning.setWordWrap(True)
        layout.addWidget(warning)

        # 双重安全确认
        self.motor_check1 = QCheckBox("我已拆下所有螺旋桨")
        self.motor_check2 = QCheckBox("我了解风险，确认开始测试")
        self.motor_check1.stateChanged.connect(self._update_motor_lock)
        self.motor_check2.stateChanged.connect(self._update_motor_lock)
        layout.addWidget(self.motor_check1)
        layout.addWidget(self.motor_check2)

        # 电机滑块区域（连接后按实际通道数动态生成）
        self.motor_area = QGroupBox("电机输出（未连接）")
        self.motor_layout = QGridLayout(self.motor_area)
        layout.addWidget(self.motor_area)

        # 全部停止按钮
        self.motor_stop_btn = QPushButton("🛑 全部停止")
        self.motor_stop_btn.setObjectName("dangerBtn")
        self.motor_stop_btn.setEnabled(False)
        self.motor_stop_btn.clicked.connect(self.on_motor_stop)
        layout.addWidget(self.motor_stop_btn)
        layout.addStretch()
        return tab

    # ---------- 页签 4：接收机通道 ----------

    def _build_rc_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        hint = QLabel("实时显示接收机各通道数值（正常范围约 1000~2000，"
                      "中位约 1500）。打开发射机并拨动摇杆，数值会跟着动。")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        self.rc_area = QGroupBox("通道（未连接）")
        self.rc_layout = QGridLayout(self.rc_area)
        layout.addWidget(self.rc_area)
        layout.addStretch()
        return tab

    # ---------- 页签 5：黑匣子分析（对标 BF Blackbox Explorer）----------

    def _build_blackbox_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        # 顶部：文件操作 + 日志段切换 + 日志信息
        top = QHBoxLayout()
        self.bb_open_btn = QPushButton("📂 打开日志文件")
        self.bb_open_btn.clicked.connect(self.on_bb_open)
        top.addWidget(self.bb_open_btn)
        self.bb_demo_btn = QPushButton("🧪 生成演示日志")
        self.bb_demo_btn.clicked.connect(self.on_bb_demo)
        top.addWidget(self.bb_demo_btn)
        # 从飞控闪存直接下载黑匣子（需先连接飞控）
        top.addWidget(QLabel("下载范围："))
        self.bb_flash_range = QComboBox()
        self.bb_flash_range.addItem("最近 1 MB（约 25 秒）", 1 * 1048576)
        self.bb_flash_range.addItem("最近 2 MB（约 45 秒）", 2 * 1048576)
        self.bb_flash_range.addItem("最近 4 MB（约 1.5 分钟）", 4 * 1048576)
        self.bb_flash_range.addItem("最近 8 MB（约 3 分钟）", 8 * 1048576)
        self.bb_flash_range.addItem("全部（慢）", 0)
        self.bb_flash_range.setCurrentIndex(1)     # 默认最近 2MB
        self.bb_flash_range.setToolTip(
            "黑匣子日志在闪存里是顺序追加写入的，最新的一次飞行在末尾。\n"
            "大多数时候只需要最近一次飞行的日志，选「最近 N MB」几十秒就能下完。\n"
            "如果最近一次飞行特别长，会自动向前扩展到该次飞行的起点，\n"
            "确保下载的数据能被解码。")
        top.addWidget(self.bb_flash_range)
        self.bb_flash_btn = QPushButton("📥 从飞控下载")
        self.bb_flash_btn.clicked.connect(self.on_bb_flash_download)
        self.bb_flash_btn.setEnabled(False)
        top.addWidget(self.bb_flash_btn)
        self.bb_erase_toggle = ToggleSwitch()
        self.bb_erase_toggle.setToolTip(
            "下载成功后自动清空飞控闪存。\n"
            "清空后下次只积累新日志，下载只需几秒钟。")
        top.addWidget(self.bb_erase_toggle)
        top.addWidget(QLabel("下完清空闪存"))
        self._flash_cancel = None             # 下载取消标志

        self.bb_session_label = QLabel("日志段：")
        self.bb_session_label.setVisible(False)   # 多段日志时才显示
        top.addWidget(self.bb_session_label)
        self.bb_session_combo = QComboBox()
        self.bb_session_combo.currentIndexChanged.connect(
            self.on_bb_session_changed)
        self.bb_session_combo.setVisible(False)
        top.addWidget(self.bb_session_combo)

        self.bb_info_btn = QPushButton("ℹ️ 日志信息")
        self.bb_info_btn.clicked.connect(self.on_bb_info)
        self.bb_info_btn.setEnabled(False)
        top.addWidget(self.bb_info_btn)

        self.bb_file_label = QLabel("未加载日志（支持 .bbl / .bfl / .csv）")
        top.addWidget(self.bb_file_label, 1)
        layout.addLayout(top)

        if not HAS_MPL:
            warn = QLabel("⚠️ 未安装 matplotlib，无法绘图。\n"
                          "请在终端运行：pip install matplotlib")
            warn.setStyleSheet("color: #E04545; font-weight: bold;")
            layout.addWidget(warn)
            layout.addStretch()
            return tab

        body = QHBoxLayout()

        # ---- 左侧控制面板 ----
        left = QVBoxLayout()
        left.addWidget(QLabel("选择通道（打开开关即选中）："))
        # 通道开关列表（仿 BF 的 toggle 样式），放在滚动区里
        self.bb_channel_scroll = QScrollArea()
        self.bb_channel_scroll.setWidgetResizable(True)
        self.bb_channel_scroll.setMaximumWidth(260)
        self.bb_channel_container = QWidget()
        self.bb_channel_rows = QVBoxLayout(self.bb_channel_container)
        self.bb_channel_rows.setContentsMargins(4, 4, 4, 4)
        self.bb_channel_rows.setSpacing(8)
        self.bb_channel_scroll.setWidget(self.bb_channel_container)
        left.addWidget(self.bb_channel_scroll, 1)
        self.bb_toggles = {}                  # {原始列名: ToggleSwitch}

        # 时间范围裁剪
        range_box = QGroupBox("时间范围")
        range_form = QFormLayout(range_box)
        self.bb_start = QDoubleSpinBox()
        self.bb_start.setRange(0, 99999)
        self.bb_start.setSuffix(" 秒")
        self.bb_end = QDoubleSpinBox()
        self.bb_end.setRange(0, 99999)
        self.bb_end.setSuffix(" 秒")
        range_form.addRow("起点：", self.bb_start)
        range_form.addRow("终点：", self.bb_end)
        left.addWidget(range_box)

        self.bb_normalize = QCheckBox("归一化显示（比较形状）")
        left.addWidget(self.bb_normalize)

        self.bb_plot_btn = QPushButton("🎨 绘制曲线")
        self.bb_plot_btn.setObjectName("connectBtn")
        self.bb_plot_btn.clicked.connect(self.on_bb_plot)
        self.bb_plot_btn.setEnabled(False)
        left.addWidget(self.bb_plot_btn)

        self.bb_fft_btn = QPushButton("📶 频谱分析")
        self.bb_fft_btn.clicked.connect(self.on_bb_fft)
        self.bb_fft_btn.setEnabled(False)
        left.addWidget(self.bb_fft_btn)

        # 游标读数
        self.bb_cursor_label = QLabel("移动鼠标到图上查看数值")
        self.bb_cursor_label.setWordWrap(True)
        self.bb_cursor_label.setStyleSheet("color: #9AA0A6;")
        left.addWidget(self.bb_cursor_label)

        self.bb_stats_label = QLabel("")
        self.bb_stats_label.setWordWrap(True)
        left.addWidget(self.bb_stats_label)
        body.addLayout(left)

        # ---- 右侧：工具栏 + 画布 ----
        right = QVBoxLayout()
        self.bb_figure = Figure(figsize=(6, 4), facecolor="#1B1E23")
        self.bb_canvas = FigureCanvasQTAgg(self.bb_figure)
        self.bb_toolbar = NavigationToolbar2QT(self.bb_canvas, tab)
        self.bb_toolbar.setStyleSheet(
            "QToolBar { background: #23272E; border: none; }")
        right.addWidget(self.bb_toolbar)
        right.addWidget(self.bb_canvas, 1)
        body.addLayout(right, 1)
        layout.addLayout(body, 1)

        # 鼠标游标事件
        self.bb_canvas.mpl_connect("motion_notify_event",
                                   self._bb_on_mouse_move)

        # 数据缓存
        self.bb_time = []
        self.bb_data = {}
        self.bb_columns = []
        self.bb_sessions = []             # 多段日志的 CSV 路径
        self.bb_header_info = {}          # .bbl 头部信息
        self.bb_axes = []                 # 当前图中的子图
        self.bb_cursor_lines = []         # 游标竖线
        self.bb_plotted = []              # [(列名, 数值, 显示名)]，游标读数用
        return tab

    # ---------- 黑匣子：文件加载 ----------

    def on_bb_demo(self):
        """生成演示日志并自动加载"""
        try:
            self.statusBar().showMessage("正在生成演示日志……")
            path = generate_demo_log()
            self.bb_sessions = [path]
            self.bb_header_info = {}
            self._load_blackbox_file(path)
        except Exception as e:
            self.on_error(f"生成演示日志失败：{e}")

    def on_bb_flash_download(self):
        """从飞控板载闪存下载黑匣子日志（再次点击可取消）"""
        if self._flash_cancel is not None and not self._flash_cancel.is_set():
            # 正在下载 → 本次点击表示取消
            self._flash_cancel.set()
            self.bb_flash_btn.setText("📥 从飞控下载")
            self.statusBar().showMessage("正在取消下载……")
            return
        self._flash_cancel = threading.Event()
        self.bb_flash_btn.setText("⏹ 取消下载")
        # 下载期间暂停姿态/状态轮询：
        # 轮询线程会和下载线程抢同一把串口锁，拖慢下载速度
        self.fast_timer.stop()
        self.poll_timer.stop()
        tail = self.bb_flash_range.currentData() or 0
        erase = self.bb_erase_toggle.isChecked()
        self._run_in_thread(self.worker.download_blackbox_flash,
                            self._flash_cancel, tail, erase)

    def _resume_polling_after_flash(self):
        """闪存下载结束（完成/取消/失败）后恢复轮询定时器"""
        if self.worker.is_connected:
            self.fast_timer.start()
            self.poll_timer.start()

    def on_flash_done(self, path_str: str):
        """闪存下载完成：解码并自动加载"""
        self.bb_flash_btn.setText("📥 从飞控下载")
        self._flash_cancel = None
        self._resume_polling_after_flash()
        try:
            path = Path(path_str)
            self.statusBar().showMessage("下载完成，正在解码……")
            self.bb_sessions = decode_blackbox(path)
            self.bb_header_info = parse_bbl_header(path)
            self._setup_session_combo()
            self._load_blackbox_file(self.bb_sessions[0])
            self.statusBar().showMessage(
                f"飞控黑匣子已加载：共 {len(self.bb_sessions)} 段飞行记录 ✅")
        except Exception as e:
            self.on_error(f"解码下载的日志失败：{e}")

    def on_bb_open(self):
        """打开日志文件（.bbl/.bfl 自动解码全部日志段，.csv 直接读取）"""
        path_str, _ = QFileDialog.getOpenFileName(
            self, "选择黑匣子日志", str(LOGS_DIR),
            "黑匣子日志 (*.bbl *.bfl *.csv);;所有文件 (*)")
        if not path_str:
            return
        path = Path(path_str)
        try:
            if path.suffix.lower() in (".bbl", ".bfl"):
                self.statusBar().showMessage(
                    "正在解码二进制日志（大文件可能需要一两分钟）……")
                self.bb_sessions = decode_blackbox(path)   # 全部日志段
                self.bb_header_info = parse_bbl_header(path)
                self.statusBar().showMessage(
                    f"解码完成：共 {len(self.bb_sessions)} 段飞行记录")
            else:
                self.bb_sessions = [path]
                self.bb_header_info = {}
            self._setup_session_combo()
            self._load_blackbox_file(self.bb_sessions[0])
        except BlackboxError as e:
            self.on_error(str(e))
        except Exception as e:
            self.on_error(f"日志读取失败：{e}")

    def _setup_session_combo(self):
        """根据日志段数量更新下拉框（单段时隐藏）"""
        self.bb_session_combo.blockSignals(True)
        self.bb_session_combo.clear()
        for i, csv_path in enumerate(self.bb_sessions):
            self.bb_session_combo.addItem(f"第 {i + 1} 段（{csv_path.name}）")
        multi = len(self.bb_sessions) > 1
        self.bb_session_combo.setVisible(multi)
        self.bb_session_label.setVisible(multi)
        self.bb_session_combo.blockSignals(False)

    def on_bb_session_changed(self, index: int):
        """切换日志段"""
        if 0 <= index < len(self.bb_sessions):
            try:
                self._load_blackbox_file(self.bb_sessions[index])
            except Exception as e:
                self.on_error(f"加载第 {index + 1} 段日志失败：{e}")

    def on_bb_info(self):
        """弹出日志头部信息（固件版本、机名、PID、滤波配置等）"""
        if not self.bb_header_info:
            return
        lines = [f"{k}：{v}" for k, v in self.bb_header_info.items()]
        QMessageBox.information(self, "日志信息", "\n".join(lines))

    def _load_blackbox_file(self, csv_path: Path):
        """解析 CSV 并填充通道列表"""
        self.statusBar().showMessage("正在解析日志数据……")
        self.bb_time, self.bb_data, self.bb_columns = \
            load_blackbox_csv(csv_path)
        duration = self.bb_time[-1]
        self.bb_file_label.setText(
            f"已加载：{csv_path.name}（{duration:.1f} 秒，"
            f"{len(self.bb_columns)} 个通道）")
        self.bb_info_btn.setEnabled(bool(self.bb_header_info))

        # 时间范围控件初始化为全程
        self.bb_start.setRange(0, duration)
        self.bb_end.setRange(0, duration)
        self.bb_start.setValue(0)
        self.bb_end.setValue(duration)

        # 填充通道开关列表（中文名优先，仿 BF toggle 样式）
        # 先清空旧开关
        while self.bb_channel_rows.count():
            item = self.bb_channel_rows.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
            elif item.layout():
                while item.layout().count():
                    sub = item.layout().takeAt(0)
                    if sub.widget():
                        sub.widget().deleteLater()
        self.bb_toggles = {}
        for col in self.bb_columns:
            display = CHANNEL_NAMES.get(col, col)
            row = QHBoxLayout()
            toggle = ToggleSwitch()
            toggle.toggled.connect(lambda _on: self.on_bb_plot())  # 开关即重绘
            row.addWidget(toggle)
            name_label = QLabel(display)
            row.addWidget(name_label, 1)
            row_widget = QWidget()
            row_widget.setLayout(row)
            self.bb_channel_rows.addWidget(row_widget)
            self.bb_toggles[col] = toggle
        self.bb_channel_rows.addStretch()
        # 默认打开前两个通道
        for col in self.bb_columns[:2]:
            self.bb_toggles[col].setChecked(True)
        self.bb_plot_btn.setEnabled(True)
        self.bb_fft_btn.setEnabled(True)
        self.statusBar().showMessage("日志加载完成，点击「绘制曲线」")
        self.on_bb_plot()

    # ---------- 黑匣子：数据切片 ----------

    def _bb_selected_channels(self) -> list:
        """返回所有开关处于打开状态的通道（保持原始列顺序）"""
        return [col for col, toggle in self.bb_toggles.items()
                if toggle.isChecked()]

    def _bb_slice(self, values: list) -> tuple:
        """按时间范围控件裁剪数据，返回 (时间, 数值)"""
        t0, t1 = self.bb_start.value(), self.bb_end.value()
        if t1 <= t0:
            t1 = self.bb_time[-1]
        idx = [i for i, t in enumerate(self.bb_time) if t0 <= t <= t1]
        if not idx:
            idx = list(range(len(self.bb_time)))
        return ([self.bb_time[i] for i in idx],
                [values[i] for i in idx])

    # ---------- 黑匣子：曲线绘制（多子图堆叠，仿 BF Explorer）----------

    def on_bb_plot(self):
        """每个选中通道一个子图轨道，共享时间轴"""
        if not self.bb_time:
            return
        selected = self._bb_selected_channels()
        if not selected:
            self.statusBar().showMessage("请先勾选至少一个通道")
            return
        if len(selected) > 10:
            self.statusBar().showMessage("一次最多绘制 10 个通道，请减少选择")
            return

        normalize = self.bb_normalize.isChecked()
        fig = self.bb_figure
        fig.clear()

        self.bb_axes = []
        self.bb_cursor_lines = []
        self.bb_plotted = []
        stats_lines = []

        for row, col in enumerate(selected):
            ax = fig.add_subplot(len(selected), 1, row + 1,
                                 sharex=self.bb_axes[0] if self.bb_axes
                                 else None)
            ax.set_facecolor("#1B1E23")
            t, values = self._bb_slice(self.bb_data[col])
            if normalize:
                peak = max(abs(min(values)), abs(max(values)), 1e-9)
                values = [v / peak for v in values]
            display = CHANNEL_NAMES.get(col, col)
            ax.plot(t, values, linewidth=0.8, color="#3EC6E8")
            ax.set_ylabel(display, color="#E8E8E8", fontsize=9)
            ax.grid(True, alpha=0.2, color="#9AA0A6")
            ax.tick_params(colors="#9AA0A6", labelsize=8)
            for spine in ax.spines.values():
                spine.set_color("#363C44")
            # 游标竖线（初始隐藏）
            vline = ax.axvline(x=t[0], color="#F5A83D", linewidth=0.8,
                               visible=False)
            self.bb_cursor_lines.append(vline)
            self.bb_axes.append(ax)
            self.bb_plotted.append((col, values, display))
            stats_lines.append(
                f"{display}：均值 {sum(values)/len(values):.1f}，"
                f"范围 {min(values):.0f} ~ {max(values):.0f}")

        self.bb_axes[-1].set_xlabel("时间 (秒)", color="#E8E8E8")
        title = "归一化对比" if normalize else "黑匣子数据轨道"
        self.bb_axes[0].set_title(title, color="#3EC6E8")
        fig.tight_layout()
        self.bb_canvas.draw()

        self.bb_stats_label.setText("\n".join(stats_lines[:4]))
        self.statusBar().showMessage(
            f"已绘制 {len(selected)} 个通道轨道（鼠标移动可看读数）")

    def _bb_on_mouse_move(self, event):
        """鼠标在图上移动时：显示游标竖线和该时刻各通道数值"""
        if not self.bb_axes or event.xdata is None:
            return
        # 找最近的时间点
        t = event.xdata
        for vline in self.bb_cursor_lines:
            vline.set_xdata([t, t])
            vline.set_visible(True)

        # 计算各通道该时刻的值
        parts = [f"t = {t:.3f}s"]
        for col, values, display in self.bb_plotted[:5]:
            # 二分找最近下标（时间轴有序）
            import bisect
            i = bisect.bisect_left(self.bb_time, t)
            i = max(0, min(len(self.bb_time) - 1, i))
            parts.append(f"{display}: {values[min(i, len(values)-1)]:.1f}")
        self.bb_cursor_label.setText("\n".join(parts))
        self.bb_canvas.draw_idle()

    # ---------- 黑匣子：频谱分析（FFT）----------

    def on_bb_fft(self):
        """对选中通道做 FFT 频谱分析，自动标注前 3 个噪声峰"""
        if not HAS_MPL or not self.bb_time:
            return
        selected = self._bb_selected_channels()
        if not selected:
            self.statusBar().showMessage("请先勾选要做频谱分析的通道")
            return

        import numpy as np

        fig = self.bb_figure
        fig.clear()
        ax = fig.add_subplot(111)
        ax.set_facecolor("#1B1E23")
        self.bb_axes = [ax]
        self.bb_cursor_lines = []
        self.bb_plotted = []

        colors = ["#3EC6E8", "#F5A83D", "#7CE38B", "#E04545", "#C792EA"]
        peak_notes = []
        for idx, col in enumerate(selected[:5]):
            t, values = self._bb_slice(self.bb_data[col])
            y = np.array(values, dtype=float)
            y = y[~np.isnan(y)]
            if len(y) < 64:
                continue
            # 采样率：由时间轴中位间隔决定
            dt = np.median(np.diff(np.array(t)[:len(y)]))
            if dt <= 0:
                continue
            fs = 1.0 / dt
            y = y - y.mean()                      # 去直流
            y = y * np.hanning(len(y))            # 汉宁窗减少频谱泄漏
            spectrum = np.abs(np.fft.rfft(y)) / len(y) * 2
            freqs = np.fft.rfftfreq(len(y), dt)

            display = CHANNEL_NAMES.get(col, col)
            color = colors[idx % len(colors)]
            ax.plot(freqs, spectrum, label=display,
                    linewidth=0.8, color=color)

            # 标注前 3 个峰（忽略 20Hz 以下的机身运动频率）
            valid = spectrum[freqs > 20]
            valid_freqs = freqs[freqs > 20]
            if len(valid) > 0:
                top = np.argsort(valid)[-3:]
                for j in sorted(top):
                    f, amp = valid_freqs[j], valid[j]
                    ax.annotate(f"{f:.0f}Hz", xy=(f, amp),
                                textcoords="offset points", xytext=(0, 6),
                                color=color, fontsize=8, ha="center")
                    peak_notes.append(f"{display} 噪声峰 ≈ {f:.0f} Hz")

        ax.set_xlabel("频率 (Hz)", color="#E8E8E8")
        ax.set_ylabel("幅度", color="#E8E8E8")
        ax.set_title("频谱分析（噪声峰位置决定滤波器截止频率）",
                     color="#3EC6E8")
        ax.legend(loc="upper right", fontsize=8,
                  facecolor="#23272E", labelcolor="#E8E8E8")
        ax.grid(True, alpha=0.2, color="#9AA0A6")
        ax.tick_params(colors="#9AA0A6")
        for spine in ax.spines.values():
            spine.set_color("#363C44")
        fig.tight_layout()
        self.bb_canvas.draw()
        self.bb_stats_label.setText("\n".join(peak_notes[:6]))
        # v0.5：噪声峰同步显示到「滤波器」页，辅助设置低通/陷波
        if hasattr(self, "filter_peak_label"):
            self.filter_peak_label.setText(
                "黑匣子噪声峰：" + "；".join(peak_notes[:4])
                if peak_notes else "黑匣子噪声峰：未找到明显噪声峰")
        self.statusBar().showMessage("频谱分析完成（点「绘制曲线」返回时域图）")

    # ---------- 页签 3：Rates 调参（v0.5） ----------

    def _build_rates_tab(self) -> QWidget:
        tab = QWidget()
        layout = QHBoxLayout(tab)

        # 左列：三轴滑块 + 写入按钮
        left = QVBoxLayout()
        hint = QLabel("拖动滑块实时预览手感曲线，确认后点「写入飞控」。"
                      "写入前会自动备份当前全部配置到 backups/ 文件夹。")
        hint.setWordWrap(True)
        left.addWidget(hint)

        self._rc_raw = None                   # 23 字节 bytearray
        self._rates_controls = {}             # {(字段, 轴): (滑块, 数值标签)}

        axes = [("横滚 Roll", 0), ("俯仰 Pitch", 1), ("偏航 Yaw", 2)]
        fields = [("rc_rate", "中位灵敏度 RC Rate"),
                  ("rate", "满杆速率 Super Rate"),
                  ("expo", "中位指数 Expo")]
        for axis_name, axis in axes:
            box = QGroupBox(axis_name)
            grid = QGridLayout(box)
            for row, (field, field_name) in enumerate(fields):
                slider = QSlider(Qt.Orientation.Horizontal)
                slider.setRange(0, 255)       # 存储值 0~255 = 0.00~2.55
                value_label = QLabel("—")
                value_label.setMinimumWidth(40)
                slider.valueChanged.connect(
                    lambda v, f=field, a=axis: self._on_rates_slider(f, a, v))
                grid.addWidget(QLabel(field_name), row, 0)
                grid.addWidget(slider, row, 1)
                grid.addWidget(value_label, row, 2)
                self._rates_controls[(field, axis)] = (slider, value_label)
            left.addWidget(box)

        btns = QHBoxLayout()
        self.rates_reload_btn = QPushButton("重新读取")
        self.rates_reload_btn.clicked.connect(self.on_tuning_reload)
        self.rates_reload_btn.setEnabled(False)
        btns.addWidget(self.rates_reload_btn)
        self.rates_write_btn = QPushButton("写入飞控")
        self.rates_write_btn.setObjectName("connectBtn")
        self.rates_write_btn.clicked.connect(self.on_tuning_write)
        self.rates_write_btn.setEnabled(False)
        btns.addWidget(self.rates_write_btn)
        btns.addStretch()
        left.addLayout(btns)
        left.addStretch()
        layout.addLayout(left, 1)

        # 右列：手感曲线
        right = QVBoxLayout()
        curve_box = QGroupBox("手感曲线（摇杆偏转 → 角速度）")
        curve_layout = QVBoxLayout(curve_box)
        if HAS_MPL:
            self.rates_figure = Figure(figsize=(5, 4), facecolor="#1B1E23")
            self.rates_canvas = FigureCanvasQTAgg(self.rates_figure)
            curve_layout.addWidget(self.rates_canvas)
        else:
            curve_layout.addWidget(QLabel("未安装 matplotlib，无法绘制曲线"))
        self.rates_max_label = QLabel("满杆角速度：—")
        curve_layout.addWidget(self.rates_max_label)
        right.addWidget(curve_box)
        layout.addLayout(right, 1)
        return tab

    # ---------- Rates / 滤波器：数据到达与写回 ----------

    def on_tuning_reload(self):
        self.statusBar().showMessage("正在读取 Rates 与滤波器配置……")
        self._run_in_thread(self.worker.read_tuning)

    def on_tuning_ready(self, data: dict):
        """Rates/滤波器数据到达（连接后、写入后、手动读取都会触发）"""
        try:
            self._rc_raw = bytearray(data["rc_raw"])
            parsed = parse_rc_tuning(bytes(self._rc_raw))
            for (field, axis), (slider, label) in \
                    self._rates_controls.items():
                slider.blockSignals(True)
                slider.setValue(round(parsed[field][axis] * 100))
                slider.blockSignals(False)
                label.setText(f"{parsed[field][axis]:.2f}")
            self._draw_rates_curve()
            self.rates_write_btn.setEnabled(True)
        except (MspError, KeyError, TypeError):
            self._rc_raw = None
            self.rates_write_btn.setEnabled(False)
        try:
            self._filter_raw = bytearray(data["filter_raw"])
            values = parse_filter_config(bytes(self._filter_raw))
            for key, spin in self._filter_spins.items():
                spin.blockSignals(True)
                spin.setValue(values[key])
                spin.blockSignals(False)
            self.filter_write_btn.setEnabled(True)
        except (MspError, KeyError, TypeError):
            self._filter_raw = None
            self.filter_write_btn.setEnabled(False)

    def _on_rates_slider(self, field: str, axis: int, value: int):
        """滑块变化：只更新本地数据与曲线，确认后才写飞控"""
        if self._rc_raw is None:
            return
        set_rc_value(self._rc_raw, field, axis, value / 100)
        _slider, label = self._rates_controls[(field, axis)]
        label.setText(f"{value / 100:.2f}")
        self._draw_rates_curve()

    def _draw_rates_curve(self):
        """按当前滑块值绘制三轴手感曲线"""
        if not HAS_MPL or self._rc_raw is None:
            return
        parsed = parse_rc_tuning(bytes(self._rc_raw))
        fig = self.rates_figure
        fig.clear()
        ax = fig.add_subplot(111)
        ax.set_facecolor("#1B1E23")
        sticks = [i / 100 for i in range(101)]
        colors = ["#3EC6E8", "#F5A83D", "#7CE38B"]
        names = ["横滚", "俯仰", "偏航"]
        maxes = []
        for axis in range(3):
            curve = [bf_rate_curve(s, parsed["rc_rate"][axis],
                                   parsed["rate"][axis],
                                   parsed["expo"][axis]) for s in sticks]
            ax.plot([s * 100 for s in sticks], curve, color=colors[axis],
                    linewidth=1.2, label=names[axis])
            maxes.append(f"{names[axis]} {curve[-1]:.0f}")
        title = "手感曲线"
        if parsed.get("rates_type", 0) != 0:
            title += "（固件使用非经典 Rates 类型，曲线仅供参考）"
        ax.set_title(title, color="#3EC6E8")
        ax.set_xlabel("摇杆偏转 (%)", color="#E8E8E8")
        ax.set_ylabel("角速度 (°/s)", color="#E8E8E8")
        ax.legend(loc="upper left", fontsize=8, facecolor="#23272E",
                  labelcolor="#E8E8E8")
        ax.grid(True, alpha=0.2, color="#9AA0A6")
        ax.tick_params(colors="#9AA0A6")
        for spine in ax.spines.values():
            spine.set_color("#363C44")
        fig.tight_layout()
        self.rates_canvas.draw()
        self.rates_max_label.setText(
            "满杆角速度：" + " ｜ ".join(maxes) + " °/s")

    def on_tuning_write(self):
        """把 Rates 与滤波器一起写入飞控（一次 EEPROM 保存）"""
        if self._rc_raw is None or self._filter_raw is None:
            self.statusBar().showMessage(
                "配置尚未读取完整：请先连接飞控或点「重新读取」")
            return
        # 安全检查：低通设为 0 = 关闭滤波，需要额外警告
        warnings = []
        values = parse_filter_config(bytes(self._filter_raw))
        if values["gyro_lpf1_hz"] == 0 or values["dterm_lpf1_hz"] == 0:
            warnings.append("陀螺仪或 D 项低通被设为 0（关闭滤波），"
                            "噪声可能烧毁电机/电调！")
        msg = ("将把当前 Rates 与滤波器设置写入飞控并保存到闪存。\n"
               "写入前会自动备份当前全部配置到 backups/ 文件夹。\n\n")
        if warnings:
            msg += "⚠️ " + "\n⚠️ ".join(warnings) + "\n\n"
        msg += "确定继续吗？"
        reply = QMessageBox.question(self, "确认写入", msg)
        if reply != QMessageBox.StandardButton.Yes:
            return
        self._run_in_thread(self.worker.write_tuning,
                            list(self._rc_raw), list(self._filter_raw))

    # ---------- 页签 4：滤波器（v0.5） ----------

    def _build_filter_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        hint = QLabel("滤波器用于压制机架振动噪声。截止频率 0 = 关闭该滤波器"
                      "（危险！）。建议先在「黑匣子」页做频谱分析，"
                      "找到噪声峰后再调整。")
        hint.setWordWrap(True)
        layout.addWidget(hint)

        self._filter_raw = None               # 49 字节 bytearray
        self._filter_spins = {}               # {键名: QSpinBox}

        name_map = {k: n for k, n, _o, _t, _lo, _hi in FILTER_FIELDS}
        range_map = {k: (lo, hi) for k, _n, _o, _t, lo, hi in FILTER_FIELDS}
        groups = [
            ("静态低通（常用）", ["gyro_lpf1_hz", "gyro_lpf2_hz",
                              "dterm_lpf1_hz", "dterm_lpf2_hz", "yaw_lpf_hz"]),
            ("动态低通", ["gyro_dyn_min", "gyro_dyn_max",
                        "dterm_dyn_min", "dterm_dyn_max", "dyn_expo"]),
            ("动态陷波", ["notch_q", "notch_min", "notch_max", "notch_count"]),
            ("RPM 滤波", ["rpm_harmonics", "rpm_min_hz"]),
        ]
        group_row = QHBoxLayout()
        for group_name, keys in groups:
            box = QGroupBox(group_name)
            form = QFormLayout(box)
            for key in keys:
                spin = QSpinBox()
                lo, hi = range_map[key]
                spin.setRange(lo, hi)
                spin.setMaximumWidth(110)
                spin.valueChanged.connect(
                    lambda v, k=key: self._on_filter_spin(k, v))
                form.addRow(name_map[key] + "：", spin)
                self._filter_spins[key] = spin
            group_row.addWidget(box)
        layout.addLayout(group_row)

        # 与黑匣子频谱联动：显示最近一次 FFT 找到的噪声峰
        self.filter_peak_label = QLabel("黑匣子噪声峰：尚未做频谱分析"
                                        "（黑匣子页 → 频谱分析）")
        self.filter_peak_label.setWordWrap(True)
        self.filter_peak_label.setStyleSheet("color: #9AA0A6;")
        layout.addWidget(self.filter_peak_label)

        btns = QHBoxLayout()
        self.filter_reload_btn = QPushButton("重新读取")
        self.filter_reload_btn.clicked.connect(self.on_tuning_reload)
        self.filter_reload_btn.setEnabled(False)
        btns.addWidget(self.filter_reload_btn)
        self.filter_write_btn = QPushButton("写入飞控")
        self.filter_write_btn.setObjectName("connectBtn")
        self.filter_write_btn.clicked.connect(self.on_tuning_write)
        self.filter_write_btn.setEnabled(False)
        btns.addWidget(self.filter_write_btn)
        btns.addStretch()
        layout.addLayout(btns)
        layout.addStretch()
        return tab

    def _on_filter_spin(self, key: str, value: int):
        """滤波器数值变化：只更新本地数据，确认后才写飞控"""
        if self._filter_raw is None:
            return
        set_filter_value(self._filter_raw, key, value)

    # ---------- 页签 8：调参方案（v0.5） ----------

    def _build_preset_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        hint = QLabel("预设 = 一整套调参状态（PID + Rates + 滤波器）。\n"
                      "把当前飞控状态保存为预设，之后可一键切换；"
                      "应用前会自动备份当前配置，随时可以调回来。")
        hint.setWordWrap(True)
        layout.addWidget(hint)

        self.preset_list = QListWidget()
        layout.addWidget(self.preset_list, 1)

        name_row = QHBoxLayout()
        name_row.addWidget(QLabel("预设名称："))
        self.preset_name_edit = QLineEdit()
        self.preset_name_edit.setPlaceholderText("例如：花飞手感 / 竞速稳拍")
        name_row.addWidget(self.preset_name_edit, 1)
        layout.addLayout(name_row)

        btns = QHBoxLayout()
        self.preset_save_btn = QPushButton("💾 保存当前为预设")
        self.preset_save_btn.setObjectName("connectBtn")
        self.preset_save_btn.clicked.connect(self.on_preset_save)
        self.preset_save_btn.setEnabled(False)
        btns.addWidget(self.preset_save_btn)
        self.preset_apply_btn = QPushButton("✅ 应用选中预设")
        self.preset_apply_btn.clicked.connect(self.on_preset_apply)
        self.preset_apply_btn.setEnabled(False)
        btns.addWidget(self.preset_apply_btn)
        self.preset_delete_btn = QPushButton("🗑️ 删除选中")
        self.preset_delete_btn.setObjectName("dangerBtn")
        self.preset_delete_btn.clicked.connect(self.on_preset_delete)
        btns.addWidget(self.preset_delete_btn)
        self.preset_refresh_btn = QPushButton("🔄 刷新列表")
        self.preset_refresh_btn.clicked.connect(self.refresh_preset_list)
        btns.addWidget(self.preset_refresh_btn)
        btns.addStretch()
        layout.addLayout(btns)

        self.refresh_preset_list()
        return tab

    # ---------- 调参方案：列表与操作 ----------

    def refresh_preset_list(self):
        """扫描 presets/ 目录刷新预设列表"""
        if not hasattr(self, "preset_list"):
            return
        self.preset_list.clear()
        PRESETS_DIR.mkdir(exist_ok=True)
        for path in sorted(PRESETS_DIR.glob("*.json"), reverse=True):
            try:
                data = load_preset_file(path)
                item = QListWidgetItem(
                    f"{data.get('name', path.stem)}　"
                    f"（{data.get('saved_time', '')[:16]} · "
                    f"{data.get('board', '')}）")
                item.setData(Qt.ItemDataRole.UserRole, str(path))
                self.preset_list.addItem(item)
            except Exception:
                self.preset_list.addItem(
                    QListWidgetItem(f"{path.name}（文件损坏）"))

    def _selected_preset_path(self):
        item = self.preset_list.currentItem()
        if not item:
            return None
        p = item.data(Qt.ItemDataRole.UserRole)
        return Path(p) if p else None

    def on_preset_save(self):
        name = self.preset_name_edit.text().strip()
        if not name:
            self.statusBar().showMessage("请先输入预设名称")
            return
        self._run_in_thread(self.worker.capture_preset, name)

    def on_preset_apply(self):
        path = self._selected_preset_path()
        if not path:
            self.statusBar().showMessage("请先在列表中选择一个预设")
            return
        try:
            preset = load_preset_file(path)
        except Exception as e:
            self.on_error(f"预设文件读取失败：{e}")
            return
        reply = QMessageBox.question(
            self, "确认应用预设",
            f"将把预设「{preset.get('name', path.stem)}」完整写入飞控\n"
            "（PID + Rates + 滤波器）并保存到闪存。\n"
            "写入前会自动备份当前配置到 backups/ 文件夹。\n\n确定继续吗？")
        if reply != QMessageBox.StandardButton.Yes:
            return
        self._run_in_thread(self.worker.apply_preset, preset)

    def on_preset_delete(self):
        path = self._selected_preset_path()
        if not path:
            self.statusBar().showMessage("请先在列表中选择一个预设")
            return
        reply = QMessageBox.question(
            self, "确认删除", f"将删除预设文件：\n{path}\n\n确定吗？")
        if reply != QMessageBox.StandardButton.Yes:
            return
        try:
            path.unlink()
            self.refresh_preset_list()
            self.statusBar().showMessage("预设已删除")
        except OSError as e:
            self.on_error(f"删除失败：{e}")

    # ---------- 页签 9：AI 助手（v0.4 新功能） ----------

    def _build_ai_tab(self) -> QWidget:
        """AI 助手页：连接本机 Ollama 大模型，做调参问答与数据分析"""
        tab = QWidget()
        layout = QVBoxLayout(tab)

        # ---- 顶部：服务状态 + 模型选择 ----
        top = QGroupBox("本地 AI 服务（Ollama）")
        top_row = QHBoxLayout(top)
        self.ai_status_label = QLabel("检测中……")
        top_row.addWidget(self.ai_status_label)
        top_row.addWidget(QLabel("模型："))
        self.ai_model_combo = QComboBox()
        self.ai_model_combo.setMinimumWidth(180)
        top_row.addWidget(self.ai_model_combo, 1)
        self.ai_refresh_btn = QPushButton("🔄 刷新状态")
        self.ai_refresh_btn.clicked.connect(self.on_ai_refresh)
        top_row.addWidget(self.ai_refresh_btn)
        self.ai_install_btn = QPushButton("📦 安装指引")
        self.ai_install_btn.clicked.connect(self.on_ai_install_help)
        top_row.addWidget(self.ai_install_btn)
        layout.addWidget(top)

        # ---- 快捷分析按钮 ----
        quick = QHBoxLayout()
        self.ai_pid_btn = QPushButton("🎛️ 分析当前 PID")
        self.ai_pid_btn.setToolTip("把当前读取到的 PID 参数发给 AI 分析")
        self.ai_pid_btn.clicked.connect(self.on_ai_analyze_pid)
        quick.addWidget(self.ai_pid_btn)
        self.ai_bb_btn = QPushButton("📈 分析黑匣子统计")
        self.ai_bb_btn.setToolTip("把黑匣子统计/频谱结果发给 AI 分析")
        self.ai_bb_btn.clicked.connect(self.on_ai_analyze_bb)
        quick.addWidget(self.ai_bb_btn)
        self.ai_clear_btn = QPushButton("🗑️ 清空对话")
        self.ai_clear_btn.clicked.connect(self.on_ai_clear)
        quick.addWidget(self.ai_clear_btn)
        quick.addStretch(1)
        layout.addLayout(quick)

        # ---- 对话显示区 ----
        self.ai_chat_view = QTextEdit()
        self.ai_chat_view.setReadOnly(True)
        self.ai_chat_view.setPlaceholderText(
            "AI 回答会显示在这里。\n"
            "提示：AI 在本地电脑上运行（Ollama），不会上传任何飞行数据。")
        layout.addWidget(self.ai_chat_view, 1)

        # ---- 输入行 ----
        input_row = QHBoxLayout()
        self.ai_input = QLineEdit()
        self.ai_input.setPlaceholderText(
            "输入你的调参问题，例如：翻滚时感觉有点软，应该怎么调？")
        self.ai_input.returnPressed.connect(self.on_ai_send)
        input_row.addWidget(self.ai_input, 1)
        self.ai_send_btn = QPushButton("发送")
        self.ai_send_btn.setObjectName("connectBtn")
        self.ai_send_btn.setMinimumWidth(80)
        self.ai_send_btn.clicked.connect(self.on_ai_send)
        input_row.addWidget(self.ai_send_btn)
        layout.addLayout(input_row)
        return tab

    # ---------- AI 助手：状态检测 ----------

    def _ai_probe_and_emit(self):
        """后台线程：探测 Ollama 并把结果通过信号发回界面"""
        running, models = ollama_status()
        self.ai_probe_done.emit(running, models)

    def on_ai_probe(self, running: bool, models: list):
        """界面线程：根据探测结果刷新状态行和模型下拉框"""
        self.ai_model_combo.blockSignals(True)
        self.ai_model_combo.clear()
        if running:
            if models:
                self.ai_status_label.setText("✅ Ollama 运行中")
                self.ai_status_label.setStyleSheet("color: #6FCF97;")
                self.ai_model_combo.addItems(models)
                # 优先选中推荐的小模型（若已安装）
                for rec in AI_RECOMMENDED_MODELS:
                    idx = self.ai_model_combo.findText(rec)
                    if idx >= 0:
                        self.ai_model_combo.setCurrentIndex(idx)
                        break
            else:
                self.ai_status_label.setText(
                    "⚠️ Ollama 运行中，但还没有安装模型")
                self.ai_status_label.setStyleSheet("color: #F5A83D;")
                self.ai_model_combo.addItem("（请先下载模型）", None)
        else:
            self.ai_status_label.setText("❌ 未检测到 Ollama 服务")
            self.ai_status_label.setStyleSheet("color: #E06C75;")
            self.ai_model_combo.addItem("（服务未运行）", None)
        self.ai_model_combo.blockSignals(False)

    def on_ai_refresh(self):
        self.ai_status_label.setText("检测中……")
        self.ai_status_label.setStyleSheet("")
        self._run_in_thread(self._ai_probe_and_emit)

    def on_ai_install_help(self):
        QMessageBox.information(
            self, "安装本地 AI（Ollama）",
            "ApexFlight 的 AI 助手使用 Ollama 在你的电脑上本地运行大模型，"
            "飞行数据不会上传到网络。\n\n"
            "安装步骤：\n"
            "1. 打开浏览器访问 https://ollama.com/download 下载 Windows 版并安装；\n"
            "2. 安装完成后 Ollama 会自动在后台运行；\n"
            "3. 按 Win+R 输入 cmd 打开命令行，执行：\n"
            "      ollama pull qwen2.5:1.5b      （快速问答，约 1GB）\n"
            "      ollama pull qwen2.5:3b        （深度分析，约 2GB，可选）\n"
            "4. 回到本页点击「🔄 刷新状态」即可开始对话。")

    # ---------- AI 助手：对话 ----------

    def on_ai_send(self):
        text = self.ai_input.text().strip()
        if not text:
            return
        self.ai_input.clear()
        self._ai_ask(text)

    def on_ai_analyze_pid(self):
        """快捷按钮：把当前 PID 表格内容发给 AI"""
        if not self._pid_names:
            self.statusBar().showMessage(
                "还没有 PID 数据：请先连接飞控")
            return
        lines = []
        for row, name in enumerate(self._pid_names):
            vals = []
            for col in range(3):
                item = self.pid_table.item(row, col)
                vals.append(item.text() if item else "?")
            lines.append(f"{name}: P={vals[0]} I={vals[1]} D={vals[2]}")
        info = self.worker.fc_info or {}
        prompt = (
            "请分析我这台穿越机当前的 PID 参数，指出是否合理、"
            "常见问题（如抖动、发软、洗桨）对应的调整方向：\n"
            f"固件：{info.get('firmware', '未知')}，"
            f"机架：{info.get('board', '未知')}\n"
            + "\n".join(lines))
        self._ai_ask(prompt)

    def on_ai_analyze_bb(self):
        """快捷按钮：把黑匣子统计结果发给 AI"""
        stats = self.bb_stats_label.text().strip()
        if not stats:
            self.statusBar().showMessage(
                "还没有黑匣子分析结果：请先在黑匣子页绘制曲线或做频谱分析")
            return
        prompt = (
            "这是我的穿越机黑匣子日志的统计/频谱分析结果，"
            "请解读这些数据反映了什么飞行状态或噪声问题，"
            "并给出滤波或 PID 调整建议：\n" + stats)
        self._ai_ask(prompt)

    def _ai_ask(self, user_text: str):
        """发起一轮 AI 对话（统一入口）"""
        if self._ai_busy:
            self.statusBar().showMessage("AI 正在回答中，请稍候……")
            return
        model = self.ai_model_combo.currentText()
        if not model or model.startswith("（"):
            self.statusBar().showMessage(
                "AI 不可用：请先安装并启动 Ollama（点「📦 安装指引」）")
            return

        # 把用户消息追加到对话记录与显示区
        self._ai_messages.append({"role": "user", "content": user_text})
        self.ai_chat_view.append(f"<b style='color:#3EC6E8'>你：</b> "
                                 f"{user_text}\n")
        self.ai_chat_view.append("<b style='color:#F5A83D'>AI：</b> ")
        self._ai_reply_buffer = ""

        self._ai_busy = True
        self.ai_send_btn.setText("⏹ 停止")
        self.ai_send_btn.clicked.disconnect()
        self.ai_send_btn.clicked.connect(self._ai_stop)

        # 完整对话历史（含系统提示词）发给后台线程
        messages = ([{"role": "system", "content": AI_SYSTEM_PROMPT}]
                    + self._ai_messages[-10:])   # 只带最近 10 条，控制上下文长度
        self._run_in_thread(self.ai.chat, model, messages)

    def _ai_stop(self):
        self.ai.cancel()
        self.statusBar().showMessage("已请求停止回答")

    def on_ai_token(self, piece: str):
        """收到一小段 AI 生成的文字，追加到显示区"""
        self._ai_reply_buffer += piece
        cursor = self.ai_chat_view.textCursor()
        cursor.movePosition(cursor.MoveOperation.End)
        cursor.insertText(piece)
        self.ai_chat_view.setTextCursor(cursor)
        self.ai_chat_view.ensureCursorVisible()

    def on_ai_done(self):
        """一轮回答结束：恢复按钮、保存到对话记录"""
        self._ai_busy = False
        self.ai_send_btn.setText("发送")
        self.ai_send_btn.clicked.disconnect()
        self.ai_send_btn.clicked.connect(self.on_ai_send)
        if self._ai_reply_buffer:
            self._ai_messages.append(
                {"role": "assistant", "content": self._ai_reply_buffer})
            self.ai_chat_view.append("")          # 换行分隔
        self.statusBar().showMessage("AI 回答完成")

    def on_ai_failed(self, msg: str):
        self._ai_busy = False
        self.ai_send_btn.setText("发送")
        self.ai_send_btn.clicked.disconnect()
        self.ai_send_btn.clicked.connect(self.on_ai_send)
        self.ai_chat_view.append(
            f"<span style='color:#E06C75'>⚠️ {msg}</span>\n")
        self.statusBar().showMessage("AI 调用失败")

    def on_ai_clear(self):
        self._ai_messages.clear()
        self.ai_chat_view.clear()
        self.statusBar().showMessage("对话已清空")

    # ---------- 信号连接 ----------

    def _connect_signals(self):
        self.worker.connected.connect(self.on_connected)
        self.worker.pid_ready.connect(self.on_pid_ready)
        self.worker.status_ready.connect(self.on_status_ready)
        self.worker.fast_ready.connect(self.on_fast_ready)
        self.worker.write_done.connect(self.on_write_done)
        self.worker.backup_done.connect(
            lambda p: self.statusBar().showMessage(f"已自动备份：{p}"))
        self.worker.motor_count_ready.connect(self.on_motor_count)
        self.worker.flash_progress.connect(self.statusBar().showMessage)
        self.worker.flash_done.connect(self.on_flash_done)
        self.worker.tuning_ready.connect(self.on_tuning_ready)
        self.worker.error.connect(self.on_error)
        self.worker.status.connect(self.statusBar().showMessage)
        # AI 助手信号
        self.ai.token.connect(self.on_ai_token)
        self.ai.done.connect(self.on_ai_done)
        self.ai.failed.connect(self.on_ai_failed)
        self.ai_probe_done.connect(self.on_ai_probe)

    def _run_in_thread(self, func, *args):
        """通用后台线程启动器（顺手清理已结束的线程引用，防止列表无限增长）"""
        self._threads = [t for t in self._threads if t.is_alive()]
        thread = threading.Thread(target=func, args=args, daemon=True)
        self._threads.append(thread)
        thread.start()

    # ---------- 串口扫描与连接 ----------

    def refresh_ports(self):
        """扫描并列出所有可用串口"""
        self.statusBar().showMessage("正在扫描串口……")
        self.port_combo.clear()
        ports = list(list_ports.comports())
        if not ports:
            self.port_combo.addItem("（未检测到串口，请插入飞控）", None)
            self.statusBar().showMessage(
                "未检测到串口：请插入飞控 USB 线后点击「刷新」")
            return
        for p in ports:
            self.port_combo.addItem(f"{p.device} - {p.description}", p.device)
        self.statusBar().showMessage(f"扫描完成：发现 {len(ports)} 个串口")

    def on_connect_clicked(self):
        port = self.port_combo.currentData()
        if not port:
            self.statusBar().showMessage("错误：没有可用的串口")
            return
        baudrate = int(self.baud_combo.currentText())
        self.connect_button.setEnabled(False)
        self.refresh_button.setEnabled(False)
        self.statusBar().showMessage(f"正在连接 {port} @ {baudrate}……")
        self._run_in_thread(self.worker.connect_and_query, port, baudrate)

    def on_disconnect_clicked(self):
        # 先停定时器，再停电机（避免轮询线程与停电机命令争用串口）
        self.poll_timer.stop()
        self.fast_timer.stop()
        self._stop_motors_safely()            # 断开前把电机停掉
        self.worker.close_port()
        self._set_disconnected_ui()
        self.statusBar().showMessage("已断开连接，串口已释放")

    # ---------- 实时轮询 ----------

    def _poll_once(self):
        """慢通道定时器触发：读电压/CPU/解锁标志（避免重复启动）"""
        if self._polling or not self.worker.is_connected:
            return
        self._polling = True
        def run():
            try:
                self.worker.poll_status()
            finally:
                self._polling = False
        self._run_in_thread(run)

    def _poll_fast_once(self):
        """快通道定时器触发：读姿态角和 RC 通道（避免重复启动）"""
        if self._polling_fast or not self.worker.is_connected:
            return
        self._polling_fast = True
        def run():
            try:
                self.worker.poll_fast()
            finally:
                self._polling_fast = False
        self._run_in_thread(run)

    # ---------- 信号槽 ----------

    def on_connected(self, info: dict):
        self.firmware_label.setText(info.get("firmware", "未知"))
        self.board_label.setText(info.get("board", "未知"))
        self.motors_label.setText(info.get("motors", "未知"))
        self.disconnect_button.setEnabled(True)
        for btn in (self.pid_reload_btn, self.pid_write_btn,
                    self.pid_backup_btn, self.pid_restore_btn,
                    self.rates_reload_btn, self.filter_reload_btn,
                    self.preset_save_btn, self.preset_apply_btn):
            btn.setEnabled(True)
        self.bb_flash_btn.setEnabled(True)    # 连接后允许从飞控下载黑匣子
        self.poll_timer.start()               # 开始慢通道轮询
        self.fast_timer.start()               # 开始快通道轮询（姿态 10 帧/秒）
        self.statusBar().showMessage("已连接")

    def on_pid_ready(self, names: list, values: list):
        """PID 数据到达：重建表格（连接、写入后、恢复后都会触发）"""
        self._pid_names = names
        self.pid_table.setRowCount(len(names))
        self.pid_table.setVerticalHeaderLabels(names)
        for row, (p, i_val, d) in enumerate(values):
            for col, v in enumerate((p, i_val, d)):
                self.pid_table.setItem(row, col, QTableWidgetItem(str(v)))
        self.connect_button.setEnabled(True)
        self.refresh_button.setEnabled(True)

    def on_status_ready(self, data: dict):
        """慢通道数据到达：更新电源和飞控状态区域"""
        self._polling = False
        self.voltage_label.setText(f"{data.get('voltage', 0):.2f} V")
        self.amps_label.setText(f"{data.get('amps', 0):.2f} A")
        self.mah_label.setText(f"{data.get('mah', 0)} mAh")
        self.rssi_label.setText(f"{data.get('rssi', 0)} %")
        self.cpu_label.setText(f"{data.get('cpu_load', 0)} %")
        self.cycle_label.setText(f"{data.get('cycle_us', 0)} µs")

        disabled = data.get("arming_disabled", [])
        self.arming_label.setText("、".join(disabled) if disabled
                                  else "无（可以解锁）")

    def on_fast_ready(self, data: dict):
        """快通道数据到达：更新人工地平线和接收机通道"""
        self._polling_fast = False
        attitude = data.get("attitude")
        if attitude:
            roll, pitch, yaw = attitude
            self.horizon.set_attitude(roll, pitch)
            self.attitude_label.setText(
                f"横滚 {roll:.1f}° ｜ 俯仰 {pitch:.1f}° ｜ 航向 {yaw:.0f}°")
        rc = data.get("rc", [])
        if rc:
            self._update_rc_display(rc)

    def on_write_done(self, message: str):
        self.statusBar().showMessage(message.splitlines()[0])
        self.refresh_preset_list()            # 保存预设后刷新列表
        QMessageBox.information(self, "ApexFlight", message)

    def on_motor_count(self, count: int):
        """根据电机通道数动态生成滑块"""
        self._motor_count = count
        self.motor_area.setTitle(f"电机输出（{count} 个通道）")
        # 清空旧滑块
        while self.motor_layout.count():
            item = self.motor_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self._motor_sliders = []
        for i in range(count):
            label = QLabel(f"电机 {i + 1}: 0")
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(0, 1000)          # 0=停转，1000 对应输出 2000
            slider.setEnabled(False)
            slider.valueChanged.connect(
                lambda v, idx=i, lab=label: self._on_motor_slider(idx, v, lab))
            self.motor_layout.addWidget(label, i, 0)
            self.motor_layout.addWidget(slider, i, 1)
            self._motor_sliders.append((label, slider))
        self._update_motor_lock()

    def on_error(self, message: str):
        self._polling = False
        self._polling_fast = False
        # 闪存下载失败/被取消时：恢复按钮文字和轮询定时器
        if self._flash_cancel is not None:
            self._flash_cancel = None
            self.bb_flash_btn.setText("📥 从飞控下载")
            self._resume_polling_after_flash()
        self.statusBar().showMessage(f"错误：{message.splitlines()[0]}")
        self.connect_button.setEnabled(True)
        self.refresh_button.setEnabled(True)

    # ---------- PID 页操作 ----------

    def on_pid_reload(self):
        self.statusBar().showMessage("正在重新读取 PID……")
        def run():
            try:
                names, values = query_pid(self.worker.serial_port)
                self.worker.pid_ready.emit(names, values)
                self.worker.status.emit("PID 已重新读取")
            except MspError as e:
                self.worker.error.emit(str(e))
        self._run_in_thread(run)

    def _read_pid_table(self) -> list:
        """把表格里的数值读出来，返回 [(P, I, D), ...]"""
        values = []
        for row in range(self.pid_table.rowCount()):
            triple = []
            for col in range(3):
                item = self.pid_table.item(row, col)
                try:
                    v = int(item.text()) if item else 0
                except ValueError:
                    raise ValueError(f"第 {row + 1} 行有非数字内容，请检查")
                if not 0 <= v <= 255:
                    raise ValueError(f"第 {row + 1} 行数值 {v} 超出 0~255 范围")
                triple.append(v)
            values.append(tuple(triple))
        return values

    def on_pid_write(self):
        try:
            values = self._read_pid_table()
        except ValueError as e:
            QMessageBox.warning(self, "ApexFlight", str(e))
            return
        reply = QMessageBox.question(
            self, "确认写入",
            "将把表格中的 PID 写入飞控并保存到闪存。\n"
            "写入前会自动备份当前参数到 backups/ 文件夹。\n\n确定继续吗？")
        if reply != QMessageBox.StandardButton.Yes:
            return
        self._run_in_thread(self.worker.write_pids,
                            self._pid_names, values, True)

    def on_pid_backup(self):
        self._run_in_thread(self.worker.backup_now, self._pid_names)

    def on_pid_restore(self):
        BACKUP_DIR.mkdir(exist_ok=True)
        path, _ = QFileDialog.getOpenFileName(
            self, "选择备份文件", str(BACKUP_DIR),
            "ApexFlight 备份 (*.json)")
        if not path:
            return
        reply = QMessageBox.question(
            self, "确认恢复",
            f"将用备份文件覆盖飞控当前 PID：\n{path}\n\n确定继续吗？")
        if reply != QMessageBox.StandardButton.Yes:
            return
        self._run_in_thread(self.worker.restore_pids, path)

    # ---------- 电机测试页操作 ----------

    def _update_motor_lock(self):
        """只有两个安全确认都勾选且已连接，滑块才能用"""
        unlocked = (self.motor_check1.isChecked()
                    and self.motor_check2.isChecked()
                    and self.worker.is_connected)
        for _, slider in self._motor_sliders:
            slider.setEnabled(unlocked)
        self.motor_stop_btn.setEnabled(unlocked)

    def _on_motor_slider(self, index: int, value: int, label: QLabel):
        """滑块变化：更新标签并发送全部电机值"""
        label.setText(f"电机 {index + 1}: {value}")
        values = [s.value() for _, s in self._motor_sliders]
        # 补齐到 8 个通道（协议固定 8 个电机）
        values += [0] * (8 - len(values))
        self._run_in_thread(self.worker.set_motor_values, values[:8])

    def on_motor_stop(self):
        """全部停止：所有滑块归零"""
        for _, slider in self._motor_sliders:
            slider.setValue(0)

    def _stop_motors_safely(self):
        """断开/关闭前尝试把所有电机停掉"""
        if self.worker.is_connected and self._motor_count:
            try:
                set_motors(self.worker.serial_port, [0] * 8)
            except Exception:
                pass

    # ---------- 接收机页 ----------

    def _update_rc_display(self, channels: list):
        """按通道数动态生成/更新接收机通道显示"""
        if len(self._rc_bars) != len(channels):
            # 通道数变化（首次连接）时重建
            while self.rc_layout.count():
                item = self.rc_layout.takeAt(0)
                if item.widget():
                    item.widget().deleteLater()
            self._rc_bars = []
            self.rc_area.setTitle(f"通道（{len(channels)} 个）")
            for i in range(len(channels)):
                name = QLabel(f"CH {i + 1}")
                value = QLabel("0")
                value.setMinimumWidth(50)
                bar = QSlider(Qt.Orientation.Horizontal)
                bar.setRange(800, 2200)
                bar.setEnabled(False)         # 只读显示条
                row = i // 2
                col = (i % 2) * 3
                self.rc_layout.addWidget(name, row, col)
                self.rc_layout.addWidget(bar, row, col + 1)
                self.rc_layout.addWidget(value, row, col + 2)
                self._rc_bars.append((bar, value))
        for (bar, value_label), v in zip(self._rc_bars, channels):
            bar.setValue(max(800, min(2200, v)))
            value_label.setText(str(v))

    # ---------- 断开与关闭 ----------

    def _set_disconnected_ui(self):
        self.connect_button.setEnabled(True)
        self.refresh_button.setEnabled(True)
        self.disconnect_button.setEnabled(False)
        for btn in (self.pid_reload_btn, self.pid_write_btn,
                    self.pid_backup_btn, self.pid_restore_btn,
                    self.rates_reload_btn, self.rates_write_btn,
                    self.filter_reload_btn, self.filter_write_btn,
                    self.preset_save_btn, self.preset_apply_btn):
            btn.setEnabled(False)
        self._rc_raw = None
        self._filter_raw = None
        self.bb_flash_btn.setEnabled(False)
        self.firmware_label.setText("未连接")
        self.board_label.setText("未连接")
        self.motors_label.setText("未连接")
        for label in (self.voltage_label, self.amps_label, self.mah_label,
                      self.rssi_label, self.cpu_label, self.cycle_label):
            label.setText("—")
        self.arming_label.setText("—")
        self.pid_table.setRowCount(0)
        self._update_motor_lock()

    def closeEvent(self, event):
        """关闭窗口：先停电机，再释放串口"""
        self.poll_timer.stop()
        self.fast_timer.stop()
        self._stop_motors_safely()
        self.worker.close_port()
        event.accept()


# ============================================================
# 第七部分：崩溃日志 + 程序入口
# ============================================================
# 任何未被捕获的异常（包括界面回调、后台线程）都会：
#   1. 带时间戳追加写入 logs/crash.log（用户可把这个文件发给开发者）
#   2. 弹出错误对话框提示，而不是让窗口无声消失
# faulthandler 还会捕获 C 层面的崩溃（段错误等），写入同一文件。

CRASH_LOG = LOGS_DIR / "crash.log"
_CRASH_FH = None                # faulthandler 的文件句柄（需保持存活）


def _write_crash_log(title: str, text: str):
    """把崩溃信息追加到日志文件（带时间戳和分隔线）"""
    try:
        LOGS_DIR.mkdir(exist_ok=True)
        with open(CRASH_LOG, "a", encoding="utf-8") as f:
            f.write(f"\n{'=' * 60}\n")
            f.write(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] {title}\n")
            f.write(text)
            if not text.endswith("\n"):
                f.write("\n")
    except OSError:
        pass


def _excepthook(exc_type, exc_value, exc_tb):
    """主线程/界面回调未捕获异常：记录 + 弹窗，程序继续运行"""
    import traceback as _tb
    text = "".join(_tb.format_exception(exc_type, exc_value, exc_tb))
    _write_crash_log("未捕获的异常（界面/主线程）", text)
    try:
        QMessageBox.critical(
            None, "ApexFlight 发生错误",
            f"程序遇到一个未处理的错误（详情已写入 logs/crash.log）：\n\n"
            f"{exc_type.__name__}: {exc_value}\n\n"
            "程序会继续运行，如反复出现请把 crash.log 发给开发者。")
    except Exception:
        pass


def _thread_excepthook(args):
    """后台线程未捕获异常：只记录日志（线程无法弹窗）"""
    import traceback as _tb
    text = "".join(_tb.format_exception(
        args.exc_type, args.exc_value, args.exc_tb))
    _write_crash_log(f"未捕获的异常（线程 {args.thread.name}）", text)


def install_crash_logging():
    """安装全局崩溃日志钩子（在创建 QApplication 之前调用）"""
    global _CRASH_FH
    sys.excepthook = _excepthook
    threading.excepthook = _thread_excepthook
    try:
        import faulthandler
        LOGS_DIR.mkdir(exist_ok=True)
        _CRASH_FH = open(CRASH_LOG, "a", encoding="utf-8")
        faulthandler.enable(file=_CRASH_FH)   # 段错误等硬崩溃也留痕
    except Exception:
        pass


def main():
    install_crash_logging()
    app = QApplication(sys.argv)
    app.setApplicationName("ApexFlight")
    if ICON_PATH.exists():
        app.setWindowIcon(QIcon(str(ICON_PATH)))
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
