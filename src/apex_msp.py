# -*- coding: utf-8 -*-
"""ApexFlight - MSP 协议层（MultiWii Serial Protocol v1/v2 编解码）"""

import threading
import time

import serial

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
                timeout: float = 2.0, retries: int = 1) -> bytes:
    """发送一条 MSP 命令并等待响应（线程安全，全程持锁）。

    retries：瞬态错误（超时、帧损坏）自动重试次数，默认 1（即共尝试 2 次）。
    USB 转串口在飞控刚上电、总线繁忙时会偶发丢字节，重试一次可显著减少
    偶发"读取超时"误报。飞控明确拒绝的命令（固件不支持）不重试。
    """
    with _MSP_LOCK:
        last_err: MspError | None = None
        for attempt in range(retries + 1):
            try:
                ser.reset_input_buffer()          # 丢弃缓冲区里的旧数据
                ser.write(build_msp_request(cmd, payload))
                ser.flush()
                return read_msp_response(ser, cmd, timeout)
            except MspError as e:
                if str(e).startswith("飞控拒绝了命令"):
                    raise                         # 确定性失败，重试无意义
                last_err = e
                if attempt < retries:
                    time.sleep(0.05)
        raise last_err


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
                 timeout: float = 5.0, retries: int = 1) -> bytes:
    """发送一条 MSP v2 命令并等待响应（与 v1 共用同一把串口锁）。
    瞬态错误自动重试（规则同 msp_request）。"""
    with _MSP_LOCK:
        last_err: MspError | None = None
        for attempt in range(retries + 1):
            try:
                ser.reset_input_buffer()
                ser.write(build_msp2_request(cmd, payload))
                ser.flush()
                return read_msp2_response(ser, cmd, timeout)
            except MspError as e:
                if str(e).startswith("飞控拒绝了命令"):
                    raise
                last_err = e
                if attempt < retries:
                    time.sleep(0.05)
        raise last_err


def u16(data: bytes, offset: int) -> int:
    """从字节串中按小端序读取 2 字节无符号整数"""
    return data[offset] | (data[offset + 1] << 8)


def s16(data: bytes, offset: int) -> int:
    """从字节串中按小端序读取 2 字节有符号整数"""
    value = u16(data, offset)
    return value - 65536 if value >= 32768 else value


