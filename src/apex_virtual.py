# -*- coding: utf-8 -*-
"""ApexFlight 虚拟飞控（v0.93）：不插真机也能体验全部功能。

实现思路：VirtualSerial 模拟 pyserial 的最小接口
（write / read / flush / reset_input_buffer / close），
内部解析 MSP v1 请求帧并返回内置的仿真响应——
上层所有查询/写入逻辑零改动，与真实飞控走同一条代码路径。

仿真内容：
- 固件 Betaflight 4.5.2（BTFL），板型 APEXVIRTUAL（STM32H743，代号 VIRT）
- 姿态角随时间缓慢摆动（仪表盘人工地平线会动起来）
- RC 通道 16 路中位、油门轻微波动；8 个电机通道
- 10 组默认 PID、23 字节 Rates、49 字节滤波器默认值
- 写入类命令（SET_PID / SET_RC_TUNING / SET_FILTER_CONFIG / SET_MOTOR）
  存进内存并 ACK，EEPROM 保存模拟 100ms 延迟
- 无板载闪存（DATAFLASH 命令一律拒绝，黑匣子下载页优雅降级）
"""

import math
import time

from apex_msp import (
    MSP_API_VERSION, MSP_FC_VARIANT, MSP_FC_VERSION, MSP_BOARD_INFO,
    MSP_IDENT, MSP_MOTOR, MSP_RC, MSP_ATTITUDE, MSP_ANALOG, MSP_PID,
    MSP_STATUS_EX, MSP_SET_PID, MSP_SET_MOTOR, MSP_EEPROM_WRITE,
    MSP_DATAFLASH_SUMMARY, MSP_DATAFLASH_READ, MSP_DATAFLASH_ERASE,
    MSP_FILTER_CONFIG, MSP_SET_FILTER_CONFIG,
    MSP_RC_TUNING, MSP_SET_RC_TUNING,
)

# 串口下拉框里虚拟连接的 userData 值
VIRTUAL_PORT = "VIRTUAL"
VIRTUAL_PORT_LABEL = "🔌 虚拟连接（无飞控体验全部功能）"


def _u16(v: int) -> bytes:
    return int(v).to_bytes(2, "little", signed=False)


def _s16(v: int) -> bytes:
    return int(v).to_bytes(2, "little", signed=True)


def _u32(v: int) -> bytes:
    return int(v).to_bytes(4, "little", signed=False)


# Betaflight 4.5 风格的默认 PID（10 组：Roll/Pitch/Yaw/Alt/Pos/PosR/NavR/Level/Mag/Vel）
_DEFAULT_PID = [
    (45, 80, 40), (47, 84, 46), (45, 80, 0),
    (50, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0),
    (50, 50, 75), (40, 0, 0), (55, 55, 15),
]


def _default_rc_tuning() -> bytearray:
    """23 字节 Rates 默认值（布局见 apex_fc.parse_rc_tuning）"""
    raw = bytearray(23)
    raw[0] = 110                       # rcRate roll = 1.10
    raw[1] = 50                        # expo roll = 0.50
    raw[2], raw[3], raw[4] = 67, 67, 62    # super rate r/p/y
    raw[6] = 50                        # 油门中点 0.50
    raw[8:10] = _u16(1350)             # TPA 断点
    raw[10] = 50                       # expo yaw
    raw[11] = 110                      # rcRate yaw
    raw[12] = 110                      # rcRate pitch
    raw[13] = 50                       # expo pitch
    raw[14] = 0                        # 油门限幅类型
    raw[15] = 100                      # 油门限幅百分比
    raw[16:18] = _u16(1998)            # 角速度上限 roll
    raw[18:20] = _u16(1998)            # pitch
    raw[20:22] = _u16(1998)            # yaw
    raw[22] = 0                        # Rates 类型：Betaflight 经典
    return raw


def _default_filter_config() -> bytearray:
    """49 字节滤波器默认值（偏移见 apex_fc.FILTER_FIELDS）"""
    raw = bytearray(49)

    def put16(off: int, v: int):
        raw[off:off + 2] = _u16(v)

    put16(1, 150)                      # dterm_lpf1_hz
    put16(3, 0)                        # yaw_lpf_hz（关闭）
    put16(5, 0); put16(7, 0)           # gyro notch1
    put16(9, 0); put16(11, 0)          # dterm notch
    put16(13, 0); put16(15, 0)         # gyro notch2
    raw[17] = 1                        # dterm_lpf1_type = Biquad
    put16(20, 250)                     # gyro_lpf1_hz
    put16(22, 500)                     # gyro_lpf2_hz
    raw[24] = 0                        # gyro_lpf1_type = PT1
    raw[25] = 1                        # gyro_lpf2_type = Biquad
    put16(26, 300)                     # dterm_lpf2_hz
    raw[28] = 1                        # dterm_lpf2_type = Biquad
    put16(29, 250); put16(31, 500)     # gyro 动态低通 下限/上限
    put16(33, 150); put16(35, 250)     # D 项动态低通 下限/上限
    put16(39, 250)                     # 动态陷波 Q
    put16(41, 100); put16(45, 600)     # 动态陷波最低/最高频率
    raw[43] = 3                        # RPM 谐波数量
    raw[44] = 100                      # RPM 最低频率
    raw[47] = 5                        # D 项动态 expo
    raw[48] = 3                        # 动态陷波数量
    return raw


class VirtualSerial:
    """模拟 pyserial 的虚拟飞控串口。线程安全由上层 _MSP_LOCK 保证。"""

    def __init__(self):
        self.is_open = True
        self.timeout = 0.2
        self._rx = bytearray()         # 待读取的响应字节
        self._tx = bytearray()         # 累积的请求字节
        self._t0 = time.time()
        # 可写的仿真状态
        self._pid = [list(t) for t in _DEFAULT_PID]
        self._rc_tuning = _default_rc_tuning()
        self._filter = _default_filter_config()
        self._motors = [1000] * 8

    # ---------- pyserial 接口 ----------

    def write(self, data: bytes) -> int:
        self._tx += data
        self._drain_requests()
        return len(data)

    def read(self, n: int = 1) -> bytes:
        out = bytes(self._rx[:n])
        del self._rx[:n]
        return out

    def flush(self):
        pass

    def reset_input_buffer(self):
        self._rx.clear()

    def reset_output_buffer(self):
        self._tx.clear()

    @property
    def in_waiting(self) -> int:
        return len(self._rx)

    def close(self):
        self.is_open = False

    # ---------- MSP 请求解析 ----------

    def _drain_requests(self):
        """从发送缓冲里拆出完整 MSP v1 请求帧并逐帧响应"""
        while True:
            # 找帧头 "$M<"
            idx = self._tx.find(b"$M<")
            if idx < 0:
                self._tx.clear()
                return
            if idx > 0:
                del self._tx[:idx]
            if len(self._tx) < 5:
                return                    # 帧头还不完整
            size = self._tx[3]
            cmd = self._tx[4]
            total = 6 + size              # 头3 + 长度1 + 命令1 + 数据N + 校验1
            if len(self._tx) < total:
                return                    # 数据还没收齐
            payload = bytes(self._tx[5:5 + size])
            del self._tx[:total]
            self._handle(cmd, payload)

    # ---------- MSP 命令响应 ----------

    def _respond(self, cmd: int, payload: bytes = b""):
        """构造 MSP v1 成功响应帧，放入接收缓冲"""
        checksum = len(payload) ^ cmd
        for b in payload:
            checksum ^= b
        self._rx += (b"$M>" + bytes([len(payload), cmd])
                     + payload + bytes([checksum]))

    def _reject(self, cmd: int):
        """构造 MSP v1 拒绝帧（固件不支持该命令）"""
        self._rx += b"$M!" + bytes([0, cmd, cmd ^ 0])

    def _handle(self, cmd: int, payload: bytes):
        t = time.time() - self._t0

        if cmd == MSP_API_VERSION:
            self._respond(cmd, bytes([0, 1, 45]))
        elif cmd == MSP_FC_VARIANT:
            self._respond(cmd, b"BTFL")
        elif cmd == MSP_FC_VERSION:
            self._respond(cmd, bytes([4, 5, 2]))
        elif cmd == MSP_BOARD_INFO:
            mcu = b"STM32H743"
            board = b"APEXVIRTUAL"
            data = (b"VIRT" + bytes(4)
                    + bytes([len(mcu)]) + mcu
                    + bytes([len(board)]) + board)
            self._respond(cmd, data)
        elif cmd == MSP_IDENT:
            self._reject(cmd)             # 让上层走 MSP_MOTOR 数电机
        elif cmd == MSP_MOTOR:
            self._respond(cmd, b"".join(_u16(v) for v in self._motors))
        elif cmd == MSP_RC:
            # 16 通道：油门轻微波动，其余中位，AUX1 高位（已解锁开关演示）
            thr = 1500 + int(180 * math.sin(t / 2.7))
            chans = [1500, 1500, thr, 1500,
                     1800, 1000, 1500, 1500] + [1500] * 8
            self._respond(cmd, b"".join(_u16(v) for v in chans))
        elif cmd == MSP_ATTITUDE:
            roll = int(180 * math.sin(t / 2.3))          # ±18°，0.1° 单位
            pitch = int(120 * math.sin(t / 3.1 + 1.0))   # ±12°
            yaw = int((t * 8) % 360)                     # 缓慢转圈
            self._respond(cmd, _s16(roll) + _s16(pitch) + _s16(yaw))
        elif cmd == MSP_ANALOG:
            vbat = 1675 - int(t / 3) % 20                # 电压缓慢下降
            data = (bytes([168]) + _u16(350 + int(t))    # 老格式电压 + mAh
                    + _u16(950) + _u16(420)              # RSSI + 电流 4.20A
                    + _u16(vbat))                        # 新格式电压
            self._respond(cmd, data)
        elif cmd == MSP_PID:
            flat = bytes(v for triple in self._pid for v in triple)
            self._respond(cmd, flat)
        elif cmd == MSP_STATUS_EX:
            # 循环125us / CPU 18% / 解锁禁用=油门过高+解锁开关不安全
            flags = (1 << 7) | (1 << 25)
            data = (_u16(125) + _u16(0) + _u16(0b1111)
                    + _u32(0) + bytes([0])
                    + _u16(18) + bytes([2]) + _u32(flags))
            self._respond(cmd, data)
        elif cmd == MSP_RC_TUNING:
            self._respond(cmd, bytes(self._rc_tuning))
        elif cmd == MSP_FILTER_CONFIG:
            self._respond(cmd, bytes(self._filter))
        elif cmd == MSP_SET_PID:
            vals = list(payload)
            self._pid = [vals[i:i + 3] for i in range(0, len(vals) - 2, 3)]
            self._respond(cmd)
        elif cmd == MSP_SET_RC_TUNING:
            self._rc_tuning = bytearray(payload)
            self._respond(cmd)
        elif cmd == MSP_SET_FILTER_CONFIG:
            self._filter = bytearray(payload)
            self._respond(cmd)
        elif cmd == MSP_SET_MOTOR:
            self._motors = [int.from_bytes(payload[i:i + 2], "little")
                            for i in range(0, min(len(payload), 16), 2)]
            self._respond(cmd)
        elif cmd == MSP_EEPROM_WRITE:
            time.sleep(0.1)               # 模拟写闪存耗时
            self._respond(cmd)
        elif cmd in (MSP_DATAFLASH_SUMMARY, MSP_DATAFLASH_READ,
                     MSP_DATAFLASH_ERASE):
            self._reject(cmd)             # 虚拟飞控没有板载闪存
        else:
            self._reject(cmd)             # 未实现的命令一律拒绝
