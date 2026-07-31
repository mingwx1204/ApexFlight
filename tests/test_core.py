# -*- coding: utf-8 -*-
"""
ApexFlight 核心逻辑单元测试（不依赖飞控硬件、不需要显示器）。
运行：  python tests/test_core.py
覆盖：MSP 编解码、CRC8、Rates 解析/修改、BF 曲线公式、
      滤波器解析/修改、黑匣子结构化分析。
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import main as m


def test_msp_v1_roundtrip():
    """MSP v1 请求帧：帧头/长度/命令/校验和正确"""
    frame = m.build_msp_request(112, b"\x01\x02")
    assert frame[:3] == b"$M<"
    assert frame[3] == 2 and frame[4] == 112
    assert frame[-1] == (2 ^ 112 ^ 0x01 ^ 0x02)


def test_crc8_dvb_s2():
    """CRC8-DVB-S2：全零输入得 0；回归值锁定"""
    assert m.crc8_dvb_s2(b"") == 0
    assert m.crc8_dvb_s2(bytes(8)) == 0
    assert m.crc8_dvb_s2(bytes([0, 111, 0, 0])) == 147  # 回归值


def test_msp2_request_structure():
    """MSP v2 请求帧：帧头/标志/命令小端/长度小端/CRC 位置"""
    frame = m.build_msp2_request(71, b"\x00" * 6)
    assert frame[:3] == b"$X<"
    assert frame[3] == 0                            # 标志位
    assert int.from_bytes(frame[4:6], "little") == 71
    assert int.from_bytes(frame[6:8], "little") == 6
    assert frame[-1] == m.crc8_dvb_s2(frame[3:-1])


def test_rc_tuning_parse_real_bytes():
    """用真机实测的 23 字节验证解析（DAKEFPVH743 / BF 4.5.2）"""
    rc = bytes([150, 0, 70, 70, 70, 0, 50, 0, 0, 0, 0, 150,
                150, 0, 0, 100, 206, 7, 206, 7, 206, 7, 0])
    p = m.parse_rc_tuning(rc)
    assert p["rc_rate"] == [1.5, 1.5, 1.5]
    assert p["rate"] == [0.7, 0.7, 0.7]
    assert p["expo"] == [0.0, 0.0, 0.0]
    assert p["thr_mid"] == 0.5 and p["thr_expo"] == 0.0
    assert p["thr_limit_pct"] == 100
    assert p["rate_limit"] == [1998, 1998, 1998]
    assert p["rates_type"] == 0


def test_rc_tuning_read_modify_write():
    """修改只动目标字节，其余原样保留"""
    rc = bytes([150, 0, 70, 70, 70, 0, 50, 0, 0, 0, 0, 150,
                150, 0, 0, 100, 206, 7, 206, 7, 206, 7, 0])
    raw = bytearray(rc)
    m.set_rc_value(raw, "rc_rate", 0, 1.80)
    assert raw[0] == 180 and raw[12] == 150        # 只动横滚
    m.set_rc_value(raw, "expo", 2, 0.35)
    assert raw[10] == 35 and raw[1] == 0           # 偏航 expo 独立
    m.set_rc_value(raw, "rate", 1, 0.85)
    assert raw[3] == 85
    raw[6] = 60                                    # 油门中点直写
    p = m.parse_rc_tuning(bytes(raw))
    assert abs(p["thr_mid"] - 0.6) < 1e-9


def test_bf_rate_curve_known_values():
    """BF 经典公式对照：rcRate 1.5 / Rate 0.7 → 满杆 1000°/s"""
    v = m.bf_rate_curve(1.0, 1.5, 0.7, 0.0)
    assert abs(v - 1000.0) < 1.0
    # expo 不影响满杆终点
    assert abs(m.bf_rate_curve(1.0, 1.5, 0.7, 0.5) - 1000.0) < 1.0
    # 半杆：expo 越大中位越平
    flat = m.bf_rate_curve(0.5, 1.5, 0.7, 0.6)
    plain = m.bf_rate_curve(0.5, 1.5, 0.7, 0.0)
    assert flat < plain
    # rcRate > 2.0 的固件增益
    assert m.bf_rate_curve(1.0, 2.2, 0.0, 0.0) > 2.2 * 200


def test_bf_throttle_curve():
    """油门曲线：过 (0,0)、(mid,mid)、(1,1) 三个锚点"""
    for mid, expo in [(0.5, 0.0), (0.5, 0.8), (0.3, 0.5)]:
        assert abs(m.bf_throttle_curve(0.0, mid, expo)) < 1e-6
        assert abs(m.bf_throttle_curve(mid, mid, expo) - mid) < 1e-6
        assert abs(m.bf_throttle_curve(1.0, mid, expo) - 1.0) < 1e-6
    # expo=0 时是直线
    assert abs(m.bf_throttle_curve(0.4, 0.5, 0.0) - 0.4) < 1e-6


def test_filter_parse_real_bytes():
    """用真机实测的 49 字节验证滤波器解析"""
    filt = bytes([250, 75, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 250, 0, 244, 1, 0, 0, 150, 0, 0, 250, 0, 244,
                  1, 75, 0, 150, 0, 0, 0, 244, 1, 100, 0, 3, 100, 88, 2,
                  5, 1])
    f = m.parse_filter_config(filt)
    assert f["gyro_lpf1_hz"] == 250 and f["gyro_lpf2_hz"] == 500
    assert f["dterm_lpf1_hz"] == 75 and f["dterm_lpf2_hz"] == 150
    assert f["yaw_lpf_hz"] == 100
    assert f["gyro_dyn_min"] == 250 and f["gyro_dyn_max"] == 500
    assert f["rpm_harmonics"] == 3 and f["rpm_min_hz"] == 100
    assert f["notch_max"] == 600 and f["notch_count"] == 1


def test_filter_read_modify_write():
    filt = bytes([250, 75, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 250, 0, 244, 1, 0, 0, 150, 0, 0, 250, 0, 244,
                  1, 75, 0, 150, 0, 0, 0, 244, 1, 100, 0, 3, 100, 88, 2,
                  5, 1])
    raw = bytearray(filt)
    m.set_filter_value(raw, "gyro_lpf1_hz", 300)
    assert raw[20] == 44 and raw[21] == 1 and raw[0] == 44  # u16+旧副本
    m.set_filter_value(raw, "rpm_harmonics", 2)
    assert raw[43] == 2 and raw[41] == 100 and raw[42] == 0  # 相邻不动
    m.set_filter_value(raw, "gyro_notch1_hz", 400)
    assert raw[5] == 144 and raw[6] == 1
    f = m.parse_filter_config(bytes(raw))
    assert f["gyro_notch1_hz"] == 400


def test_analyze_blackbox_stats():
    """结构化分析：合成一段含 200Hz 噪声 + 阶跃跟踪的日志"""
    import math
    n = 4000                                       # 8 秒 @500Hz
    time_axis = [i / 500 for i in range(n)]
    gyro, sp = [], []
    for i in range(n):
        t = i / 500
        target = 500 if 2.0 <= t < 4.0 else 0      # 阶跃设定值
        follow = target * (1 - math.exp(-max(0.0, t - 2.0) * 10)) \
            if t >= 2.0 else 0
        noise = 30 * math.sin(2 * math.pi * 200 * t)   # 200Hz 噪声
        gyro.append(follow + noise)
        sp.append(target)
    data = {"gyroADC[0]": gyro, "setpoint[0]": sp,
            "motor[0]": [1500] * n, "vbatLatest": [1680] * n}
    stats = m.analyze_blackbox_stats(time_axis, data,
                                     ["gyroADC[0]", "setpoint[0]",
                                      "motor[0]", "vbatLatest"])
    assert stats["时长(秒)"] > 7.9
    assert stats["采样率(Hz)"] == 500
    assert 200 in stats["陀螺仪·横滚 噪声峰(Hz)"]
    assert stats["跟踪滞后·横滚(ms)"] >= 0
    assert stats["电机饱和时间占比(%)"] == 0
    assert stats["最低电压(V)"] == 16.8


def test_preset_snapshot_roundtrip(tmp: Path = None):
    """调参快照 JSON 保存/读取往返"""
    rc = bytes(range(23))
    filt = bytes(range(49))
    snap = m.tuning_snapshot({"firmware": "BF 4.5.2", "board": "TEST"},
                             ["Roll"], [(1, 2, 3)], rc, filt, name="单测")
    directory = Path(tmp) if tmp else m.PROJECT_ROOT / "presets"
    path = m.save_preset_file(directory, "_unittest_preset.json", snap)
    loaded = m.load_preset_file(path)
    assert loaded["rc_tuning_raw"] == list(rc)
    assert loaded["filter_raw"] == list(filt)
    assert loaded["pid_values"] == [[1, 2, 3]]
    path.unlink()


def test_classify_log_type():
    """飞行/地面空转日志判别"""
    n = 500
    time_axis = [i * 2000 for i in range(n)]  # 500Hz, 1 秒

    # 地面空转：电机恒定 1400、电流 1.5A、陀螺仪小噪声、设定点全 0
    ground = {
        "motor[0]": [1400] * n, "motor[1]": [1400] * n,
        "amperageLatest": [150] * n,  # 0.01A 单位 → 1.5A
        "gyroADC[0]": [5 * ((i % 7) - 3) for i in range(n)],
        "gyroADC[1]": [3 * ((i % 5) - 2) for i in range(n)],
        "setpoint[0]": [0] * n, "setpoint[1]": [0] * n, "setpoint[2]": [0] * n,
    }
    r = m.classify_log_type(time_axis, ground,
                            ["motor[0]", "motor[1]", "amperageLatest",
                             "gyroADC[0]", "gyroADC[1]",
                             "setpoint[0]", "setpoint[1]", "setpoint[2]"])
    assert "空转" in r["verdict"] or "静止" in r["verdict"], r
    assert r["confidence"] >= 60
    assert r["features"]["平均电流(A)"] < 3

    # 正常飞行：电流 15A、陀螺仪大幅活动、设定点持续变化
    import math as _math
    flight = {
        "motor[0]": [1500 + int(200 * _math.sin(i / 8)) for i in range(n)],
        "motor[1]": [1450 + int(180 * _math.cos(i / 9)) for i in range(n)],
        "amperageLatest": [1500 + 100 * (i % 4) for i in range(n)],  # ~15A
        "gyroADC[0]": [int(300 * _math.sin(i / 5)) for i in range(n)],
        "gyroADC[1]": [int(250 * _math.cos(i / 6)) for i in range(n)],
        "setpoint[0]": [int(200 * _math.sin(i / 7)) for i in range(n)],
        "setpoint[1]": [int(150 * _math.cos(i / 8)) for i in range(n)],
        "setpoint[2]": [30] * n,
    }
    r2 = m.classify_log_type(time_axis, flight,
                             ["motor[0]", "motor[1]", "amperageLatest",
                              "gyroADC[0]", "gyroADC[1]",
                              "setpoint[0]", "setpoint[1]", "setpoint[2]"])
    assert "正常飞行" in r2["verdict"], r2
    assert r2["confidence"] >= 60
    assert r2["features"]["平均电流(A)"] > 10

    # 列缺失时（无电流、无电机）不应抛异常
    r3 = m.classify_log_type(time_axis,
                             {"gyroADC[0]": [0] * n, "setpoint[0]": [0] * n},
                             ["gyroADC[0]", "setpoint[0]"])
    assert r3["verdict"]


if __name__ == "__main__":
    tests = [(name, fn) for name, fn in list(globals().items())
             if name.startswith("test_") and callable(fn)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"  ✅ {name}")
        except AssertionError as e:
            failed += 1
            print(f"  ❌ {name}: {e}")
        except Exception as e:
            failed += 1
            print(f"  💥 {name}: {type(e).__name__}: {e}")
    print(f"\n{len(tests) - failed}/{len(tests)} 通过")
    sys.exit(1 if failed else 0)
