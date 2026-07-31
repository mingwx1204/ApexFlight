# -*- coding: utf-8 -*-
"""v0.98 扫频引擎验证：用【已知传递函数的被控对象】仿真，检验辨识精度。

- 对象 A：二阶欠阻尼 wn=2π·40Hz ζ=0.55 + 2ms 延迟 → 理论 -3dB 带宽 ≈48.5Hz
- 对象 B：wn=2π·200Hz ζ=0.15 → 理论谐振峰 ≈195Hz / +10.5dB
- 输入：对数扫频 chirp（2→250Hz，幅度 300°/s），加测量噪声
- 断言：辨识带宽在理论值 ±15% 内；相干性 >0.9；谐振峰定位 ±10%
- 另测：演示日志端到端流程 + 规则引擎触发

运行：python tests/test_sweep.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import numpy as np                              # noqa: E402
import apex_sweep                               # noqa: E402
from apex_blackbox import generate_demo_log     # noqa: E402

FS = 2000.0


def make_chirp(t, f0=2.0, f1=250.0, amp=300.0):
    """对数扫频信号（与 BF Chirp 同型）"""
    T = t[-1]
    k = math.log(f1 / f0)
    phase = 2 * math.pi * f0 * T / k * (np.exp(k * t / T) - 1)
    return amp * np.sin(phase)


def simulate_plant(u, fs, wn_hz, zeta, delay_s=0.002, noise=3.0, seed=7):
    """二阶欠阻尼 + 纯延迟对象的欧拉仿真（已知真值）"""
    wn = 2 * math.pi * wn_hz
    T = 1.0 / fs
    d = int(delay_s * fs)
    x1 = x2 = 0.0
    y = np.zeros_like(u)
    rng = np.random.default_rng(seed)
    for i in range(len(u)):
        ud = u[i - d] if i >= d else 0.0
        x1 += T * x2
        x2 += T * (wn * wn * (ud - x1) - 2 * zeta * wn * x2)
        y[i] = x1
    y = y + rng.normal(0, noise, len(y))
    return y


def theoretical_bw_2nd(wn_hz, zeta):
    """二阶低通 -3dB 带宽解析解"""
    z2 = 4 * zeta * zeta
    r2 = (2 - z2 + math.sqrt((z2 - 2) ** 2 + 4)) / 2
    return wn_hz * math.sqrt(r2)


def test_bandwidth_identification():
    t = np.arange(0, 12.0, 1.0 / FS)
    u = make_chirp(t)
    y = simulate_plant(u, FS, wn_hz=40.0, zeta=0.55)
    m = apex_sweep.analyze_axis(u, y, FS)
    expect = theoretical_bw_2nd(40.0, 0.55)          # ≈48.5Hz
    got = m["bandwidth_hz"]
    assert got is not None, "带宽未识别"
    err = abs(got - expect) / expect
    assert err < 0.15, f"带宽辨识偏差 {err:.0%}（{got} vs {expect:.1f}Hz）"
    assert m["coherence"] > 0.9, f"相干性过低 {m['coherence']}"
    assert m["sensitivity_peak_db"] is not None
    assert m["phase_margin_deg"] is not None
    print(f"  ✅ 带宽辨识 {got}Hz ≈ 理论 {expect:.1f}Hz（误差 {err:.0%}），"
          f"相干性 {m['coherence']:.0%}，PM {m['phase_margin_deg']}°")


def test_resonance_detection():
    # 200Hz 对象用 8kHz 采样仿真（欧拉法要求 T·wn 远小于 1）
    fs = 8000.0
    t = np.arange(0, 12.0, 1.0 / fs)
    u = make_chirp(t, f0=20.0, f1=800.0)
    y = simulate_plant(u, fs, wn_hz=200.0, zeta=0.15, delay_s=0.0)
    m = apex_sweep.analyze_axis(u, y, fs)
    res = m["resonances"]
    assert res, "未检出谐振峰"
    f_r, m_r = res[0]
    assert 180 <= f_r <= 220, f"谐振峰定位偏差：{f_r}Hz（理论≈195Hz）"
    print(f"  ✅ 谐振峰检出 {f_r}Hz / +{m_r}dB（理论≈195Hz / +10.5dB）")


def test_demo_log_end_to_end():
    path = generate_demo_log()
    result = apex_sweep.analyze_log(path)
    assert "axes" in result and "suggestions" in result
    roll = result["axes"].get("横滚", {})
    # 演示日志：指数跟随 τ=1/8s → 带宽约 1.3Hz
    bw = roll.get("bandwidth_hz")
    assert bw is not None and bw < 5, f"演示日志带宽异常：{bw}"
    print(f"  ✅ 演示日志端到端：横滚带宽 {bw}Hz（一阶系统理论≈1.3Hz），"
          f"建议 {len(result['suggestions'])} 条")


def test_recommend_rules():
    # 构造失衡指标：俯仰带宽只有横滚一半 → 规则 1 必须触发
    fake = {
        "横滚": {"bandwidth_hz": 40.0, "coherence": 0.95,
                 "phase_margin_deg": 80.0, "sensitivity_peak_db": 3.0,
                 "sensitivity_peak_hz": 150.0, "resonances": [],
                 "plateau_db": 0.0},
        "俯仰": {"bandwidth_hz": 20.0, "coherence": 0.93,
                 "phase_margin_deg": 35.0, "sensitivity_peak_db": 7.5,
                 "sensitivity_peak_hz": 210.0, "resonances": [(210.0, 8.2)],
                 "plateau_db": -2.0},
    }
    sug = apex_sweep.recommend(fake)
    rules = {s["param"] for s in sug}
    assert "PID" in rules, "带宽失衡建议缺失"
    assert "D 项 / 阻尼" in rules, "低相位裕度建议缺失"
    assert "滤波器" in rules, "灵敏度峰值建议缺失"
    assert "陷波滤波" in rules, "谐振峰建议缺失"
    assert "I 项 / 跟踪" in rules, "低频跟踪建议缺失"
    pid_s = next(s for s in sug if s["param"] == "PID")
    assert pid_s["axis"] == "俯仰" and pid_s["level"] == "action"
    # 低相干性 → 只给重飞建议
    bad = {a: dict(m, coherence=0.5) for a, m in fake.items()}
    sug2 = apex_sweep.recommend(bad)
    assert len(sug2) == 1 and sug2[0]["param"] == "数据质量"
    print(f"  ✅ 规则引擎：{len(sug)} 条建议全部按预期触发，质量门生效")


def test_load_series_fs():
    path = generate_demo_log()
    fs, series = apex_sweep.load_log_series(path)
    assert abs(fs - 500.0) < 1.0, f"采样率推算错误：{fs}"
    assert "setpoint[0]" in series and "gyroADC[0]" in series
    print(f"  ✅ 日志加载：fs={fs}Hz，通道齐全")


if __name__ == "__main__":
    test_load_series_fs()
    test_bandwidth_identification()
    test_resonance_detection()
    test_demo_log_end_to_end()
    test_recommend_rules()
    print("\n全部通过")
