# -*- coding: utf-8 -*-
"""v0.99 离屏测试：_run_simple_task 信号派发在重负载下不丢回调"""
import os
import sys
import time

os.environ["QT_QPA_PLATFORM"] = "offscreen"
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from PyQt6.QtCore import QElapsedTimer, QTimer, pyqtSignal, QObject
from PyQt6.QtWidgets import QApplication
import main as m

app = QApplication([])
win = m.MainWindow()
win.show()
app.processEvents()

failed = 0


def check(name, cond, detail=""):
    global failed
    if cond:
        print(f"  ✅ {name}")
    else:
        failed += 1
        print(f"  ❌ {name} {detail}")


def spin_until(pred, timeout_ms=5000):
    """转事件循环直到 pred() 为真或超时，返回是否等到"""
    t = QElapsedTimer()
    t.start()
    while not pred() and t.elapsed() < timeout_ms:
        app.processEvents()
        time.sleep(0.005)
    return pred()


# ---- 1. 基本回调：work 返回结果，done 必须收到 ----
got = []
win._run_simple_task(lambda: 42, got.append, "测试任务")
check("简单任务回调到达", spin_until(lambda: got, 3000))
check("回调结果正确", got == [42], str(got))

# ---- 2. 重负载事件流：高频信号轰炸期间并发 5 个任务，全部回调都必须到达 ----
class Noise(QObject):
    tick = pyqtSignal(int)


noise = Noise()
noise_count = [0]
noise.tick.connect(lambda i: noise_count.__setitem__(0, i + 1))
noise_stop = [False]


def bombard():
    i = 0
    while not noise_stop[0] and i < 20000:
        noise.tick.emit(i)
        i += 1


import threading
results = []
for k in range(5):
    win._run_simple_task(lambda k=k: k * 10, results.append, f"任务{k}")

t = threading.Thread(target=bombard, daemon=True)
t.start()
ok = spin_until(lambda: len(results) == 5, 8000)
noise_stop[0] = True
t.join(timeout=2)
check("重负载下 5 个并发任务回调全部到达", ok, f"只到 {len(results)}/5，噪声事件 {noise_count[0]} 个")
check("结果集合正确", sorted(results) == [0, 10, 20, 30, 40], str(results))

# ---- 3. 异常路径：work 抛错 → statusBar 提示，不崩 ----
def boom():
    raise RuntimeError("故意失败")

win._run_simple_task(boom, lambda r: None, "性能匹配失败")
ok = spin_until(lambda: "性能匹配失败" in win.statusBar().currentMessage(), 3000)
check("异常任务走 statusBar 提示", ok, win.statusBar().currentMessage())

print()
if failed:
    print(f"TASK_DISPATCH_FAIL ({failed} 项)")
    sys.exit(1)
print("TASK_DISPATCH_OK")
