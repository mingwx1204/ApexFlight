# -*- coding: utf-8 -*-
"""崩溃重现最小脚本：扫频绘图后 processEvents 原生崩溃 (RC=127)。"""
import os, sys, math
os.environ.setdefault('QT_QPA_PLATFORM', '')
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
import numpy as np
from PyQt6.QtWidgets import QApplication

app = QApplication([])
import main, apex_sweep

if os.environ.get('DISABLE_AI_REFRESH'):
    main.MainWindow.on_ai_refresh = lambda self, *a, **k: None
if os.environ.get('TRACE_AI_REFRESH'):
    _orig = main.MainWindow.on_ai_refresh
    def _traced(self, *a, **k):
        print('>>> on_ai_refresh FIRED', flush=True)
        try:
            r = _orig(self, *a, **k)
            print('>>> on_ai_refresh RETURNED OK', flush=True)
            return r
        except BaseException as e:
            import traceback
            print('>>> on_ai_refresh RAISED:', repr(e), flush=True)
            traceback.print_exc()
            raise
    main.MainWindow.on_ai_refresh = _traced

# --- 修复方案实验：猴子补丁 _ensure_page ---
_patch = os.environ.get('PATCH_MODE', '')
if _patch:
    from PyQt6.QtCore import QCoreApplication, QEvent
    def _ensure_patched(self, index: int):
        if self._page_built[index]:
            return
        old = self.pages.widget(index)
        w = self._builders[index]()
        self.pages.removeWidget(old)
        if old is not None:
            if _patch == 'nodelete':
                old.setParent(None)   # 不删，挂起
            elif _patch == 'deletefirst':
                old.deleteLater()
                QCoreApplication.sendPostedEvents(old, QEvent.Type.DeferredDelete)
            elif _patch == 'hideonly':
                old.hide()            # 留在栈里但隐藏（占索引，不能用）
        self.pages.insertWidget(index, w)
        self._page_built[index] = True
    main.MainWindow._ensure_page = _ensure_patched

win = main.MainWindow()
_pre = os.environ.get('PRE_ENSURE', '')
if _pre:
    for i in (int(x) for x in _pre.split(',') if x.strip()):
        win._ensure_page(i)
win.show()
app.processEvents()
print('setup ok', flush=True)

FS = 2000.0
t = np.arange(0, 12.0, 1.0 / FS)
k = math.log(250.0 / 2.0)
phase = 2 * math.pi * 2.0 * t[-1] / k * (np.exp(k * t / t[-1]) - 1)
u = 300.0 * np.sin(phase)


def plant(u, wn_hz, zeta):
    wn = 2 * math.pi * wn_hz
    T = 1.0 / FS
    x1 = x2 = 0.0
    y = np.zeros_like(u)
    for i in range(len(u)):
        x1 += T * x2
        x2 += T * (wn * wn * (u[i] - x1) - 2 * zeta * wn * x2)
        y[i] = x1
    return y


m_roll = apex_sweep.analyze_axis(u, plant(u, 40.0, 0.55), FS)
m_roll['resonances'] = [(185.0, 8.2), (320.0, 4.1)]
result = {'axes': {'横滚': m_roll}, 'suggestions': []}
print('analyze ok', flush=True)
if os.environ.get('ONLY_ENSURE'):
    win._ensure_page(7)
    app.processEvents()
    print('ensure+events ok', flush=True)
    sys.exit(0)
if 7 not in getattr(win, '_page_built', []) or not win._page_built[7]:
    win._ensure_page(7)
win._sweep_show_result(result)
print('show ok', flush=True)
if not os.environ.get('SKIP_EVENTS'):
    app.processEvents()
print('events ok', flush=True)
print('REPRO_DONE', flush=True)
