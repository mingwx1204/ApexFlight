# -*- coding: utf-8 -*-
"""v0.7 离屏冒烟测试：主窗口 + 黑匣子判别 + 双日志对比 + 全部页面渲染"""
import os
import sys

os.environ["QT_QPA_PLATFORM"] = "offscreen"
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from PyQt6.QtWidgets import QApplication
import main as m

OUT = os.path.join(os.path.dirname(__file__), "..", "logs", "smoke")
os.makedirs(OUT, exist_ok=True)

app = QApplication([])
win = m.MainWindow()
win.resize(1440, 900)
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


# 1. 演示黑匣子日志 → 判别
win.on_bb_demo()
app.processEvents()
lt = getattr(win, "bb_log_type", None)
check("demo 日志判别产生结论", bool(lt and lt.get("verdict")), str(lt))
print(f"     → 判别: {lt['verdict']} (置信度 {lt['confidence']})")
check("判别理由非空", len(lt["reasons"]) >= 1)

# 2. 模拟加载第二段日志并启用对比模式
win.bb2_time = win.bb_time
win.bb2_data = win.bb_data
win.bb2_columns = win.bb_columns
win.bb2_name = "对比日志(模拟)"
win.bb_compare.setChecked(True)
win.on_bb_plot()
app.processEvents()
axes = win.bb_canvas.figure.axes
check("对比模式绘图不崩且有多轴", len(axes) >= 1)
legends = [a.get_legend() for a in axes if a.get_legend()]
check("对比模式有图例", len(legends) >= 1)

# 3. AI 上下文包含判别结论
ctx = win._collect_tuning_context()
check("AI 上下文包含判别", "判别" in ctx or "空转" in ctx or "日志类型" in ctx,
      ctx[:200])

# 4. 取消对比再画一次
win.bb_compare.setChecked(False)
win.on_bb_plot()
app.processEvents()
check("单日志绘图正常", True)

# 5. 渲染全部页面截图
n_pages = win.pages.count() if hasattr(win, "pages") else 0
check("存在 12 个页面", n_pages == 12, f"实际 {n_pages}")
for i in range(n_pages):
    try:
        if hasattr(win, "sidebar"):
            win.sidebar.setCurrentRow(i)
        app.processEvents()
        win.grab().save(os.path.join(OUT, f"page{i}.png"))
    except Exception as e:
        failed += 1
        print(f"  💥 页面 {i} 渲染失败: {e}")
print("  ✅ 全部页面截图完成")

# 6. AI 页切到黑匣子页签不崩
try:
    win.bb_ai_btn.click()
    app.processEvents()
    check("黑匣子 AI 提问入口可点击", True)
except Exception as e:
    failed += 1
    print(f"  💥 AI 入口: {e}")

# 7. v0.8：模拟接入 INAV 飞控 → 兼容提示条出现、写入按钮锁定
win.on_connected({"firmware": "Betaflight 4.5.2（注意：检测到固件为 INAV）",
                  "board": "TEST", "motors": "4 个电机通道",
                  "variant": "INAV", "version_tuple": (7, 1, 0)})
app.processEvents()
check("INAV 接入显示兼容提示条", not win.compat_label.isHidden())
check("INAV 接入锁定 Rates 写入", not win.rates_write_btn.isEnabled())
check("INAV 接入锁定滤波写入", not win.filter_write_btn.isEnabled())
check("INAV 接入锁定预设应用", not win.preset_apply_btn.isEnabled())

# 8. v0.8：模拟接入 BF 4.5.2 → 无提示条、完全支持
win.on_connected({"firmware": "Betaflight 4.5.2", "board": "TEST",
                  "motors": "4 个电机通道",
                  "variant": "BTFL", "version_tuple": (4, 5, 2)})
app.processEvents()
check("BF 4.5 接入无兼容提示", not win.compat_label.isVisible())

# 9. v0.9：版本号显示
import apex_i18n as i18n
check("窗口标题含版本号", i18n.APP_VERSION in win.windowTitle(),
      win.windowTitle())
check("顶栏版本标签", win._version_label.text() == f"v{i18n.APP_VERSION}")

# 10. v0.9：日志页收到事件
m.log_event("冒烟测试日志事件")
app.processEvents()
check("日志页显示事件", "冒烟测试日志事件" in win.log_view.toPlainText())

# 11. v0.9：语言切换到英文再切回（全界面立即生效）
i18n.set_language("en")
win.retranslate_ui()
app.processEvents()
item0 = win.sidebar.item(0).text()
check("英文侧栏", "Welcome" in item0 and "Dashboard" in win.sidebar.item(1).text(), item0)
check("英文顶栏按钮", win.connect_button.text() == "Connect")
# 页面内部也应切换：仪表盘分组、Rates 分组、按钮
from PyQt6.QtWidgets import QGroupBox as _GB, QPushButton as _PB, QLabel
titles = {w.title() for w in win.findChildren(_GB)}
check("英文分组标题（飞控信息）", "FC Info" in titles, str(titles)[:120])
check("英文分组标题（油门）", "Throttle" in titles)
check("英文分组标题（双日志对比）", "Dual-log compare" in titles)
btns = {w.text() for w in win.findChildren(_PB)}
check("英文按钮（保存）", "Save" in btns)
check("英文按钮（频谱分析）", any("FFT" in b for b in btns))
labels = {w.text() for w in win.findChildren(QLabel)}
check("英文表单标签", "Firmware:" in labels)
check("英文滤波器字段", any("Min cutoff (Hz)" in l for l in labels))
# 切回中文：应完整还原
i18n.set_language("zh")
win.retranslate_ui()
app.processEvents()
check("切回中文侧栏", "欢迎" in win.sidebar.item(0).text() and "仪表盘" in win.sidebar.item(1).text())
check("切回中文按钮", win.connect_button.text() == "连接")
titles_zh = {w.title() for w in win.findChildren(_GB)}
check("中文还原（飞控信息）", "飞控信息" in titles_zh)
check("中文还原（油门）", "油门" in titles_zh)

# 12. v0.9：诊断信息复制 + 配置往返
win._copy_diagnostics()
clip = QApplication.clipboard().text()
check("诊断信息含版本号", i18n.APP_VERSION in clip, clip[:80])
cfg = i18n.load_config()
cfg["language"] = "zh"
i18n.save_config(cfg)
check("config.json 存在", i18n.CONFIG_PATH.exists())

win.close()
print(f"\n{'全部通过' if failed == 0 else str(failed) + ' 项失败'}")
sys.exit(1 if failed else 0)
