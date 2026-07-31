# -*- coding: utf-8 -*-
"""
ApexFlight —— 开源无人机调参软件
v0.8：实时仪表盘 + PID/Rates/滤波在线调参 + 调参方案管理
      + 电机测试 + 接收机监视 + 黑匣子分析（闪存下载/频谱/双日志对比）
      + 日志类型智能判别（飞行 vs 地面空转）+ 本地 AI 助手（Ollama）
      + 全面兼容适配（固件变体识别 / 版本分级 / 容错解析 / 环境自检）

代码结构（v0.7 起按职责拆分）：
    apex_msp.py      MSP 协议编解码（v1/v2 帧、CRC8）
    apex_fc.py       飞控查询/写入、备份、调参快照
    apex_blackbox.py 黑匣子解码、统计、日志类型判别
    apex_ai.py       Ollama AI 助手通信
    apex_widgets.py  自定义控件（开关、姿态仪）
    main.py          后台串口线程 + 主窗口 + 入口（本文件）

运行方式：
    1. 安装依赖：  pip install -r requirements.txt
    2. 运行程序：  python src/main.py   （或双击 启动ApexFlight.bat）

技术栈：PyQt6 + pyserial + MSP v1/v2；所有串口操作在后台线程执行。
"""

import sys
import threading
import time
from datetime import datetime
from pathlib import Path

# v0.8：依赖自检 —— 缺 PyQt6/pyserial 时给出可读提示而不是一堆 traceback
try:
    import serial
    from serial.tools import list_ports

    from PyQt6.QtCore import QObject, Qt, QTimer, pyqtSignal
    from PyQt6.QtGui import (
        QColor, QFont, QIcon, QPainter, QPainterPath, QPen, QPixmap)
    from PyQt6.QtWidgets import (
        QAbstractButton, QApplication, QCheckBox, QComboBox, QDialog,
        QDoubleSpinBox,
        QFileDialog, QFormLayout, QFrame, QGridLayout, QGroupBox, QHBoxLayout,
        QHeaderView, QLabel, QListWidget, QListWidgetItem, QMainWindow,
        QMessageBox, QProgressBar, QPushButton, QScrollArea, QSlider, QSpinBox,
        QStackedWidget, QTableWidget, QTableWidgetItem, QTextEdit, QLineEdit,
        QVBoxLayout, QWidget)
except ImportError as _dep_err:
    print("=" * 56)
    print(f"  缺少运行依赖：{_dep_err.name}")
    print("  请先在项目目录执行：")
    print("      pip install -r requirements.txt")
    print("  （或单独安装：pip install PyQt6 pyserial matplotlib）")
    print("=" * 56)
    try:
        input("按回车键退出……")
    except EOFError:
        pass
    raise SystemExit(1)

# matplotlib 用于黑匣子曲线绘制（嵌入式画布）。
# v0.91：改为延迟加载——模块导入时不再阻塞，main() 在启动画面显示后
# 再调用 load_matplotlib()，启动更快、有进度反馈；未安装时黑匣子页给出提示。
HAS_MPL = False
Figure = None
FigureCanvasQTAgg = None
NavigationToolbar2QT = None


def load_matplotlib() -> bool:
    """加载 matplotlib（幂等）。返回是否可用。"""
    global HAS_MPL, Figure, FigureCanvasQTAgg, NavigationToolbar2QT
    if HAS_MPL or Figure is not None:
        return HAS_MPL
    try:
        import matplotlib
        matplotlib.use("QtAgg")
        # 中文显示配置（微软雅黑），并修复负号显示
        matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
        matplotlib.rcParams["axes.unicode_minus"] = False
        from matplotlib.backends.backend_qtagg import (
            FigureCanvasQTAgg as _Canvas, NavigationToolbar2QT as _Toolbar)
        from matplotlib.figure import Figure as _Figure
        Figure, FigureCanvasQTAgg, NavigationToolbar2QT = (
            _Figure, _Canvas, _Toolbar)
        HAS_MPL = True
    except ImportError:
        HAS_MPL = False
    return HAS_MPL

# 同目录下的功能模块（src/ 已在 sys.path 中，直接 import）
from apex_msp import *       # noqa: F401,F403  MSP 协议
from apex_fc import *        # noqa: F401,F403  飞控查询/写入/备份/快照
from apex_blackbox import *  # noqa: F401,F403  黑匣子分析
from apex_ai import *        # noqa: F401,F403  AI 助手
from apex_widgets import *   # noqa: F401,F403  自定义控件
import apex_i18n as i18n     # 版本号 / 配置 / 多语言
from apex_i18n import tr     # 界面翻译
from apex_virtual import (    # v0.93：虚拟飞控（无真机体验全部功能）
    VIRTUAL_PORT, VIRTUAL_PORT_LABEL, VirtualSerial)



# ------------------------------------------------------------
# 应用日志（v0.99 独立为 apex_log 模块，页签 mixin 共用）
# ------------------------------------------------------------
from apex_log import *        # noqa: F401,F403  app_logger / log_event 等
from apex_log import _app_log_lines, _app_log_lock   # 显式导入（星号不含下划线名）
from tab_map import MapTabMixin          # 适飞地图页
from tab_sweep import SweepTabMixin      # 扫频调参页
from tab_motor import MotorTabMixin      # 电机测试页
from tab_log import LogTabMixin          # 应用日志页

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
    motor_values_ready = pyqtSignal(list)     # 电机输出读回（µs 列表）
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
            if port == VIRTUAL_PORT:
                # v0.93：虚拟连接——用仿真飞控替代真实串口，
                # 后续所有 MSP 查询/写入走同一条代码路径
                self.serial_port = VirtualSerial()
                self.status.emit("虚拟模式：正在启动仿真飞控……")
            else:
                self.serial_port = serial.Serial(
                    port=port, baudrate=baudrate,
                    bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.2, write_timeout=2,
                )
                time.sleep(0.5)               # 等待飞控串口稳定

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
            # v0.8：按错误类型给出可操作的排查提示
            detail = str(e)
            if "PermissionError" in detail or "拒绝访问" in detail \
                    or "Access is denied" in detail:
                hint = ("串口被占用：请关闭 Betaflight Configurator / "
                        "其他地面站和串口工具后重试。")
            elif "FileNotFoundError" in detail or "找不到指定" in detail \
                    or "cannot find" in detail.lower():
                hint = "串口不存在：飞控可能已拔出，重新插拔 USB 后点「刷新」。"
            else:
                hint = ("请确认已安装飞控驱动（STM32 VCP / CP210x / CH340），"
                        "并换一个 USB 口试试。")
            self.error.emit(f"无法打开串口 {port}\n{hint}\n（{detail}）")
            self.close_port()
        except MspError as e:
            hint = ""
            if "超时" in str(e):
                hint = ("\n设备无响应：确认选的是飞控（而非蓝牙/调试器串口），"
                        "波特率保持 115200；刚插上 USB 可等 2 秒再连。")
            self.error.emit(f"MSP 通信失败：{e}{hint}")
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

    def read_motor_values(self):
        """读回飞控实际电机输出（电机测试期间轮询，对标 BF motorData）"""
        if not self.is_connected:
            return
        try:
            self.motor_values_ready.emit(
                query_motor_values(self.serial_port))
        except (MspError, serial.SerialException):
            pass                          # 轮询失败静默，下一轮再试

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
# 第六部分：GUI 主窗口
# ============================================================

class MainWindow(MapTabMixin, SweepTabMixin, MotorTabMixin,
                LogTabMixin, QMainWindow):
    """ApexFlight 主窗口（6 个功能页签）"""

    # AI 探测结果信号（后台线程探测 Ollama → 界面线程刷新显示）
    ai_probe_done = pyqtSignal(bool, list)
    # 后台小任务完成信号（v0.99：替代 QTimer.singleShot 派发。
    # 信号走事件队列 FIFO 投递，下载/分析等重负载期间不会被饿死）
    task_result_ready = pyqtSignal(object)

    def __init__(self):
        super().__init__()
        self.worker = SerialWorker()
        self.ai = AIBridge()                  # AI 对话桥（Ollama）
        self._ai_messages = []                # 对话历史（发给模型的上下文）
        self._ai_reply_buffer = ""            # 当前这一轮回答的累积文字
        self._ai_busy = False                 # AI 是否正在回答
        self._ai_probe_cache = None           # AI 探测结果缓存（懒加载用）
        self._flash_cancel = None             # 闪存下载取消标志（须在任何
        #                                       回调触发前就存在：on_error 是
        #                                       全局槽，可能先于黑匣子页构建）
        self._threads = []                    # 持有线程引用，防止被回收
        self._poll_thread = None              # 慢通道轮询线程（竞态防护）
        self._poll_fast_thread = None         # 快通道轮询线程
        self._pid_names = []                  # 当前 PID 名称列表
        self._motor_sliders = []              # 电机滑块列表
        self._motor_count = 0
        self._compat = None                   # 兼容性评估结果（连接后设置）

        # 用户配置（语言 / 默认波特率），启动时加载并应用
        self._cfg = i18n.load_config()
        i18n.init_from_config(self._cfg)

        self.setWindowTitle(f"{i18n.APP_NAME} v{i18n.APP_VERSION}")
        self.resize(1360, 850)                # v0.93：字号加大后同步放大工作区
        self.setMinimumSize(1160, 730)
        self._center_on_screen()
        if ICON_PATH.exists():
            self.setWindowIcon(QIcon(str(ICON_PATH)))

        self._build_ui()
        self._apply_stylesheet()
        self._connect_signals()
        # 启动时应用非中文语言（设置里切换则立即生效）
        if i18n.get_language() != "zh":
            self.retranslate_ui()

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
        """应用全局暗色主题（v0.98 现代化翻新：渐变主按钮 / 分层卡片 /
        输入框聚焦态 / 双向滑块 / 隔行表格；配色取自图标：青 #3EC6E8 + 橙 #F5A83D）"""
        self.setStyleSheet("""
            /* ===== 全局底色与字体 ===== */
            QMainWindow, QWidget {
                background: #181B21;
                color: #E9ECEF;
                font-family: "Microsoft YaHei", "Segoe UI";
                font-size: 15px;
            }
            /* ===== 顶栏：微渐变 + 细分隔线 ===== */
            QWidget#topbar {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #171A20, stop:1 #12151A);
                border-bottom: 1px solid #2A3039;
            }
            QLabel#titleLabel {
                color: #3EC6E8;
                font-size: 23px;
                font-weight: bold;
                padding-left: 4px;
            }
            QLabel#subtitleLabel { color: #8B929C; padding-left: 8px; }
            /* ===== 左侧导航栏：深色底 + 胶囊选中态 ===== */
            QListWidget#sidebar {
                background: #11141A;
                border: none;
                border-right: 1px solid #262C35;
                outline: none;
                font-size: 16px;
            }
            QListWidget#sidebar::item {
                padding: 13px 14px;
                margin: 3px 8px;
                border-radius: 10px;
                color: #8B929C;
                border-left: 3px solid transparent;
            }
            QListWidget#sidebar::item:selected {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #1E3A44, stop:1 #18242B);
                color: #3EC6E8;
                border-left: 3px solid #3EC6E8;
                font-weight: bold;
            }
            QListWidget#sidebar::item:hover {
                background: #1A1E25;
                color: #C9CDD3;
            }
            /* ===== 分组框：分层卡片 ===== */
            QGroupBox {
                border: 1px solid #2A3039;
                border-radius: 12px;
                margin-top: 14px;
                padding-top: 12px;
                background: #1F232B;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 6px;
                color: #3EC6E8;
                font-weight: bold;
            }
            /* ===== 按钮：默认次级 + 渐变主按钮 ===== */
            QPushButton {
                background: #272C35;
                border: 1px solid #3A414C;
                border-radius: 10px;
                padding: 8px 16px;
                color: #E9ECEF;
            }
            QPushButton:hover {
                background: #30363F;
                border-color: #3EC6E8;
            }
            QPushButton:pressed { background: #21262E; }
            QPushButton:disabled { color: #596069; background: #20242B;
                border-color: #2A3039; }
            QPushButton#connectBtn {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 #4CD0F0, stop:1 #2AA9C9);
                color: #0C2028;
                font-weight: bold;
                border: none;
            }
            QPushButton#connectBtn:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 #6CDBF4, stop:1 #38B8D8);
            }
            QPushButton#connectBtn:pressed { background: #2598B8; }
            QPushButton#connectBtn:disabled { background: #27505C;
                color: #7FA5AF; }
            QPushButton#disconnectBtn {
                background: transparent;
                border: 1px solid #F5A83D;
                color: #F5A83D;
            }
            QPushButton#disconnectBtn:hover { background: #33291A; }
            QPushButton#dangerBtn {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 #EE5A5A, stop:1 #C93C3C);
                color: white;
                font-weight: bold;
                border: none;
                padding: 8px 20px;
            }
            QPushButton#dangerBtn:hover { background: #F06868; }
            QPushButton#dangerBtn:disabled { background: #4E2A2A;
                color: #9A7A7A; }
            /* ===== 下拉框 ===== */
            QComboBox {
                background: #16191F;
                border: 1px solid #3A414C;
                border-radius: 10px;
                padding: 7px 10px;
            }
            QComboBox:hover, QComboBox:focus { border-color: #3EC6E8; }
            QComboBox::drop-down {
                border: none;
                width: 26px;
            }
            QComboBox QAbstractItemView {
                background: #1F232B;
                border: 1px solid #3A414C;
                selection-background-color: #3EC6E8;
                selection-color: #0C2028;
                outline: none;
            }
            /* ===== 表格：隔行变色 + 细网格 ===== */
            QTableWidget {
                background: #1A1E25;
                alternate-background-color: #20242D;
                gridline-color: #2A3039;
                border: 1px solid #2A3039;
                border-radius: 10px;
                selection-background-color: #3EC6E8;
                selection-color: #0C2028;
            }
            QHeaderView::section {
                background: #232830;
                color: #3EC6E8;
                padding: 8px;
                border: none;
                border-bottom: 1px solid #2A3039;
                font-weight: bold;
            }
            QTableWidget::item:selected {
                background: #3EC6E8;
                color: #0C2028;
            }
            /* ===== 输入框：内凹深色 + 聚焦亮边 ===== */
            QLineEdit {
                background: #14171C;
                border: 1px solid #3A414C;
                border-radius: 8px;
                padding: 7px 9px;
                selection-background-color: #3EC6E8;
                selection-color: #0C2028;
            }
            QLineEdit:focus { border-color: #3EC6E8; }
            /* ===== 文本编辑区 / 数字输入框 ===== */
            QTextEdit, QSpinBox, QDoubleSpinBox {
                background: #14171C;
                border: 1px solid #2A3039;
                border-radius: 10px;
                padding: 4px 6px;
            }
            QTextEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus {
                border-color: #3EC6E8;
            }
            QSpinBox, QDoubleSpinBox { padding-right: 4px; }
            /* ===== 滑块：横向 + 纵向（电机页用纵向） ===== */
            QSlider::groove:horizontal {
                height: 8px;
                background: #2A3039;
                border-radius: 4px;
            }
            QSlider::sub-page:horizontal {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #2AA9C9, stop:1 #4CD0F0);
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                width: 20px;
                height: 20px;
                margin: -7px 0;
                border-radius: 10px;
                background: #4CD0F0;
                border: 2px solid #0C2028;
            }
            QSlider::handle:horizontal:hover { background: #6CDBF4; }
            QSlider::handle:horizontal:disabled { background: #596069; }
            QSlider::sub-page:horizontal:disabled { background: #333943; }
            QSlider::groove:vertical {
                width: 8px;
                background: #2A3039;
                border-radius: 4px;
            }
            QSlider::add-page:vertical {
                background: qlineargradient(x1:0, y1:1, x2:0, y2:0,
                    stop:0 #2AA9C9, stop:1 #4CD0F0);
                border-radius: 4px;
            }
            QSlider::handle:vertical {
                width: 20px;
                height: 20px;
                margin: 0 -7px;
                border-radius: 10px;
                background: #4CD0F0;
                border: 2px solid #0C2028;
            }
            QSlider::handle:vertical:hover { background: #6CDBF4; }
            QSlider::handle:vertical:disabled { background: #596069; }
            QSlider::add-page:vertical:disabled { background: #333943; }
            /* ===== 复选框 ===== */
            QCheckBox { spacing: 6px; }
            QCheckBox::indicator {
                width: 18px;
                height: 18px;
                border: 1px solid #3A414C;
                border-radius: 5px;
                background: #16191F;
            }
            QCheckBox::indicator:hover { border-color: #3EC6E8; }
            QCheckBox::indicator:checked {
                background: #F5A83D;
                border-color: #F5A83D;
            }
            /* ===== 状态栏 ===== */
            QStatusBar {
                background: #11141A;
                color: #8B929C;
                border-top: 1px solid #262C35;
            }
            /* ===== 滚动区 ===== */
            QScrollArea {
                border: 1px solid #2A3039;
                border-radius: 8px;
                background: #1A1E25;
            }
            /* ===== 滚动条：纤细现代 ===== */
            QScrollBar:vertical {
                background: transparent;
                width: 9px;
                margin: 2px;
            }
            QScrollBar::handle:vertical {
                background: #363D47;
                border-radius: 4px;
                min-height: 30px;
            }
            QScrollBar::handle:vertical:hover { background: #3EC6E8; }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                height: 0;
            }
            QScrollBar:horizontal {
                background: transparent;
                height: 9px;
                margin: 2px;
            }
            QScrollBar::handle:horizontal {
                background: #363D47;
                border-radius: 4px;
                min-width: 30px;
            }
            QScrollBar::handle:horizontal:hover { background: #3EC6E8; }
            QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {
                width: 0;
            }
            /* ===== 工具提示：深色卡片 ===== */
            QToolTip {
                background: #232830;
                color: #E9ECEF;
                border: 1px solid #3A414C;
                border-radius: 8px;
                padding: 6px 8px;
            }
        """)
        # 隔行变色对全部表格生效（QSS 的 alternate-background-color
        # 需要控件开启 alternatingRowColors）
        for table in self.findChildren(QTableWidget):
            table.setAlternatingRowColors(True)

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
                42, 42, Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation))
            top.addWidget(icon_label)

        title = QLabel("ApexFlight")
        title.setObjectName("titleLabel")
        top.addWidget(title)
        version_label = QLabel(f"v{i18n.APP_VERSION}")
        version_label.setObjectName("versionLabel")
        version_label.setStyleSheet("color: #7A828C; font-size: 14px; "
                                    "padding-top: 4px;")
        self._version_label = version_label
        top.addWidget(version_label)
        top.addStretch()

        self.lbl_port = QLabel(tr("串口"))
        top.addWidget(self.lbl_port)
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(210)
        top.addWidget(self.port_combo)
        self.refresh_button = QPushButton(tr("刷新"))
        self.refresh_button.clicked.connect(self.refresh_ports)
        top.addWidget(self.refresh_button)
        self.lbl_baud = QLabel(tr("波特率"))
        top.addWidget(self.lbl_baud)
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["57600", "115200", "230400", "460800"])
        self.baud_combo.setCurrentText(self._cfg.get("baud", "115200"))
        top.addWidget(self.baud_combo)
        self.connect_button = QPushButton(tr("连接"))
        self.connect_button.setObjectName("connectBtn")
        self.connect_button.setMinimumWidth(90)
        self.connect_button.clicked.connect(self.on_connect_clicked)
        top.addWidget(self.connect_button)
        self.disconnect_button = QPushButton(tr("断开"))
        self.disconnect_button.setObjectName("disconnectBtn")
        self.disconnect_button.setEnabled(False)
        self.disconnect_button.clicked.connect(self.on_disconnect_clicked)
        top.addWidget(self.disconnect_button)
        self.settings_button = QPushButton("⚙ " + tr("设置"))
        self.settings_button.clicked.connect(self.open_settings)
        top.addWidget(self.settings_button)
        layout.addWidget(topbar)

        # ---- 主体：左侧导航 + 右侧页面堆栈 ----
        body = QHBoxLayout()
        body.setContentsMargins(0, 0, 0, 0)
        body.setSpacing(0)

        self.sidebar = QListWidget()
        self.sidebar.setObjectName("sidebar")
        self.sidebar.setFixedWidth(196)       # v0.93：字号加大后同步加宽
        self.sidebar.setHorizontalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff)   # 隐藏横向滚动条
        # 侧栏条目 = 图标 + 名称键（名称键用于 i18n 翻译）
        self._sidebar_entries = [
            ("👋", "欢迎"),
            ("📊", "仪表盘"), ("🎛️", "PID 调参"), ("🎯", "Rates 调参"),
            ("🌊", "滤波器"), ("⚙️", "电机测试"),
            ("📈", "黑匣子"), ("🌀", "扫频调参"), ("💾", "调参方案"), ("🤖", "AI 助手"),
            ("🗺️", "适飞地图"),
            ("📜", "日志"),
        ]
        self.sidebar.addItems([f"{ic}  {tr(name)}"
                               for ic, name in self._sidebar_entries])
        self.sidebar.setCurrentRow(0)
        body.addWidget(self.sidebar)

        self.pages = QStackedWidget()
        # v0.99 页签懒加载：重资源页（黑匣子/扫频/AI/地图）首次切换到
        # 才构建——matplotlib 加载、瓦片线程池都不挡启动，冷启动明显更快
        self._builders = [self._build_welcome_tab, self._build_dashboard_tab,
                          self._build_pid_tab, self._build_rates_tab,
                          self._build_filter_tab, self._build_motor_tab,
                          self._build_blackbox_tab, self._build_sweep_tab,
                          self._build_preset_tab, self._build_ai_tab,
                          self._build_map_tab, self._build_log_tab]
        self._page_built = [True] * len(self._builders)
        _LAZY_PAGES = (6, 7, 9, 10)          # 黑匣子/扫频/AI/地图
        for i, builder in enumerate(self._builders):
            if i in _LAZY_PAGES:
                ph = QWidget()
                phl = QVBoxLayout(ph)
                lab = QLabel(tr("页面加载中…"))
                lab.setAlignment(Qt.AlignmentFlag.AlignCenter)
                lab.setStyleSheet("color: #7A828C; font-size: 15px;")
                phl.addWidget(lab)
                self.pages.addWidget(ph)
                self._page_built[i] = False
            else:
                self.pages.addWidget(builder())
        body.addWidget(self.pages, 1)

        body_widget = QWidget()
        body_widget.setLayout(body)
        layout.addWidget(body_widget, 1)

        # 导航点击切换页面（带淡入动画）
        self.sidebar.currentRowChanged.connect(self._on_page_changed)

        # 应用日志总线 → 日志页
        app_logger.appended.connect(self._on_app_log)
        log_event(f"ApexFlight v{i18n.APP_VERSION} 已启动")

        self.statusBar().showMessage(tr("就绪：请选择串口后点击「连接」"))

    def _ensure_page(self, index: int):
        """懒加载：首次切换到某页时才真正构建（v0.99）"""
        if self._page_built[index]:
            return
        old = self.pages.widget(index)
        w = self._builders[index]()
        self.pages.removeWidget(old)
        old.deleteLater()
        self.pages.insertWidget(index, w)
        self._page_built[index] = True
        log_event(f"页签首次加载：{self._sidebar_entries[index][1]}")

    def _on_page_changed(self, index: int):
        """切换页面：先切索引，再给新页面加 180ms 淡入动画（v0.91）。
        动画结束后移除透明效果，避免影响后续重绘性能。"""
        from PyQt6.QtCore import QAbstractAnimation, QPropertyAnimation
        from PyQt6.QtWidgets import QGraphicsOpacityEffect
        self._ensure_page(index)
        self.pages.setCurrentIndex(index)
        page = self.pages.currentWidget()
        if page is None:
            return
        effect = QGraphicsOpacityEffect(page)
        page.setGraphicsEffect(effect)
        anim = QPropertyAnimation(effect, b"opacity", self)
        anim.setDuration(180)
        anim.setStartValue(0.0)
        anim.setEndValue(1.0)
        anim.finished.connect(lambda: page.setGraphicsEffect(None))
        anim.start(QAbstractAnimation.DeletionPolicy.DeleteWhenStopped)

    # ---------- 后台小任务派发 ----------

    def _run_simple_task(self, work, done, err_hint: str):
        """在后台线程跑无参函数 work()，UI 线程回调 done(result)。

        每次调用独立闭包状态，多个任务并发互不干扰（v0.96）；
        完成派发走 task_result_ready 信号（v0.99）——信号按事件队列
        FIFO 投递，与下载进度等高频信号同优先级，杜绝 QTimer 派发
        在重负载下被饿死导致的「点了没反应」。"""
        import threading as _th
        state = {"done": done}

        def runner():
            try:
                state["result"] = work()
            except Exception as e:
                state["err"] = f"{err_hint}（{e}）"
            self.task_result_ready.emit(state)

        _th.Thread(target=runner, daemon=True).start()

    def _simple_task_dispatch(self, state: dict):
        """UI 线程：分发后台小任务结果（信号槽，永远按到达顺序执行）"""
        if "err" in state:
            self.statusBar().showMessage(state["err"], 7000)
            return
        state["done"](state.get("result"))

    # ---- v1.0 伯德图交互：截图（扫频/黑匣子共用） ----

    def _save_figure_png(self, fig, prefix: str):
        """把 matplotlib 图存成 PNG（发交流群讨论用），返回保存路径"""
        try:
            from datetime import datetime
            out_dir = LOGS_DIR / "screenshots"
            out_dir.mkdir(parents=True, exist_ok=True)
            path = out_dir / f"{prefix}_{datetime.now():%Y%m%d_%H%M%S}.png"
            fig.savefig(path, dpi=160, facecolor=fig.get_facecolor(),
                        bbox_inches="tight")
            self.statusBar().showMessage(f"截图已保存：{path}", 8000)
            log_event(f"图表截图已保存：{path}")
            return path
        except Exception as e:
            self.statusBar().showMessage(f"截图保存失败：{e}", 6000)
            return None

    # ---------- 设置对话框（参考 BF Configurator 设置） ----------

    def open_settings(self):
        """设置：语言、默认波特率、日志入口、关于信息"""
        from PyQt6.QtWidgets import QDialog, QDialogButtonBox
        dlg = QDialog(self)
        dlg.setWindowTitle("⚙ " + tr("设置"))
        dlg.setMinimumWidth(380)
        form = QFormLayout(dlg)

        lang_combo = QComboBox()
        lang_combo.addItem("简体中文", "zh")
        lang_combo.addItem("English", "en")
        lang_combo.setCurrentIndex(0 if i18n.get_language() == "zh" else 1)
        form.addRow(tr("语言") + "：", lang_combo)

        baud_combo = QComboBox()
        baud_combo.addItems(["57600", "115200", "230400", "460800"])
        baud_combo.setCurrentText(self._cfg.get("baud", "115200"))
        form.addRow(tr("默认波特率") + "：", baud_combo)

        # 基础单位切换（v0.99）：地图比例尺等带单位读数跟随
        units_combo = QComboBox()
        units_combo.addItem(tr("公制（米 / 公里）"), "metric")
        units_combo.addItem(tr("英制（英尺 / 英里）"), "imperial")
        units_combo.setCurrentIndex(
            0 if self._cfg.get("units", "metric") == "metric" else 1)
        form.addRow(tr("基础单位") + "：", units_combo)

        log_btn = QPushButton(tr("打开日志文件夹"))
        log_btn.clicked.connect(self._open_log_folder)
        form.addRow(tr("日志") + "：", log_btn)

        # 用户反馈通道（v0.9）：跳转 GitHub Issues + 一键复制诊断信息
        fb_row = QHBoxLayout()
        fb_btn = QPushButton("💬 " + tr("在 GitHub 上提交建议/问题"))
        fb_btn.clicked.connect(self._open_feedback)
        fb_row.addWidget(fb_btn)
        diag_btn = QPushButton("📋 " + tr("复制诊断信息"))
        diag_btn.clicked.connect(self._copy_diagnostics)
        fb_row.addWidget(diag_btn)
        fb_wrap = QWidget()
        fb_wrap.setLayout(fb_row)
        fb_row.setContentsMargins(0, 0, 0, 0)
        form.addRow(tr("意见反馈") + "：", fb_wrap)

        about = QLabel(
            f"{i18n.APP_NAME} v{i18n.APP_VERSION} · GPL-3.0<br>"
            "github.com/mingwx1204/ApexFlight")
        about.setStyleSheet("color: #7A828C;")
        form.addRow(tr("关于") + "：", about)

        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok
            | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(dlg.accept)
        buttons.rejected.connect(dlg.reject)
        form.addRow(buttons)

        if dlg.exec() != QDialog.DialogCode.Accepted:
            return

        # 应用设置并持久化到 config.json
        old_lang = i18n.get_language()
        self._cfg["language"] = lang_combo.currentData()
        self._cfg["baud"] = baud_combo.currentText()
        self._cfg["units"] = units_combo.currentData()
        i18n.save_config(self._cfg)
        self.baud_combo.setCurrentText(self._cfg["baud"])
        if hasattr(self, "map_widget"):
            self.map_widget.set_units(self._cfg["units"])
        i18n.set_language(self._cfg["language"])
        self.retranslate_ui()
        if i18n.get_language() != old_lang:
            self.statusBar().showMessage(tr(
                "语言已切换：导航与顶栏立即生效，其余界面重启后完全生效。"))
        log_event(f"设置已保存（语言={self._cfg['language']}，"
                  f"默认波特率={self._cfg['baud']}，"
                  f"单位={self._cfg['units']}）")

    FEEDBACK_URL = "https://github.com/mingwx1204/ApexFlight/issues/new"

    def _open_feedback(self):
        """用户反馈通道：打开 GitHub Issues 新建页（浏览器）"""
        import webbrowser
        webbrowser.open(self.FEEDBACK_URL)
        log_event("已打开意见反馈页面（GitHub Issues）")

    def _copy_diagnostics(self):
        """复制诊断信息到剪贴板：版本/系统/固件 + 最近 10 条日志，
        用户在 GitHub Issue 里直接粘贴即可"""
        import platform
        lines = [
            f"ApexFlight v{i18n.APP_VERSION}",
            f"OS: {platform.system()} {platform.release()} "
            f"({platform.machine()})",
            f"Firmware: {self.firmware_label.text()}",
            f"Board: {self.board_label.text()}",
        ]
        with _app_log_lock:
            tail = _app_log_lines[-10:]
        text = "\n".join(lines) + "\n\nRecent log:\n" + "\n".join(tail)
        QApplication.clipboard().setText(text)
        self.statusBar().showMessage(
            tr("诊断信息已复制，到 GitHub Issues 粘贴即可"))

    def retranslate_ui(self):
        """语言切换后全量刷新（立即生效，无需重启）：
        窗口标题 + 侧栏 + 递归遍历全部控件做双向翻译"""
        self.setWindowTitle(f"{i18n.APP_NAME} v{i18n.APP_VERSION}")
        # 侧栏条目（图标 + 名称键）
        row = self.sidebar.currentRow()
        for i, (icon, name) in enumerate(self._sidebar_entries):
            item = self.sidebar.item(i)
            if item:
                item.setText(f"{icon}  {tr(name)}")
        self.sidebar.setCurrentRow(row)
        self._retranslate_widget_tree(self)

    @staticmethod
    def _retranslate_widget_tree(root: QWidget):
        """递归遍历控件树，翻译所有静态文本（中↔英 双向）。

        覆盖：QLabel / QPushButton / QCheckBox / QAbstractButton（含开关）
        / QGroupBox 标题 / QComboBox 条目。纯 ASCII 与含变量的动态文本
        会被 localize_text 自动跳过。"""
        from PyQt6.QtWidgets import QComboBox
        for w in root.findChildren(QWidget):
            try:
                if isinstance(w, QGroupBox):
                    new = i18n.localize_text(w.title())
                    if new != w.title():
                        w.setTitle(new)
                elif isinstance(w, QComboBox):
                    for i in range(w.count()):
                        t = w.itemText(i)
                        new = i18n.localize_text(t)
                        if new != t:
                            w.setItemText(i, new)
                elif isinstance(w, (QLabel, QAbstractButton)):
                    t = w.text()
                    new = i18n.localize_text(t)
                    if new != t:
                        w.setText(new)
            except RuntimeError:
                pass                      # 控件可能已被销毁

    # ---------- 页签 0：欢迎（v0.92，参考 BF 欢迎页） ----------

    QQ_GROUP_NUMBER = "1045772754"
    AUTHOR_EMAIL = "1693161698@qq.com"
    REPO_URL = "https://github.com/mingwx1204/ApexFlight"

    def _copy_text(self, text: str):
        """复制文本到剪贴板并提示（欢迎页：群号/邮箱）"""
        QApplication.clipboard().setText(text)
        self.statusBar().showMessage(
            tr("已复制到剪贴板") + f"：{text}", 3000)

    def _build_welcome_tab(self) -> QWidget:
        """欢迎页（v0.92，参考 BF 欢迎页）：品牌区 + 快速上手 +
        QQ 交流群 / 联系作者 / 开源社区三张卡片"""
        tab = QWidget()
        outer = QVBoxLayout(tab)
        outer.setContentsMargins(26, 18, 26, 18)
        outer.setSpacing(14)

        # ---- 品牌区：图标 + 名称 + 版本 + 标语 ----
        brand = QVBoxLayout()
        brand.setSpacing(5)
        if ICON_PATH.exists():
            icon = QLabel()
            icon.setPixmap(QPixmap(str(ICON_PATH)).scaled(
                100, 100, Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation))
            icon.setAlignment(Qt.AlignmentFlag.AlignCenter)
            brand.addWidget(icon)
        name = QLabel("ApexFlight")
        name.setStyleSheet("color: #3EC6E8; font-size: 34px; "
                           "font-weight: bold;")
        name.setAlignment(Qt.AlignmentFlag.AlignCenter)
        brand.addWidget(name)
        ver = QLabel(f"v{i18n.APP_VERSION} · GPL-3.0")
        ver.setStyleSheet("color: #7A828C; font-size: 13px;")
        ver.setAlignment(Qt.AlignmentFlag.AlignCenter)
        brand.addWidget(ver)
        tag = QLabel(tr("开源 FPV 无人机调参软件"))
        tag.setStyleSheet("color: #E8E8E8; font-size: 16px;")
        tag.setAlignment(Qt.AlignmentFlag.AlignCenter)
        brand.addWidget(tag)
        sub = QLabel(tr("全本地运行 · 零云端 · 保护飞手隐私"))
        sub.setStyleSheet("color: #9AA0A6;")
        sub.setAlignment(Qt.AlignmentFlag.AlignCenter)
        brand.addWidget(sub)
        outer.addLayout(brand)

        # ---- 快速上手 ----
        quick = QGroupBox("🚀 " + tr("快速上手"))
        qv = QVBoxLayout(quick)
        for s in ("① 插入飞控 USB，关闭 Betaflight Configurator（串口独占）",
                  "② 顶栏选择串口，点击「连接」",
                  "③ 仪表盘查看姿态与状态，到 PID / Rates 页开始调参"):
            lab = QLabel(tr(s))
            lab.setStyleSheet("color: #C8CDD3; padding: 2px 4px;")
            qv.addWidget(lab)
        outer.addWidget(quick)

        # ---- 机型信息备注（v0.99）：本地保存，调参讨论时随截图报配置 ----
        craft = QGroupBox("🛩️ " + tr("机型信息备注"))
        cf = QGridLayout(craft)
        self._craft_edits = {}
        craft_fields = [("名称", tr("如：5寸花飞机")), ("机架", tr("如：Mark5")),
                        ("电机", tr("如：2207 1950KV")), ("桨叶", tr("如：51466 三叶")),
                        ("电池", tr("如：6S 1300mAh")), ("备注", tr("自由记录"))]
        saved_craft = self._cfg.get("craft", {})
        for i, (key, ph) in enumerate(craft_fields):
            lab = QLabel(tr(key) + "：")
            edit = QLineEdit(saved_craft.get(key, ""))
            edit.setPlaceholderText(ph)
            self._craft_edits[key] = edit
            row, col = divmod(i, 2)
            cf.addWidget(lab, row, col * 2)
            cf.addWidget(edit, row, col * 2 + 1)
        craft_save = QPushButton("💾 " + tr("保存机型信息"))
        craft_save.clicked.connect(self._save_craft_info)
        cf.addWidget(craft_save, 3, 0, 1, 4)
        outer.addWidget(craft)

        # ---- 三张卡片：QQ 群 / 联系作者 / 开源社区 ----
        cards = QHBoxLayout()
        cards.setSpacing(14)

        # QQ 交流群卡
        qq = QGroupBox("📡 " + tr("QQ 交流群"))
        qql = QVBoxLayout(qq)
        if QQ_QR_PATH.exists():
            qr = QLabel()
            qr.setPixmap(QPixmap(str(QQ_QR_PATH)).scaled(
                170, 170, Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation))
            qr.setAlignment(Qt.AlignmentFlag.AlignCenter)
            qql.addWidget(qr)
        hint = QLabel(tr("扫码或搜群号加入，一起交流调参心得"))
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #9AA0A6;")
        hint.setAlignment(Qt.AlignmentFlag.AlignCenter)
        qql.addWidget(hint)
        qq_row = QHBoxLayout()
        no = QLabel(tr("群号：") + self.QQ_GROUP_NUMBER)
        no.setStyleSheet("color: #3EC6E8; font-weight: bold;")
        qq_row.addWidget(no, 1)
        copy_qq = QPushButton(tr("复制群号"))
        copy_qq.clicked.connect(
            lambda: self._copy_text(self.QQ_GROUP_NUMBER))
        qq_row.addWidget(copy_qq)
        qql.addLayout(qq_row)
        cards.addWidget(qq)

        # 联系作者卡
        mail = QGroupBox("📧 " + tr("联系作者"))
        ml = QVBoxLayout(mail)
        ml.addStretch()
        mh = QLabel(tr("有建议或合作意向，欢迎来邮"))
        mh.setWordWrap(True)
        mh.setStyleSheet("color: #9AA0A6;")
        mh.setAlignment(Qt.AlignmentFlag.AlignCenter)
        ml.addWidget(mh)
        addr = QLabel(self.AUTHOR_EMAIL)
        addr.setStyleSheet("color: #3EC6E8; font-size: 15px; "
                           "font-weight: bold;")
        addr.setAlignment(Qt.AlignmentFlag.AlignCenter)
        ml.addWidget(addr)
        copy_mail = QPushButton(tr("复制邮箱"))
        copy_mail.clicked.connect(
            lambda: self._copy_text(self.AUTHOR_EMAIL))
        ml.addWidget(copy_mail, alignment=Qt.AlignmentFlag.AlignCenter)
        ml.addStretch()
        cards.addWidget(mail)

        # 开源社区卡
        gh = QGroupBox("🐙 " + tr("开源社区"))
        gl = QVBoxLayout(gh)
        gl.addStretch()
        gh_hint = QLabel(tr("代码开源（GPL-3.0），欢迎 Star / Issue / PR"))
        gh_hint.setWordWrap(True)
        gh_hint.setStyleSheet("color: #9AA0A6;")
        gh_hint.setAlignment(Qt.AlignmentFlag.AlignCenter)
        gl.addWidget(gh_hint)
        repo_btn = QPushButton("🐙 " + tr("打开 GitHub 仓库"))
        repo_btn.clicked.connect(
            lambda: __import__("webbrowser").open(self.REPO_URL))
        gl.addWidget(repo_btn)
        fb_btn = QPushButton("💬 " + tr("在 GitHub 上提交建议/问题"))
        fb_btn.clicked.connect(self._open_feedback)
        gl.addWidget(fb_btn)
        gl.addStretch()
        cards.addWidget(gh)

        outer.addLayout(cards)
        outer.addStretch()
        return tab

    def _save_craft_info(self):
        """机型信息备注：持久化到 config.json（本地保存，不上传）"""
        craft = {k: e.text().strip() for k, e in self._craft_edits.items()}
        self._cfg["craft"] = craft
        i18n.save_config(self._cfg)
        name = craft.get("名称") or "（未命名）"
        self.statusBar().showMessage(f"机型信息已保存：{name}", 4000)
        log_event(f"机型信息已保存（{name}）")

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

        # 兼容性提示条（v0.8）：非 BF 固件 / 未验证版本时显示，平时隐藏
        self.compat_label = QLabel("")
        self.compat_label.setWordWrap(True)
        self.compat_label.setVisible(False)
        self.compat_label.setStyleSheet(
            "QLabel { background: #3D2E12; color: #F5C542; "
            "border: 1px solid #8A6D1F; border-radius: 6px; padding: 8px; }")
        left.addWidget(self.compat_label)

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
        buttons.addStretch()
        self.pid_reload_btn = QPushButton("刷新")
        self.pid_reload_btn.clicked.connect(self.on_pid_reload)
        self.pid_reload_btn.setEnabled(False)
        buttons.addWidget(self.pid_reload_btn)

        self.pid_write_btn = QPushButton("保存")
        self.pid_write_btn.setObjectName("connectBtn")
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
        layout.addLayout(buttons)
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
        # _flash_cancel 已在 __init__ 初始化（全局错误槽可能先于本页触发）

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

        if not load_matplotlib():
            warn = QLabel("⚠️ 未安装 matplotlib，无法绘图。\n"
                          "请在终端运行：pip install matplotlib")
            warn.setStyleSheet("color: #E04545; font-weight: bold;")
            layout.addWidget(warn)
            layout.addStretch()
            return tab

        body = QHBoxLayout()

        # ---- 左侧控制面板 ----
        left = QVBoxLayout()
        # 通道选择标题行：全选 / 清空 / 通道用途说明
        ch_head = QHBoxLayout()
        ch_head.addWidget(QLabel("选择通道："))
        self.bb_all_btn = QPushButton("全选")
        self.bb_all_btn.clicked.connect(lambda: self.on_bb_select_all(True))
        ch_head.addWidget(self.bb_all_btn)
        self.bb_none_btn = QPushButton("清空")
        self.bb_none_btn.clicked.connect(lambda: self.on_bb_select_all(False))
        ch_head.addWidget(self.bb_none_btn)
        self.bb_help_btn = QPushButton("❓ 通道用途")
        self.bb_help_btn.clicked.connect(self.on_bb_channel_help)
        ch_head.addWidget(self.bb_help_btn)
        ch_head.addStretch()
        left.addLayout(ch_head)
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

        # 日志备注标签（v0.99）：给当前日志段打标签，随文件记住
        tag_box = QGroupBox(tr("日志备注标签"))
        tag_col = QVBoxLayout(tag_box)
        self.bb_tag_edit = QLineEdit()
        self.bb_tag_edit.setPlaceholderText(
            tr("如：柳州试飞 / 换桨后 / 抖动机架"))
        tag_col.addWidget(self.bb_tag_edit)
        self.bb_tag_btn = QPushButton("🏷 " + tr("保存标签"))
        self.bb_tag_btn.setEnabled(False)
        self.bb_tag_btn.clicked.connect(self._on_bb_tag_save)
        tag_col.addWidget(self.bb_tag_btn)
        left.addWidget(tag_box)
        self._log_tags = self._load_log_tags()

        self.bb_plot_btn = QPushButton("🎨 绘制曲线")
        self.bb_plot_btn.setObjectName("connectBtn")
        self.bb_plot_btn.clicked.connect(self.on_bb_plot)
        self.bb_plot_btn.setEnabled(False)
        left.addWidget(self.bb_plot_btn)

        self.bb_fft_btn = QPushButton("📶 频谱分析")
        self.bb_fft_btn.clicked.connect(self.on_bb_fft)
        self.bb_fft_btn.setEnabled(False)
        left.addWidget(self.bb_fft_btn)

        self.bb_shot_btn = QPushButton("📷 " + tr("保存截图"))
        self.bb_shot_btn.setToolTip(
            tr("把当前图表存为 PNG，方便发到交流群讨论"))
        self.bb_shot_btn.clicked.connect(
            lambda: self._save_figure_png(self.bb_figure, "blackbox"))
        left.addWidget(self.bb_shot_btn)

        self.bb_ai_btn = QPushButton("🤖 AI 解读图表")
        self.bb_ai_btn.setToolTip(
            "把这段日志的结构化分析（噪声峰/跟踪滞后/电机饱和等）"
            "发给 AI 解读并给出调参方向")
        self.bb_ai_btn.clicked.connect(self.on_bb_ask_ai)
        self.bb_ai_btn.setEnabled(False)
        left.addWidget(self.bb_ai_btn)

        # v0.7：双日志对比（调参前/后效果叠加）
        compare_box = QGroupBox("双日志对比")
        compare_form = QVBoxLayout(compare_box)
        self.bb_compare_btn = QPushButton("📂 加载对比日志")
        self.bb_compare_btn.setToolTip(
            "加载第二段日志（如调参前的飞行），与当前日志同图对比")
        self.bb_compare_btn.clicked.connect(self.on_bb_open_compare)
        compare_form.addWidget(self.bb_compare_btn)
        self.bb_compare = QCheckBox("对比模式（当前 vs 对比日志）")
        self.bb_compare.setEnabled(False)
        self.bb_compare.stateChanged.connect(lambda _s: self.on_bb_plot())
        compare_form.addWidget(self.bb_compare)
        self.bb_compare_label = QLabel("未加载对比日志")
        self.bb_compare_label.setWordWrap(True)
        self.bb_compare_label.setStyleSheet("color: #9AA0A6;")
        compare_form.addWidget(self.bb_compare_label)
        left.addWidget(compare_box)

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
        self.bb_log_type = {}             # 日志类型判别结果（飞行/空转）
        self.bb_axes = []                 # 当前图中的子图
        self.bb_cursor_lines = []         # 游标竖线
        self.bb_plotted = []              # [(列名, 数值, 显示名)]，游标读数用
        # v0.7：对比日志（第二段日志，调参前/后对比用）
        self.bb2_time = []
        self.bb2_data = {}
        self.bb2_columns = []
        self.bb2_name = ""
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
        log_event("开始从飞控下载黑匣子数据")
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
        log_event(f"黑匣子下载完成：{path_str}")
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
        """根据日志段数量更新下拉框（单段时隐藏），条目附备注标签"""
        self.bb_session_combo.blockSignals(True)
        self.bb_session_combo.clear()
        for i, csv_path in enumerate(self.bb_sessions):
            tag = self._log_tags.get(csv_path.name, "")
            suffix = f" 🏷{tag}" if tag else ""
            self.bb_session_combo.addItem(
                f"第 {i + 1} 段（{csv_path.name}）{suffix}")
        multi = len(self.bb_sessions) > 1
        self.bb_session_combo.setVisible(multi)
        self.bb_session_label.setVisible(multi)
        self.bb_session_combo.blockSignals(False)
        self._sync_tag_edit()

    # ---- 日志备注标签（v0.99）----

    def _load_log_tags(self) -> dict:
        """读取 logs/log_tags.json（文件名 → 备注标签）"""
        import json as _json
        try:
            p = LOGS_DIR / "log_tags.json"
            if p.exists():
                return _json.loads(p.read_text(encoding="utf-8"))
        except Exception:
            pass
        return {}

    def _save_log_tags(self):
        import json as _json
        try:
            (LOGS_DIR / "log_tags.json").write_text(
                _json.dumps(self._log_tags, ensure_ascii=False, indent=1),
                encoding="utf-8")
        except Exception as e:
            self.statusBar().showMessage(f"标签保存失败：{e}", 5000)

    def _current_bb_name(self) -> str:
        idx = self.bb_session_combo.currentIndex()
        if 0 <= idx < len(self.bb_sessions):
            return Path(self.bb_sessions[idx]).name
        return ""

    def _sync_tag_edit(self):
        """切日志段/新加载时：标签编辑框跟随当前段"""
        name = self._current_bb_name()
        if hasattr(self, "bb_tag_edit"):
            self.bb_tag_edit.setText(self._log_tags.get(name, ""))
            self.bb_tag_btn.setEnabled(bool(name))

    def _on_bb_tag_save(self):
        name = self._current_bb_name()
        if not name:
            return
        tag = self.bb_tag_edit.text().strip()
        if tag:
            self._log_tags[name] = tag
        else:
            self._log_tags.pop(name, None)
        self._save_log_tags()
        self._setup_session_combo()           # 刷新下拉条目上的标签
        self.statusBar().showMessage(
            f"标签已保存：{name} → {tag or '（已清除）'}", 4000)
        log_event(f"日志标签：{name} → {tag or '（清除）'}")

    def on_bb_session_changed(self, index: int):
        """切换日志段"""
        if 0 <= index < len(self.bb_sessions):
            try:
                self._load_blackbox_file(self.bb_sessions[index])
                self._sync_tag_edit()
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
            # 悬停提示：原始列名 + 这个通道是干什么的
            tip = f"{col}\n\n{CHANNEL_HELP.get(col, CHANNEL_HELP_DEFAULT)}"
            row = QHBoxLayout()
            toggle = ToggleSwitch()
            toggle.setToolTip(tip)
            toggle.toggled.connect(lambda _on: self.on_bb_plot())  # 开关即重绘
            row.addWidget(toggle)
            name_label = QLabel(display)
            name_label.setToolTip(tip)
            row.addWidget(name_label, 1)
            row_widget = QWidget()
            row_widget.setLayout(row)
            row_widget.setToolTip(tip)
            self.bb_channel_rows.addWidget(row_widget)
            self.bb_toggles[col] = toggle
        self.bb_channel_rows.addStretch()
        # 默认打开前两个通道
        for col in self.bb_columns[:2]:
            self.bb_toggles[col].setChecked(True)
        # v0.7：自动判别日志类型（真实飞行 / 地面空转）
        try:
            self.bb_log_type = classify_log_type(
                self.bb_time, self.bb_data, self.bb_columns)
            verdict = self.bb_log_type["verdict"]
            conf = self.bb_log_type["confidence"]
            self.bb_stats_label.setText(
                f"📋 日志判别：{verdict}（置信度 {conf}%）\n"
                + "\n".join("· " + r for r in self.bb_log_type["reasons"]))
            if "空转" in verdict or "静止" in verdict:
                self.statusBar().showMessage(
                    f"注意：这段日志{verdict}，调参参考价值有限")
        except Exception:
            self.bb_log_type = {}

        self.bb_plot_btn.setEnabled(True)
        self.bb_fft_btn.setEnabled(True)
        self.bb_ai_btn.setEnabled(True)
        if "空转" not in self.bb_log_type.get("verdict", "") \
                and "静止" not in self.bb_log_type.get("verdict", ""):
            self.statusBar().showMessage("日志加载完成，点击「绘制曲线」")
        self.on_bb_plot()

    # ---------- 黑匣子：数据切片 ----------

    def _bb_selected_channels(self) -> list:
        """返回所有开关处于打开状态的通道（保持原始列顺序）"""
        return [col for col, toggle in self.bb_toggles.items()
                if toggle.isChecked()]

    def on_bb_select_all(self, checked: bool):
        """全选/清空通道（屏蔽信号批量设置，只重绘一次）"""
        if not self.bb_toggles:
            self.statusBar().showMessage("请先加载日志文件")
            return
        for toggle in self.bb_toggles.values():
            toggle.blockSignals(True)
            toggle.setChecked(checked)
            toggle.blockSignals(False)
        if checked and len(self.bb_toggles) > 16:
            self.statusBar().showMessage(
                f"已全选 {len(self.bb_toggles)} 个通道：一次最多绘制 16 个，"
                "建议用「清空」后只开需要的通道")
            return
        self.on_bb_plot()

    def on_bb_channel_help(self):
        """弹出各通道用途说明"""
        docs = []
        for col in (self.bb_columns or CHANNEL_NAMES.keys()):
            display = CHANNEL_NAMES.get(col, col)
            docs.append(f"【{display}】\n"
                        f"{CHANNEL_HELP.get(col, CHANNEL_HELP_DEFAULT)}")
        QMessageBox.information(
            self, "通道用途说明",
            "每个通道是干什么的：\n\n" + "\n\n".join(docs)
            + "\n\n提示：鼠标悬停在通道开关上也能看到这条说明。")

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
        if len(selected) > 16:
            self.statusBar().showMessage(
                f"已选择 {len(selected)} 个通道：一次最多绘制 16 个，"
                "请关闭一部分（轨道太多会挤在一起看不清）")
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

            # v0.7：对比模式——叠加第二段日志的同名通道（橙色）
            comparing = (self.bb_compare.isChecked() and self.bb2_time
                         and col in self.bb2_data)
            if comparing:
                t2, values2 = self._bb_slice(self.bb2_data[col])
                if normalize:
                    peak2 = max(abs(min(values2)), abs(max(values2)), 1e-9)
                    values2 = [v / peak2 for v in values2]
                ax.plot(t2, values2, linewidth=0.8, color="#F5A83D",
                        alpha=0.9)
                if row == 0:
                    ax.plot([], [], color="#3EC6E8", label="当前日志")
                    ax.plot([], [], color="#F5A83D",
                            label=f"对比：{self.bb2_name}")
                    ax.legend(loc="upper right", fontsize=8,
                              facecolor="#23272E", labelcolor="#E8E8E8")

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
            if comparing:
                stats_lines.append(
                    f"{display}：当前均值 {sum(values)/len(values):.1f} ｜ "
                    f"对比 {sum(values2)/len(values2):.1f}")
            else:
                stats_lines.append(
                    f"{display}：均值 {sum(values)/len(values):.1f}，"
                    f"范围 {min(values):.0f} ~ {max(values):.0f}")

        self.bb_axes[-1].set_xlabel(tr("时间 (秒)"), color="#E8E8E8")
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

    def on_bb_open_compare(self):
        """加载对比日志（v0.7）：第二段日志与当前日志同图叠加对比"""
        path_str, _ = QFileDialog.getOpenFileName(
            self, "选择对比日志（如调参前的飞行）", str(LOGS_DIR),
            "黑匣子日志 (*.bbl *.bfl *.csv);;所有文件 (*)")
        if not path_str:
            return
        path = Path(path_str)
        try:
            if path.suffix.lower() in (".bbl", ".bfl"):
                self.statusBar().showMessage("正在解码对比日志……")
                csvs = decode_blackbox(path)
                path = csvs[0]                # 对比用第一段即可
            self.bb2_time, self.bb2_data, self.bb2_columns = \
                load_blackbox_csv(path)
            self.bb2_name = path.name
            self.bb_compare_label.setText(
                f"对比日志：{self.bb2_name}"
                f"（{self.bb2_time[-1]:.1f} 秒）")
            self.bb_compare.setEnabled(True)
            self.bb_compare.setChecked(True)
            self.on_bb_plot()
            self.statusBar().showMessage(
                "对比日志已加载：青色 = 当前，橙色 = 对比")
        except BlackboxError as e:
            self.on_error(str(e))
        except Exception as e:
            self.on_error(f"对比日志读取失败：{e}")

    def on_bb_ask_ai(self):
        """把当前日志的结构化分析发给 AI 解读，自动切到 AI 页看回答"""
        if not self.bb_time:
            self.statusBar().showMessage("请先加载日志文件")
            return
        stats = analyze_blackbox_stats(
            self.bb_time, self.bb_data, self.bb_columns)
        if not stats:
            self.statusBar().showMessage("日志数据不足以分析")
            return
        selected = [CHANNEL_NAMES.get(c, c)
                    for c in self._bb_selected_channels()]
        comparing = (self.bb_compare.isChecked() and self.bb2_time)
        if comparing:
            stats2 = analyze_blackbox_stats(
                self.bb2_time, self.bb2_data, self.bb2_columns)
            prompt = (
                "请对比这两段穿越机黑匣子日志的结构化分析"
                "（通常是调参前 vs 调参后），判断哪段整体更好、"
                "具体好在哪些指标（噪声 RMS、噪声峰位置、跟踪滞后、"
                "电机饱和、电压），并给出下一步调参建议：\n"
                f"【当前日志】\n" + "\n".join(
                    f"{k}：{v}" for k, v in stats.items())
                + f"\n\n【对比日志 {self.bb2_name}】\n" + "\n".join(
                    f"{k}：{v}" for k, v in stats2.items()))
        else:
            prompt = (
                "请解读这段穿越机黑匣子日志的分析结果。先判断日志类型结论"
                "（正常飞行还是地面通电空转）是否可信、说明依据；"
                "如果是真实飞行，再指出飞机状态问题并给出调参方向"
                "（噪声峰→滤波器设置，跟踪滞后→PID/FeedForward，"
                "电机饱和→机架/重心，掉压→电池）；"
                "如果是地面空转，请说明这类数据能用来验证什么、"
                "不能用来调什么。\n"
                f"日志类型判别：{self.bb_log_type.get('verdict', '未知')}"
                f"（置信度 {self.bb_log_type.get('confidence', 0)}%）\n"
                + "\n".join("· " + r for r in
                            self.bb_log_type.get("reasons", []))
                + f"\n我正在查看的通道："
                  f"{'、'.join(selected) if selected else '（未选择）'}\n"
                + "\n".join(f"{k}：{v}" for k, v in stats.items()))
        self.sidebar.setCurrentRow(9)         # 切到 AI 助手页（v0.92 起前面多了欢迎页）
        self._ai_ask(prompt)

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

        ax.set_xlabel(tr("频率 (Hz)"), color="#E8E8E8")
        ax.set_ylabel(tr("幅度"), color="#E8E8E8")
        ax.set_title(tr("频谱分析（噪声峰位置决定滤波器截止频率）"),
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

    # ---------- 页签 3：Rates 调参（v0.5，仿 BF Rate 配置文件布局） ----------

    # BF 轴配色：ROLL 红 / PITCH 绿 / YAW 蓝
    RATE_AXES = [("ROLL", 0, "#E04545"), ("PITCH", 1, "#7CE38B"),
                 ("YAW", 2, "#3EC6E8")]
    RATE_FIELDS = [("rc_rate", "RC Rate"), ("rate", "Rate"),
                   ("expo", "RC Expo")]

    def _build_rates_tab(self) -> QWidget:
        tab = QWidget()
        layout = QHBoxLayout(tab)

        self._rc_raw = None                   # 23 字节 bytearray
        self._rates_spins = {}                # {(字段, 轴): QDoubleSpinBox}
        self._rates_max_labels = {}           # {轴: 满杆角速度标签}

        # ---- 左列：油门设置 + BF 风格 Rate 表格 + 刷新/保存 ----
        left = QVBoxLayout()

        # 油门区（对应 BF 的 油门限制/中点/Expo）
        thr_box = QGroupBox("油门")
        thr_form = QFormLayout(thr_box)
        self.thr_limit_spin = QSpinBox()
        self.thr_limit_spin.setRange(0, 100)
        self.thr_limit_spin.setSuffix(" %")
        self.thr_limit_spin.valueChanged.connect(self._on_thr_changed)
        thr_form.addRow("油门限制百分比：", self.thr_limit_spin)
        self.thr_mid_spin = QDoubleSpinBox()
        self.thr_mid_spin.setRange(0.0, 1.0)
        self.thr_mid_spin.setSingleStep(0.01)
        self.thr_mid_spin.setDecimals(2)
        self.thr_mid_spin.valueChanged.connect(self._on_thr_changed)
        thr_form.addRow("油门中点：", self.thr_mid_spin)
        self.thr_expo_spin = QDoubleSpinBox()
        self.thr_expo_spin.setRange(0.0, 1.0)
        self.thr_expo_spin.setSingleStep(0.01)
        self.thr_expo_spin.setDecimals(2)
        self.thr_expo_spin.valueChanged.connect(self._on_thr_changed)
        thr_form.addRow("油门 Expo：", self.thr_expo_spin)
        left.addWidget(thr_box)

        # Rate 表格（BF 的 基本/手动 Rate：彩色轴行 + 数字输入列）
        table_box = QGroupBox("基本 / 手动 Rate")
        grid = QGridLayout(table_box)
        grid.addWidget(QLabel(""), 0, 0)
        for col, (_field, field_name) in enumerate(self.RATE_FIELDS):
            head = QLabel(field_name)
            head.setAlignment(Qt.AlignmentFlag.AlignCenter)
            grid.addWidget(head, 0, col + 1)
        max_head = QLabel("满杆 deg/s")
        max_head.setAlignment(Qt.AlignmentFlag.AlignCenter)
        grid.addWidget(max_head, 0, 4)
        for row, (axis_name, axis, color) in enumerate(self.RATE_AXES):
            lab = QLabel(axis_name)
            lab.setStyleSheet(f"color: {color}; font-weight: bold;")
            grid.addWidget(lab, row + 1, 0)
            for col, (field, _name) in enumerate(self.RATE_FIELDS):
                spin = QDoubleSpinBox()
                spin.setRange(0.0, 2.55)
                spin.setSingleStep(0.01)
                spin.setDecimals(2)
                spin.valueChanged.connect(
                    lambda v, f=field, a=axis: self._on_rates_spin(f, a, v))
                grid.addWidget(spin, row + 1, col + 1)
                self._rates_spins[(field, axis)] = spin
            maxlab = QLabel("—")
            maxlab.setStyleSheet(f"color: {color};")
            maxlab.setAlignment(Qt.AlignmentFlag.AlignCenter)
            grid.addWidget(maxlab, row + 1, 4)
            self._rates_max_labels[axis] = maxlab
        left.addWidget(table_box)

        btns = QHBoxLayout()
        btns.addStretch()
        self.rates_reload_btn = QPushButton("刷新")
        self.rates_reload_btn.clicked.connect(self.on_tuning_reload)
        self.rates_reload_btn.setEnabled(False)
        btns.addWidget(self.rates_reload_btn)
        self.rates_write_btn = QPushButton("保存")
        self.rates_write_btn.setObjectName("connectBtn")
        self.rates_write_btn.clicked.connect(self.on_tuning_write)
        self.rates_write_btn.setEnabled(False)
        btns.addWidget(self.rates_write_btn)
        left.addLayout(btns)
        left.addStretch()
        layout.addLayout(left, 1)

        # ---- 右列：Rates 预览 + 油门曲线预览（同一张图上下两轨）----
        right = QVBoxLayout()
        curve_box = QGroupBox("Rates 预览")
        curve_layout = QVBoxLayout(curve_box)
        if load_matplotlib():
            self.rates_figure = Figure(figsize=(5, 6), facecolor="#1B1E23")
            self.rates_canvas = FigureCanvasQTAgg(self.rates_figure)
            curve_layout.addWidget(self.rates_canvas)
        else:
            curve_layout.addWidget(QLabel("未安装 matplotlib，无法绘制曲线"))
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
            for (field, axis), spin in self._rates_spins.items():
                spin.blockSignals(True)
                spin.setValue(parsed[field][axis])
                spin.blockSignals(False)
            self.thr_mid_spin.blockSignals(True)
            self.thr_mid_spin.setValue(parsed["thr_mid"])
            self.thr_mid_spin.blockSignals(False)
            self.thr_expo_spin.blockSignals(True)
            self.thr_expo_spin.setValue(parsed["thr_expo"])
            self.thr_expo_spin.blockSignals(False)
            self.thr_limit_spin.blockSignals(True)
            self.thr_limit_spin.setValue(parsed["thr_limit_pct"])
            self.thr_limit_spin.blockSignals(False)
            self._draw_rates_curve()
            if parsed.get("partial"):
                self.statusBar().showMessage(
                    "注意：该固件返回的 Rates 数据不完整，"
                    "缺失字段已用默认值显示，请勿从此页写入")
            # 安全保护：固件使用 Actual/Quick/Raceflight 等非经典 Rates 类型时，
            # 数值含义不同，从此页写入可能破坏手感配置 → 禁止写入，只允许看
            classic = parsed.get("rates_type", 0) == 0
            allowed = classic and not parsed.get("partial") \
                and self._compat_allows("rates")
            self.rates_write_btn.setEnabled(allowed)
            self.rates_write_btn.setToolTip(
                "" if allowed else
                "该固件 Rates 配置未完全适配，为安全起见禁止从此页写入")
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
            # 滤波器类型下拉
            for tkey, combo in self._filter_combos.items():
                combo.blockSignals(True)
                combo.setCurrentIndex(self._filter_raw[
                    FILTER_TYPE_FIELDS[tkey]] % len(FILTER_TYPES))
                combo.blockSignals(False)
            # 各分组开关：任一受控字段非零即视为启用
            for toggle, fields, widgets in self._filter_group_list:
                enabled = any(values.get(k, 0) > 0 for k in fields)
                toggle.blockSignals(True)
                toggle.setChecked(enabled)
                toggle.blockSignals(False)
                for w in widgets:
                    w.setEnabled(enabled)
            if values.get("partial"):
                self.statusBar().showMessage(
                    "注意：该固件返回的滤波器数据不完整，仅供参考")
            self.filter_write_btn.setEnabled(
                not values.get("partial") and self._compat_allows("filter"))
            self.filter_write_btn.setToolTip(
                "" if self.filter_write_btn.isEnabled() else
                "该固件滤波器配置未完全适配，已锁定写入（只读）")
        except (MspError, KeyError, TypeError):
            self._filter_raw = None
            self.filter_write_btn.setEnabled(False)

    def _on_rates_spin(self, field: str, axis: int, value: float):
        """Rate 数字框变化：只更新本地数据与曲线，点「保存」才写飞控"""
        if self._rc_raw is None:
            return
        set_rc_value(self._rc_raw, field, axis, value)
        self._draw_rates_curve()

    def _on_thr_changed(self):
        """油门中点/Expo/限幅变化"""
        if self._rc_raw is None:
            return
        self._rc_raw[6] = max(0, min(255,
            round(self.thr_mid_spin.value() * 100)))
        self._rc_raw[7] = max(0, min(255,
            round(self.thr_expo_spin.value() * 100)))
        self._rc_raw[15] = max(0, min(100, self.thr_limit_spin.value()))
        self._draw_rates_curve()

    def _draw_rates_curve(self):
        """按当前数值绘制 BF 风格预览：上 = Rates 曲线，下 = 油门曲线"""
        if not HAS_MPL or self._rc_raw is None:
            return
        parsed = parse_rc_tuning(bytes(self._rc_raw))
        fig = self.rates_figure
        fig.clear()

        # 上：三轴 Rates 曲线（BF 轴配色）
        ax1 = fig.add_subplot(211)
        ax1.set_facecolor("#1B1E23")
        sticks = [i / 100 for i in range(101)]
        for axis_name, axis, color in self.RATE_AXES:
            curve = [bf_rate_curve(s, parsed["rc_rate"][axis],
                                   parsed["rate"][axis],
                                   parsed["expo"][axis]) for s in sticks]
            ax1.plot([s * 100 for s in sticks], curve, color=color,
                     linewidth=1.2, label=axis_name)
            self._rates_max_labels[axis].setText(f"{curve[-1]:.0f}")
        title = tr("Rates 预览")
        if parsed.get("rates_type", 0) != 0:
            title += "（固件为非经典类型，仅供参考）"
        ax1.set_title(title, color="#3EC6E8", fontsize=10)
        ax1.set_xlabel(tr("摇杆偏转 (%)"), color="#E8E8E8", fontsize=8)
        ax1.set_ylabel(tr("角速度 (°/s)"), color="#E8E8E8", fontsize=8)
        ax1.legend(loc="upper left", fontsize=8, facecolor="#23272E",
                   labelcolor="#E8E8E8")
        ax1.grid(True, alpha=0.2, color="#9AA0A6")
        ax1.tick_params(colors="#9AA0A6", labelsize=8)
        for spine in ax1.spines.values():
            spine.set_color("#363C44")

        # 下：油门曲线预览（BF 同款：锚定中点的 expo 曲线）
        ax2 = fig.add_subplot(212)
        ax2.set_facecolor("#1B1E23")
        mid, expo = parsed["thr_mid"], parsed["thr_expo"]
        throttle = [bf_throttle_curve(s, mid, expo) for s in sticks]
        ax2.plot([s * 100 for s in sticks], [t * 100 for t in throttle],
                 color="#F5A83D", linewidth=1.2)
        ax2.plot([mid * 100], [mid * 100], "o", color="#3EC6E8",
                 markersize=5)
        ax2.set_title(tr("油门曲线预览"), color="#3EC6E8", fontsize=10)
        ax2.set_xlabel(tr("油门输入 (%)"), color="#E8E8E8", fontsize=8)
        ax2.set_ylabel(tr("油门输出 (%)"), color="#E8E8E8", fontsize=8)
        ax2.grid(True, alpha=0.2, color="#9AA0A6")
        ax2.tick_params(colors="#9AA0A6", labelsize=8)
        for spine in ax2.spines.values():
            spine.set_color("#363C44")

        fig.tight_layout()
        self.rates_canvas.draw()

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

    # ---------- 页签 4：滤波器（v0.5，仿 BF 滤波器设置布局） ----------

    def _build_filter_tab(self) -> QWidget:
        tab = QWidget()
        outer = QVBoxLayout(tab)

        hint = QLabel("截止频率 0 = 关闭该滤波器（危险！）。"
                      "建议先在「黑匣子」页做频谱分析找到噪声峰后再调整。")
        hint.setWordWrap(True)
        outer.addWidget(hint)
        # 与黑匣子频谱联动：显示最近一次 FFT 找到的噪声峰
        self.filter_peak_label = QLabel("黑匣子噪声峰：尚未做频谱分析"
                                        "（黑匣子页 → 频谱分析）")
        self.filter_peak_label.setWordWrap(True)
        self.filter_peak_label.setStyleSheet("color: #9AA0A6;")
        outer.addWidget(self.filter_peak_label)

        self._filter_raw = None               # 49 字节 bytearray
        self._filter_spins = {}               # {字段键: QSpinBox}
        self._filter_combos = {}              # {类型键: QComboBox}
        self._filter_group_list = []          # [(开关, [受控字段], [组内控件])]
        self._filter_last = {}                # 关闭前的值，重新打开时恢复
        self._filter_ranges = {k: (lo, hi)
                               for k, _n, _o, _t, lo, hi in FILTER_FIELDS}

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        content = QWidget()
        cols = QHBoxLayout(content)
        scroll.setWidget(content)
        outer.addWidget(scroll, 1)

        # ---- 左列：陀螺仪（独立于 PID 配置文件）----
        left = QVBoxLayout()
        lhead = QLabel("陀螺仪（独立于 PID 配置文件）")
        lhead.setStyleSheet("color: #3EC6E8; font-weight: bold;")
        left.addWidget(lhead)
        self._add_filter_group(left, "陀螺仪低通滤波器 1",
            ["gyro_lpf1_hz", "gyro_dyn_min", "gyro_dyn_max"],
            {"gyro_lpf1_hz": 250, "gyro_dyn_min": 250, "gyro_dyn_max": 500},
            [("最低截止频率 (Hz)", "gyro_dyn_min"),
             ("最高截止频率 (Hz)", "gyro_dyn_max"),
             ("滤波器类型", "type:gyro_lpf1_type")])
        self._add_filter_group(left, "陀螺仪低通滤波器 2",
            ["gyro_lpf2_hz"], {"gyro_lpf2_hz": 500},
            [("静态截止频率 (Hz)", "gyro_lpf2_hz"),
             ("滤波器类型", "type:gyro_lpf2_type")])
        self._add_filter_group(left, "陀螺仪陷波滤波器 1",
            ["gyro_notch1_hz", "gyro_notch1_cutoff"],
            {"gyro_notch1_hz": 400, "gyro_notch1_cutoff": 300},
            [("频率 (Hz)", "gyro_notch1_hz"),
             ("截止 (Hz)", "gyro_notch1_cutoff")])
        self._add_filter_group(left, "陀螺仪陷波滤波器 2",
            ["gyro_notch2_hz", "gyro_notch2_cutoff"],
            {"gyro_notch2_hz": 200, "gyro_notch2_cutoff": 100},
            [("频率 (Hz)", "gyro_notch2_hz"),
             ("截止 (Hz)", "gyro_notch2_cutoff")])
        self._add_filter_group(left, "陀螺仪 RPM 滤波器",
            ["rpm_harmonics", "rpm_min_hz"],
            {"rpm_harmonics": 3, "rpm_min_hz": 100},
            [("谐波数量", "rpm_harmonics"),
             ("最低频率 (Hz)", "rpm_min_hz")])
        self._add_filter_group(left, "动态陷波滤波器",
            ["notch_count"], {"notch_count": 3},
            [("陷波数量", "notch_count"),
             ("Q 因子", "notch_q"),
             ("最低频率 (Hz)", "notch_min"),
             ("最高频率 (Hz)", "notch_max")])
        left.addStretch()
        cols.addLayout(left, 1)

        # ---- 右列：D Term / 偏航（PID 配置文件关联）----
        right = QVBoxLayout()
        rhead = QLabel("D Term / 偏航（PID 配置文件关联）")
        rhead.setStyleSheet("color: #3EC6E8; font-weight: bold;")
        right.addWidget(rhead)
        self._add_filter_group(right, "D Term 低通滤波器 1",
            ["dterm_lpf1_hz", "dterm_dyn_min", "dterm_dyn_max"],
            {"dterm_lpf1_hz": 75, "dterm_dyn_min": 75, "dterm_dyn_max": 150},
            [("最低截止频率 (Hz)", "dterm_dyn_min"),
             ("最高截止频率 (Hz)", "dterm_dyn_max"),
             ("动态曲线 Expo", "dyn_expo"),
             ("滤波器类型", "type:dterm_lpf1_type")])
        self._add_filter_group(right, "D Term 低通滤波器 2",
            ["dterm_lpf2_hz"], {"dterm_lpf2_hz": 150},
            [("静态截止频率 (Hz)", "dterm_lpf2_hz"),
             ("滤波器类型", "type:dterm_lpf2_type")])
        self._add_filter_group(right, "D Term 陷波滤波器",
            ["dterm_notch_hz", "dterm_notch_cutoff"],
            {"dterm_notch_hz": 300, "dterm_notch_cutoff": 250},
            [("频率 (Hz)", "dterm_notch_hz"),
             ("截止 (Hz)", "dterm_notch_cutoff")])
        self._add_filter_group(right, "偏航低通滤波器",
            ["yaw_lpf_hz"], {"yaw_lpf_hz": 100},
            [("静态截止频率 (Hz)", "yaw_lpf_hz")])
        right.addStretch()
        cols.addLayout(right, 1)

        # ---- 底部：刷新 / 保存（BF 右下角风格）----
        btns = QHBoxLayout()
        btns.addStretch()
        self.filter_reload_btn = QPushButton("刷新")
        self.filter_reload_btn.clicked.connect(self.on_tuning_reload)
        self.filter_reload_btn.setEnabled(False)
        btns.addWidget(self.filter_reload_btn)
        self.filter_write_btn = QPushButton("保存")
        self.filter_write_btn.setObjectName("connectBtn")
        self.filter_write_btn.clicked.connect(self.on_tuning_write)
        self.filter_write_btn.setEnabled(False)
        btns.addWidget(self.filter_write_btn)
        outer.addLayout(btns)
        return tab

    def _add_filter_group(self, layout, title: str, fields: list,
                          defaults: dict, rows: list):
        """创建一个滤波器分组（仿 BF）：胶囊开关 + 数值行/类型下拉"""
        box = QGroupBox(title)
        form = QFormLayout(box)
        toggle = ToggleSwitch()
        toggle.setToolTip("关闭 = 该滤波器的频率全部写 0（与 BF 一致）")
        form.addRow("启用：", toggle)
        widgets = []
        for label, key in rows:
            if key.startswith("type:"):
                combo = QComboBox()
                combo.addItems(FILTER_TYPES)
                tkey = key[5:]
                combo.currentIndexChanged.connect(
                    lambda i, k=tkey: self._on_filter_type(k, i))
                form.addRow(label + "：", combo)
                self._filter_combos[tkey] = combo
                widgets.append(combo)
            else:
                spin = QSpinBox()
                lo, hi = self._filter_ranges[key]
                spin.setRange(lo, hi)
                spin.setMaximumWidth(110)
                spin.valueChanged.connect(
                    lambda v, k=key: self._on_filter_spin(k, v))
                form.addRow(label + "：", spin)
                self._filter_spins[key] = spin
                widgets.append(spin)
        toggle.toggled.connect(
            lambda on, f=fields, d=defaults, w=widgets:
                self._on_filter_toggle(f, d, w, on))
        self._filter_group_list.append((toggle, fields, widgets))
        layout.addWidget(box)

    def _on_filter_toggle(self, fields: list, defaults: dict,
                          widgets: list, on: bool):
        """分组开关：开 = 恢复上次值（或默认）；关 = 全部写 0（同 BF）"""
        if self._filter_raw is None:
            return
        if on:
            for key in fields:
                value = self._filter_last.get(key, defaults.get(key, 0))
                set_filter_value(self._filter_raw, key, value)
        else:
            current = parse_filter_config(bytes(self._filter_raw))
            for key in fields:
                self._filter_last[key] = current.get(key, 0)
                set_filter_value(self._filter_raw, key, 0)
        values = parse_filter_config(bytes(self._filter_raw))
        for key in fields:
            spin = self._filter_spins.get(key)
            if spin is not None:
                spin.blockSignals(True)
                spin.setValue(values[key])
                spin.blockSignals(False)
        for w in widgets:
            w.setEnabled(on)

    def _on_filter_type(self, tkey: str, index: int):
        """滤波器类型下拉变化"""
        if self._filter_raw is None:
            return
        self._filter_raw[FILTER_TYPE_FIELDS[tkey]] = index

    def _on_filter_spin(self, key: str, value: int):
        """滤波器数值变化：只更新本地数据，点「保存」才写飞控"""
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
        top_col = QVBoxLayout(top)
        top_row = QHBoxLayout()
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
        top_col.addLayout(top_row)

        # ---- 第二行：按本机性能匹配最优模型（v0.96）----
        match_row = QHBoxLayout()
        self.ai_match_btn = QPushButton("⚡ " + tr("性能匹配"))
        self.ai_match_btn.setToolTip(
            "检测本机 CPU / 内存 / 显卡，推荐最合适的调参大模型；\n"
            "模型越大分析越全面，但也越吃配置——匹配错了会卡。")
        self.ai_match_btn.clicked.connect(self.on_ai_match_hw)
        match_row.addWidget(self.ai_match_btn)
        self.ai_match_label = QLabel(
            tr("点「性能匹配」自动检测本机配置并推荐模型"))
        self.ai_match_label.setStyleSheet("color: #9AA0A6;")
        match_row.addWidget(self.ai_match_label, 1)
        self.ai_pull_btn = QPushButton("📥 " + tr("一键拉取推荐模型"))
        self.ai_pull_btn.setVisible(False)        # 检测后确有需要才出现
        self.ai_pull_btn.clicked.connect(self.on_ai_pull_recommended)
        match_row.addWidget(self.ai_pull_btn)
        top_col.addLayout(match_row)
        layout.addWidget(top)

        # ---- 快捷分析按钮 ----
        quick = QHBoxLayout()
        self.ai_autotune_btn = QPushButton("🚀 AI 全自动调参")
        self.ai_autotune_btn.setObjectName("connectBtn")
        self.ai_autotune_btn.setToolTip(
            "招牌功能：AI 读取飞控全部数据，直接算出一套新参数，\n"
            "逐项对比展示哪些加、哪些减，确认后一键写入飞控（自动备份）")
        self.ai_autotune_btn.clicked.connect(self.on_ai_autotune)
        quick.addWidget(self.ai_autotune_btn)
        self.ai_advice_btn = QPushButton("🧠 综合调参建议")
        self.ai_advice_btn.setToolTip(
            "把飞控全部数据（PID + Rates + 滤波器 + 黑匣子分析）"
            "一次性发给 AI，生成按优先级排列的调参方案")
        self.ai_advice_btn.clicked.connect(self.on_ai_tuning_advice)
        quick.addWidget(self.ai_advice_btn)
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
        # 懒加载：探测结果可能在页面构建前就到了，先缓存再应用
        cache = getattr(self, "_ai_probe_cache", None)
        if cache is not None:
            self._apply_ai_probe(*cache)
        return tab

    # ---------- AI 助手：状态检测 ----------

    def _ai_probe_and_emit(self):
        """后台线程：探测 Ollama 并把结果通过信号发回界面"""
        running, models = ollama_status()
        self.ai_probe_done.emit(running, models)

    def on_ai_probe(self, running: bool, models: list):
        """界面线程：缓存探测结果；AI 页已构建则立即刷新，
        未构建（懒加载）则等首次进入时由构建器应用（v0.99）"""
        self._ai_probe_cache = (running, models)
        if self._page_built[9]:               # 9 = AI 助手页
            self._apply_ai_probe(running, models)

    def _apply_ai_probe(self, running: bool, models: list):
        """根据探测结果刷新状态行和模型下拉框"""
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
        # 懒加载：AI 页未构建时没有状态行，只在后台探测；
        # 结果由 on_ai_probe 缓存，首次进入 AI 页时应用（v0.99）
        if self._page_built[9]:
            self.ai_status_label.setText("检测中……")
            self.ai_status_label.setStyleSheet("")
        self._run_in_thread(self._ai_probe_and_emit)

    # ---------- AI 助手：按本机性能匹配模型（v0.96）----------

    def on_ai_match_hw(self):
        """检测本机硬件并推荐调参模型；已装则选中，没装给一键拉取"""
        self.ai_match_label.setText("正在检测本机硬件（CPU / 内存 / 显卡）……")
        self.ai_match_label.setStyleSheet("color: #9AA0A6;")
        self.ai_pull_btn.setVisible(False)

        def work():
            from apex_ai import detect_hardware, recommend_model
            hw = detect_hardware()
            model, reason, desc = recommend_model(hw)
            running, models = ollama_status()
            return model, reason, desc, running, models

        def done(result):
            model, reason, desc, running, models = result
            self._ai_recommended = model
            text = f"本机：{desc} → 推荐 {model}（{reason}）"
            log_event(f"AI 性能匹配：{desc} → 推荐 {model}")
            if running and model in models:
                idx = self.ai_model_combo.findText(model)
                if idx >= 0:
                    self.ai_model_combo.setCurrentIndex(idx)
                text += "，已安装并选中 ✓"
                self.ai_match_label.setStyleSheet("color: #6FCF97;")
            elif running:
                self.ai_pull_btn.setText(f"📥 {tr('一键拉取')} {model}")
                self.ai_pull_btn.setVisible(True)
                self.ai_match_label.setStyleSheet("color: #3EC6E8;")
            else:
                text += "（Ollama 未运行，先按「安装指引」装好再拉取）"
                self.ai_match_label.setStyleSheet("color: #F5A83D;")
            self.ai_match_label.setText(text)

        self._run_simple_task(work, done, "硬件检测失败")

    def on_ai_pull_recommended(self):
        """后台执行 ollama pull 拉取推荐模型，进度轮询显示在状态栏"""
        model = getattr(self, "_ai_recommended", "")
        if not model:
            return
        self.ai_pull_btn.setEnabled(False)
        self.ai_pull_btn.setText(f"⏳ 正在拉取 {model} …")
        self.ai_chat_view.append(
            f"<span style='color:#9AA0A6;'>📥 开始下载模型 {model}，"
            f"模型较大可能要几分钟，进度见底部状态栏……</span>")
        self._pull_progress = f"正在连接 Ollama 拉取 {model} ……"
        self._pull_timer = QTimer(self)
        self._pull_timer.setInterval(600)
        self._pull_timer.timeout.connect(
            lambda: self.statusBar().showMessage(
                "📥 " + str(getattr(self, "_pull_progress", ""))[:90]))
        self._pull_timer.start()

        def work():
            try:
                import subprocess
                proc = subprocess.Popen(
                    ["ollama", "pull", model],
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, encoding="utf-8", errors="ignore",
                    creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
                for line in proc.stdout or []:
                    line = line.strip()
                    if line:
                        self._pull_progress = line   # 状态栏轮询读取
                return proc.wait()
            except Exception as e:
                return f"error:{e}"

        def done(rc):
            self._pull_timer.stop()
            self.statusBar().clearMessage()
            self.ai_pull_btn.setEnabled(True)
            if rc == 0:
                self.ai_pull_btn.setVisible(False)
                self.ai_pull_btn.setText("📥 " + tr("一键拉取推荐模型"))
                self.ai_chat_view.append(
                    f"<span style='color:#6FCF97;'>✅ 模型 {model} "
                    f"下载完成，正在刷新模型列表……</span>")
                log_event(f"AI 模型拉取完成：{model}")
                self.on_ai_refresh()
            else:
                detail = (str(rc)[6:] if str(rc).startswith("error:")
                          else getattr(self, "_pull_progress", "未知原因"))
                self.ai_pull_btn.setText("📥 " + tr("重试拉取"))
                self.ai_chat_view.append(
                    f"<span style='color:#E06C75;'>❌ 拉取失败：{detail}。"
                    f"可在终端手动执行 ollama pull {model}</span>")

        self._run_simple_task(work, done, f"拉取 {model} 失败")

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
        if not self._page_built[6]:
            self.statusBar().showMessage(
                "请先在「黑匣子」页加载一段日志，再来让 AI 分析")
            return
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

    # ---------- v0.6：AI 综合调参建议（全量数据深度联动） ----------

    def _collect_tuning_context(self) -> str:
        """汇总飞控全部调参数据 + 黑匣子结构化分析，生成给 AI 的文本"""
        parts = []
        info = self.worker.fc_info or {}
        if info:
            parts.append(
                f"【飞控】固件 {info.get('firmware', '?')}，"
                f"板子 {info.get('board', '?')}，{info.get('motors', '?')}")
        if self._pid_names:
            lines = []
            for row, name in enumerate(self._pid_names):
                vals = []
                for col in range(3):
                    item = self.pid_table.item(row, col)
                    vals.append(item.text() if item else "?")
                lines.append(f"{name}: P={vals[0]} I={vals[1]} D={vals[2]}")
            parts.append("【PID】\n" + "\n".join(lines))
        if self._rc_raw is not None:
            p = parse_rc_tuning(bytes(self._rc_raw))
            parts.append(
                "【Rates】RC Rate R/P/Y = "
                + "/".join(f"{v:.2f}" for v in p["rc_rate"])
                + "，Rate = " + "/".join(f"{v:.2f}" for v in p["rate"])
                + "，Expo = " + "/".join(f"{v:.2f}" for v in p["expo"])
                + f"，油门中点 {p['thr_mid']:.2f}，"
                  f"油门 Expo {p['thr_expo']:.2f}")
        if self._filter_raw is not None:
            f = parse_filter_config(bytes(self._filter_raw))
            parts.append(
                f"【滤波】陀螺仪低通 动态 {f['gyro_dyn_min']}-"
                f"{f['gyro_dyn_max']}Hz（静态 {f['gyro_lpf1_hz']}/"
                f"{f['gyro_lpf2_hz']}Hz），D项低通 动态 {f['dterm_dyn_min']}-"
                f"{f['dterm_dyn_max']}Hz（静态 {f['dterm_lpf1_hz']}/"
                f"{f['dterm_lpf2_hz']}Hz），动态陷波 {f['notch_count']} 个 "
                f"{f['notch_min']}-{f['notch_max']}Hz Q={f['notch_q']}，"
                f"RPM 滤波 谐波 {f['rpm_harmonics']} 最低 "
                f"{f['rpm_min_hz']}Hz")
        bb_time = getattr(self, "bb_time", None)   # 黑匣子页可能尚未构建
        if bb_time:
            stats = analyze_blackbox_stats(
                bb_time, self.bb_data, self.bb_columns)
            if stats:
                block = "【黑匣子分析】\n" + "\n".join(
                    f"{k}：{v}" for k, v in stats.items())
                bb_log_type = getattr(self, "bb_log_type", None)
                if bb_log_type:
                    block += (f"\n日志类型判别：{bb_log_type['verdict']}"
                              f"（置信度 {bb_log_type['confidence']}%）\n"
                              + "\n".join("· " + r for r in
                                          bb_log_type["reasons"]))
                parts.append(block)
        return "\n\n".join(parts)

    def on_ai_tuning_advice(self):
        """综合调参建议：全部真实数据一次性发给 AI 出方案"""
        context = self._collect_tuning_context()
        if not context:
            self.statusBar().showMessage(
                "还没有数据：请先连接飞控（最好再加载一段黑匣子日志）")
            return
        prompt = (
            "请基于下面这台穿越机的全部真实数据给出调参建议。"
            "注意：滤波器截止频率 0 表示该滤波器已关闭；"
            "电机饱和时间占比高说明机架/重心/PID 可能有问题；"
            "跟踪滞后大可以考虑加 FeedForward 或 P。\n\n" + context)
        self._ai_ask(prompt)

    # ---------- v0.93：AI 全自动调参（招牌功能） ----------

    _AUTOTUNE_SYSTEM = (
        "你是穿越机调参专家。根据用户给出的飞控全部数据，直接计算一套改进的"
        "调参参数。只输出一个 JSON 对象，不要输出任何其他文字。JSON 结构："
        '{"pid": [[P,I,D],...], '
        '"rates": {"rc_rate": [横滚,俯仰,偏航], "expo": [横滚,俯仰,偏航], '
        '"rate": [横滚,俯仰,偏航], "thr_mid": 数值, "thr_expo": 数值}, '
        '"filters": {"gyro_lpf1_hz": 数值, "gyro_lpf2_hz": 数值, '
        '"dterm_lpf1_hz": 数值, "dterm_lpf2_hz": 数值, '
        '"gyro_dyn_min": 数值, "gyro_dyn_max": 数值, '
        '"dterm_dyn_min": 数值, "dterm_dyn_max": 数值, '
        '"notch_min": 数值, "notch_max": 数值, "notch_q": 数值}, '
        '"explanation": "一两句话说明调整思路", '
        '"reasoning": "详细分析：当前数据反映出什么问题、每类参数为什么这么改、'
        '预期改善什么（分点写，每点一句话）"}'
        "规则：PID 与滤波器必须是整数；Rates 为两位小数；"
        "pid 数组长度必须和用户给的组数一致；"
        "只调整有必要调整的项，其余项照抄当前值；"
        "任何滤波器截止频率不允许为 0；"
        "P 范围 10~160，I 范围 10~160，D 范围 0~120；"
        "黑匣子数据显示噪声大就优先降 D、压滤波；跟踪滞后大就适当加 P 或 Rate。")

    def on_ai_autotune(self):
        """AI 全自动调参：全量数据 → AI 算出参数 → 逐项对比 → 确认写入"""
        model = self.ai_model_combo.currentText()
        if not model or model.startswith("（"):
            self.statusBar().showMessage(
                "AI 服务未就绪：请先启动 Ollama 并下载模型")
            return
        if not self._pid_names:
            self.statusBar().showMessage(
                "请先连接飞控读取参数（或选择「虚拟连接」体验该功能）")
            return

        # 当前值快照（校验 + 对比基准）
        n_pid = len(self._pid_names)
        snap = {"pid": [], "rates": None, "filters": None}
        for row in range(n_pid):
            vals = []
            for col in range(3):
                item = self.pid_table.item(row, col)
                text = item.text() if item else "0"
                vals.append(int(text) if text.lstrip("-").isdigit() else 0)
            snap["pid"].append(vals)
        if self._rc_raw is not None:
            p = parse_rc_tuning(bytes(self._rc_raw))
            snap["rates"] = {
                "rc_rate": [round(v, 2) for v in p["rc_rate"]],
                "expo": [round(v, 2) for v in p["expo"]],
                "rate": [round(v, 2) for v in p["rate"]],
                "thr_mid": round(p["thr_mid"], 2),
                "thr_expo": round(p["thr_expo"], 2),
            }
        if self._filter_raw is not None:
            f = parse_filter_config(bytes(self._filter_raw))
            snap["filters"] = {k: f.get(k, 0)
                               for k, *_ in FILTER_FIELDS}
        self._at_snap = snap

        context = self._collect_tuning_context()
        prompt = ("下面是这台穿越机的全部真实数据，请计算一套改进参数，"
                  "严格按系统约定的 JSON 结构输出。\n\n" + context)
        self.ai_autotune_btn.setEnabled(False)
        self.ai_chat_view.append(
            "🚀 <b>AI 全自动调参</b>：正在分析全部数据并计算新参数，"
            "请稍候……")
        log_event("AI 全自动调参：开始分析")

        import threading as _th

        def runner():
            try:
                text = chat_blocking(model, [
                    {"role": "system", "content": self._AUTOTUNE_SYSTEM},
                    {"role": "user", "content": prompt}], json_mode=True)
                self._at_result = ("ok", text)
            except Exception as e:
                self._at_result = ("err", str(e))
            QTimer.singleShot(0, self._autotune_finish)

        _th.Thread(target=runner, daemon=True).start()

    def _autotune_finish(self):
        """AI 回答到达（界面线程）：解析 → 校验 → 弹对比对话框"""
        self.ai_autotune_btn.setEnabled(True)
        tag, payload = self._at_result
        if tag == "err":
            self.ai_chat_view.append(f"❌ AI 调用失败：{payload}")
            log_event(f"AI 全自动调参失败：{payload}")
            return
        try:
            data = extract_json(payload)
        except Exception:
            self.ai_chat_view.append(
                "⚠️ AI 回答不是有效 JSON，原文如下：\n" + payload)
            return
        changes, notes = self._validate_autotune(data, self._at_snap)
        explanation = str(data.get("explanation", "")).strip()
        reasoning = str(data.get("reasoning", "")).strip()
        # AI 的理解永久留档到对话区
        if reasoning:
            self.ai_chat_view.append(
                "🧠 <b>AI 的理解</b>：\n" + reasoning.replace("\n", "<br>"))
        if not changes:
            self.ai_chat_view.append(
                "🤖 AI 认为当前参数已经比较合适，无需改动。\n" + explanation)
            return
        self._show_autotune_dialog(changes, explanation, notes, reasoning)

    def _validate_autotune(self, data: dict, snap: dict):
        """校验并钳制 AI 给出的参数，生成变化清单。

        返回 (changes, notes)。changes 元素：
        {"kind": "pid"/"rate"/"filter", "key": ..., "label": 中文名,
         "old": 当前值, "new": 建议值}
        """
        changes, notes = [], []

        def clamp(v, lo, hi):
            return max(lo, min(hi, v))

        # ---- PID ----
        ai_pid = data.get("pid")
        if isinstance(ai_pid, list) and len(ai_pid) == len(snap["pid"]):
            names = ("P", "I", "D")
            bounds = ((10, 160), (10, 160), (0, 120))
            for row, (cur, new) in enumerate(zip(snap["pid"], ai_pid)):
                if not isinstance(new, (list, tuple)) or len(new) < 3:
                    notes.append(f"第 {row + 1} 组 PID 格式异常，已跳过")
                    continue
                for col in range(3):
                    try:
                        v = int(round(float(new[col])))
                    except (TypeError, ValueError):
                        continue
                    v = clamp(v, *bounds[col])
                    if v != cur[col]:
                        changes.append({
                            "kind": "pid", "key": (row, col),
                            "label": f"{self._pid_names[row]} {names[col]}",
                            "old": cur[col], "new": v})
        elif ai_pid is not None:
            notes.append("AI 返回的 PID 组数与当前飞控不一致，PID 部分已忽略")

        # ---- Rates ----
        ai_rates = data.get("rates")
        if snap["rates"] and isinstance(ai_rates, dict):
            cur = snap["rates"]
            field_names = {"rc_rate": "RC Rate", "expo": "Expo",
                           "rate": "Rate"}
            axes = ("横滚", "俯仰", "偏航")
            for field, label in field_names.items():
                vals = ai_rates.get(field)
                if not isinstance(vals, (list, tuple)) or len(vals) < 3:
                    continue
                for axis in range(3):
                    try:
                        v = round(float(vals[axis]), 2)
                    except (TypeError, ValueError):
                        continue
                    v = clamp(v, 0.0, 2.55)
                    if abs(v - cur[field][axis]) >= 0.01:
                        changes.append({
                            "kind": "rate", "key": (field, axis),
                            "label": f"{label} {axes[axis]}",
                            "old": cur[field][axis], "new": v})
            for field, label, lo, hi in (
                    ("thr_mid", "油门中点", 0.2, 0.8),
                    ("thr_expo", "油门 Expo", 0.0, 1.0)):
                try:
                    v = round(float(ai_rates.get(field)), 2)
                except (TypeError, ValueError):
                    continue
                v = clamp(v, lo, hi)
                if abs(v - cur[field]) >= 0.01:
                    changes.append({"kind": "rate", "key": (field, -1),
                                    "label": label,
                                    "old": cur[field], "new": v})

        # ---- 滤波器 ----
        ai_filters = data.get("filters")
        if snap["filters"] and isinstance(ai_filters, dict):
            bounds = {k: (lo, hi) for k, _n, _o, _t, lo, hi in FILTER_FIELDS}
            names = {k: n for k, n, _o, _t, _l, _h in FILTER_FIELDS}
            for key, cur_v in snap["filters"].items():
                if key not in ai_filters:
                    continue
                try:
                    v = int(round(float(ai_filters[key])))
                except (TypeError, ValueError):
                    continue
                lo, hi = bounds.get(key, (0, 65535))
                v = clamp(v, lo, hi)
                # 安全红线：AI 不允许把低通/动态滤波压到 40Hz 以下
                # （等于关掉滤波，可能烧电机）——越界直接保持原值
                if ("lpf" in key or "dyn" in key) and 0 < v < 40 \
                        and cur_v >= 40:
                    notes.append(
                        f"AI 建议把{names.get(key, key)}压到 {v}Hz，"
                        f"过于激进已保持原值 {cur_v}Hz")
                    continue
                if v == 0 and cur_v != 0:
                    notes.append(
                        f"AI 建议关闭{names.get(key, key)}，已拒绝"
                        f"（关闭滤波有烧电机风险）")
                    continue
                if v != cur_v:
                    changes.append({"kind": "filter", "key": key,
                                    "label": names.get(key, key),
                                    "old": cur_v, "new": v})

        # ---- 交叉安全检查（v0.95）：单项都合法，组合起来可能有风险 ----
        pid_deltas = {(c["key"][0], c["key"][1]): c["new"] - c["old"]
                      for c in changes if c["kind"] == "pid"}
        filt_news = {c["key"]: c["new"] for c in changes
                     if c["kind"] == "filter"}
        rate_news = {c["key"]: c["new"] for c in changes
                     if c["kind"] == "rate"}
        # D 增大 且 D 项滤波同时放松 → 噪声风险叠加
        d_up = any(d > 0 for (r, c), d in pid_deltas.items() if c == 2)
        dterm_relaxed = any(filt_news.get(k, 9999) < cur
                            for k, cur in (("dterm_dyn_min", 150),
                                           ("dterm_lpf1_hz", 150))
                            if k in filt_news)
        if d_up and dterm_relaxed:
            notes.append("交叉检查：D 项增大的同时 D 项滤波被放松，"
                         "噪声风险叠加，建议试飞时留意电机温度")
        # I < P（BF 默认 I > P，抗风/姿态保持靠 I）
        for r in range(min(2, len(snap["pid"]))):
            p_new = next((c["new"] for c in changes
                          if c["kind"] == "pid" and c["key"] == (r, 0)),
                         snap["pid"][r][0])
            i_new = next((c["new"] for c in changes
                          if c["kind"] == "pid" and c["key"] == (r, 1)),
                         snap["pid"][r][1])
            if i_new < p_new:
                notes.append("交叉检查：横滚/俯仰的 I 小于 P，"
                             "抗风与姿态保持可能偏弱，试飞注意回中手感")
                break
        # 陀螺仪滤波下限过低
        if filt_news.get("gyro_dyn_min", 9999) < 150:
            notes.append("交叉检查：陀螺仪动态低通下限低于 150Hz，"
                         "高频噪声可能直达电机，桨叶损伤时风险大")
        # Rate 过高
        if any(k[0] == "rate" and v > 1.2 for k, v in rate_news.items()):
            notes.append("交叉检查：Rate 超过 1.2，满杆角速度很大，"
                         "新手建议先低油门适应手感")
        return changes, notes

    def _show_autotune_dialog(self, changes: list, explanation: str,
                              notes: list, reasoning: str = ""):
        """AI 全自动调参对比对话框：哪些加了、哪些减了、为什么这么调、
        安不安全——一次讲清楚"""
        dlg = QDialog(self)
        dlg.setWindowTitle("🚀 AI 全自动调参 · 方案对比")
        dlg.setMinimumWidth(680)
        lay = QVBoxLayout(dlg)

        if explanation:
            head = QLabel("🤖 " + explanation)
            head.setWordWrap(True)
            head.setStyleSheet("color: #3EC6E8; font-size: 14px; "
                               "padding-bottom: 4px;")
            lay.addWidget(head)

        table = QTableWidget(len(changes), 4)
        table.setHorizontalHeaderLabels(
            [tr("参数"), tr("当前值"), tr("建议值"), tr("变化")])
        table.horizontalHeader().setSectionResizeMode(
            0, QHeaderView.ResizeMode.Stretch)
        table.verticalHeader().setVisible(False)
        table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        up = down = 0
        for i, ch in enumerate(changes):
            delta = ch["new"] - ch["old"]
            if delta > 0:
                up += 1
                arrow, color = "↑", "#4CAF50"
                sign = f"+{delta:g}"
            else:
                down += 1
                arrow, color = "↓", "#E04545"
                sign = f"{delta:g}"
            table.setItem(i, 0, QTableWidgetItem(ch["label"]))
            table.setItem(i, 1, QTableWidgetItem(f"{ch['old']:g}"))
            new_item = QTableWidgetItem(f"{ch['new']:g}")
            new_item.setForeground(QColor(color))
            table.setItem(i, 2, new_item)
            d_item = QTableWidgetItem(f"{arrow} {sign}")
            d_item.setForeground(QColor(color))
            font = d_item.font()
            font.setBold(True)
            d_item.setFont(font)
            table.setItem(i, 3, d_item)
        table.setMinimumHeight(min(300, 40 + 30 * len(changes)))
        table.setMaximumHeight(300)
        lay.addWidget(table)

        summary = QLabel(
            f"共 {len(changes)} 项调整："
            f"<span style='color:#4CAF50'>↑ 增大 {up} 项</span>　"
            f"<span style='color:#E04545'>↓ 减小 {down} 项</span>")
        summary.setStyleSheet("font-size: 14px;")
        lay.addWidget(summary)

        # 🧠 AI 的理解（为什么这么调）
        if reasoning:
            why_box = QGroupBox("🧠 AI 的理解（为什么这么调）")
            why_lay = QVBoxLayout(why_box)
            why = QTextEdit()
            why.setReadOnly(True)
            why.setPlainText(reasoning)
            why.setMaximumHeight(130)
            why.setStyleSheet("color: #C8CDD3;")
            why_lay.addWidget(why)
            lay.addWidget(why_box)

        if notes:
            note_lab = QLabel("⚠️ " + "\n⚠️ ".join(notes))
            note_lab.setWordWrap(True)
            note_lab.setStyleSheet("color: #E0A545;")
            lay.addWidget(note_lab)

        # 🛡️ 安全性说明（能不能用、会不会损坏设备，如实告知）
        safe_box = QGroupBox("🛡️ 这套方案安全吗？")
        safe_lay = QVBoxLayout(safe_box)
        safe = QLabel(
            "① 所有数值经过安全钳制，滤波器永远不会被关闭（防烧电机红线）\n"
            "② 写入前自动备份全部参数到 backups/，随时一键恢复\n"
            "③ 写入后自动读回校验，写没写上看得见\n"
            "④ 只改 PID / Rates / 滤波，不动电调、电机协议等硬件设置\n"
            "⑤ 本地小模型的建议属于启发式经验，不保证最优——"
            "首次应用请拆桨检查、低空悬停试飞验证再正常飞行")
        safe.setWordWrap(True)
        safe.setStyleSheet("color: #9AA0A6;")
        safe_lay.addWidget(safe)
        lay.addWidget(safe_box)

        btns = QHBoxLayout()
        tip = QLabel("写入前会自动备份当前全部参数，可随时恢复")
        tip.setStyleSheet("color: #9AA0A6;")
        btns.addWidget(tip, 1)
        apply_btn = QPushButton("✅ 应用方案（写入飞控）")
        apply_btn.setObjectName("connectBtn")
        cancel_btn = QPushButton("❌ 取消")
        btns.addWidget(apply_btn)
        btns.addWidget(cancel_btn)
        lay.addLayout(btns)

        cancel_btn.clicked.connect(dlg.reject)

        def apply_and_close():
            self._apply_autotune(changes)
            dlg.accept()

        apply_btn.clicked.connect(apply_and_close)
        dlg.exec()

    def _apply_autotune(self, changes: list):
        """把 AI 方案落到 UI 状态并写入飞控（自动备份 + 写后读回校验）"""
        # 1) PID → 表格
        for ch in changes:
            if ch["kind"] == "pid":
                row, col = ch["key"]
                self.pid_table.setItem(
                    row, col, QTableWidgetItem(str(ch["new"])))
        # 2) Rates → 原始字节（read-modify-write）
        if self._rc_raw is not None:
            for ch in changes:
                if ch["kind"] != "rate":
                    continue
                field, axis = ch["key"]
                if axis >= 0:
                    set_rc_value(self._rc_raw, field, axis, ch["new"])
                elif field == "thr_mid":
                    self._rc_raw[6] = max(0, min(255,
                                                 round(ch["new"] * 100)))
                elif field == "thr_expo":
                    self._rc_raw[7] = max(0, min(255,
                                                 round(ch["new"] * 100)))
        # 3) 滤波器 → 原始字节
        if self._filter_raw is not None:
            for ch in changes:
                if ch["kind"] == "filter":
                    set_filter_value(self._filter_raw, ch["key"],
                                     int(ch["new"]))
        # 刷新 Rates/滤波页控件显示
        if self._rc_raw is not None and self._filter_raw is not None:
            self.on_tuning_ready({"rc_raw": list(self._rc_raw),
                                  "filter_raw": list(self._filter_raw)})
        # 4) 一个后台线程顺序写入（各自内置备份与读回校验）
        pid_values = []
        for row in range(len(self._pid_names)):
            vals = []
            for col in range(3):
                item = self.pid_table.item(row, col)
                vals.append(int(item.text()) if item else 0)
            pid_values.append(tuple(vals))
        log_event(f"AI 全自动调参：应用 {len(changes)} 项调整，开始写入")
        self.ai_chat_view.append(
            f"✅ 正在把 AI 方案写入飞控（{len(changes)} 项调整，"
            f"已自动备份）……")
        self._run_in_thread(self._autotune_write_all,
                            pid_values,
                            list(self._rc_raw) if self._rc_raw else None,
                            list(self._filter_raw)
                            if self._filter_raw else None)

    def _autotune_write_all(self, pid_values, rc_raw, filter_raw):
        """后台线程：先写 PID、再写 Rates/滤波（顺序执行，串口不打架）"""
        self.worker.write_pids(self._pid_names, pid_values, True)
        if rc_raw is not None and filter_raw is not None:
            self.worker.write_tuning(rc_raw, filter_raw)

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
        self.worker.motor_values_ready.connect(self.on_motor_values)
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
        # v0.99：简单后台任务完成派发（信号队列保证不被重负载事件流饿死）
        self.task_result_ready.connect(self._simple_task_dispatch)

    def _run_in_thread(self, func, *args):
        """通用后台线程启动器（顺手清理已结束的线程引用，防止列表无限增长）"""
        self._threads = [t for t in self._threads if t.is_alive()]
        thread = threading.Thread(target=func, args=args, daemon=True)
        self._threads.append(thread)
        thread.start()

    # ---------- 串口扫描与连接 ----------

    def refresh_ports(self):
        """扫描并列出所有可用串口（v0.93：固定附带虚拟连接入口）"""
        self.statusBar().showMessage("正在扫描串口……")
        self.port_combo.clear()
        ports = list(list_ports.comports())
        for p in ports:
            self.port_combo.addItem(f"{p.device} - {p.description}", p.device)
        # 虚拟连接（参考 BF 虚拟模式）：不插飞控也能体验全部界面
        self.port_combo.addItem(tr(VIRTUAL_PORT_LABEL), VIRTUAL_PORT)
        if not ports:
            self.port_combo.setCurrentIndex(self.port_combo.count() - 1)
            self.statusBar().showMessage(
                "未检测到串口：可插入飞控后点「刷新」，或选择虚拟连接体验")
            return
        self.statusBar().showMessage(
            f"扫描完成：发现 {len(ports)} 个串口（另含虚拟连接）")

    def on_connect_clicked(self):
        port = self.port_combo.currentData()
        if not port:
            self.statusBar().showMessage("错误：没有可用的串口")
            return
        baudrate = int(self.baud_combo.currentText())
        self.connect_button.setEnabled(False)
        self.refresh_button.setEnabled(False)
        if port == VIRTUAL_PORT:
            self.statusBar().showMessage("正在连接虚拟飞控（演示模式）……")
            log_event("正在连接虚拟飞控（演示模式）")
        else:
            self.statusBar().showMessage(f"正在连接 {port} @ {baudrate}……")
            log_event(f"正在连接 {port} @ {baudrate}")
        self._run_in_thread(self.worker.connect_and_query, port, baudrate)

    def on_disconnect_clicked(self):
        # 先停定时器，再停电机（避免轮询线程与停电机命令争用串口）
        self.poll_timer.stop()
        self.fast_timer.stop()
        self._stop_motors_safely()            # 断开前把电机停掉
        self.worker.close_port()
        self._set_disconnected_ui()
        self.statusBar().showMessage("已断开连接，串口已释放")
        log_event("已断开连接")

    # ---------- 实时轮询 ----------

    def _poll_once(self):
        """慢通道定时器触发：上一轮没跑完就跳过。
        用线程存活判断代替布尔标志——布尔标志的读写跨线程无锁，
        定时器和后台线程可能同时看到 False 而重复启动轮询（竞态）。"""
        if not self.worker.is_connected:
            return
        if self._poll_thread is not None and self._poll_thread.is_alive():
            return
        self._poll_thread = threading.Thread(
            target=self.worker.poll_status, daemon=True)
        self._poll_thread.start()

    def _poll_fast_once(self):
        """快通道定时器触发：同理，上一轮没跑完就跳过"""
        if not self.worker.is_connected:
            return
        if self._poll_fast_thread is not None \
                and self._poll_fast_thread.is_alive():
            return
        self._poll_fast_thread = threading.Thread(
            target=self.worker.poll_fast, daemon=True)
        self._poll_fast_thread.start()

    # ---------- 信号槽 ----------

    def on_connected(self, info: dict):
        self.firmware_label.setText(info.get("firmware", "未知"))
        board_text = info.get("board", "未知")
        if isinstance(self.worker.serial_port, VirtualSerial):
            board_text += "（演示模式）"          # v0.93：虚拟连接标识
            self.statusBar().showMessage(
                "虚拟模式：数据为仿真，写入不会触碰真实飞控", 6000)
        self.board_label.setText(board_text)
        self.motors_label.setText(info.get("motors", "未知"))
        log_event(f"已连接：{info.get('firmware', '未知')} / "
                  f"{info.get('board', '未知')}")
        self.disconnect_button.setEnabled(True)
        for btn in (self.pid_reload_btn, self.pid_write_btn,
                    self.pid_backup_btn, self.pid_restore_btn,
                    self.rates_reload_btn, self.filter_reload_btn,
                    self.preset_save_btn, self.preset_apply_btn):
            btn.setEnabled(True)
        if self._page_built[6]:               # 黑匣子页可能尚未构建（懒加载）
            self.bb_flash_btn.setEnabled(True)   # 连接后允许从飞控下载黑匣子
        self.poll_timer.start()               # 开始慢通道轮询
        self.fast_timer.start()               # 开始快通道轮询（姿态 10 帧/秒）

        # v0.8：兼容性评估 —— 非 BF 固件/未验证版本给出提示并锁定风险写入
        self._compat = compatibility_report(info)
        msgs = self._compat["messages"]
        self.compat_label.setText("⚠ " + "\n⚠ ".join(msgs) if msgs else "")
        self.compat_label.setVisible(bool(msgs))
        feats = self._compat["features"]
        if not feats.get("rates", True):
            self.rates_write_btn.setEnabled(False)
            self.rates_write_btn.setToolTip(
                "该固件的 Rates 字节布局未适配，已锁定写入（只读）")
        if not feats.get("filter", True):
            self.filter_write_btn.setEnabled(False)
            self.filter_write_btn.setToolTip(
                "该固件的滤波器字节布局未适配，已锁定写入（只读）")
        if not feats.get("presets", True):
            self.preset_apply_btn.setEnabled(False)
            self.preset_apply_btn.setToolTip(
                "预设包含 Rates/滤波布局，该固件未适配，已锁定应用")
        if msgs:
            level_name = {"limited": "受限", "unknown": "未知"}.get(
                self._compat["level"], self._compat["level"])
            self.statusBar().showMessage(f"已连接（兼容模式：{level_name}）")
        else:
            self.statusBar().showMessage("已连接")

    def _compat_allows(self, feature: str) -> bool:
        """兼容性报告是否允许使用某功能（未评估时默认允许）"""
        compat = getattr(self, "_compat", None)
        if not compat:
            return True
        return compat["features"].get(feature, True)

    def on_pid_ready(self, names: list, values: list):
        """PID 数据到达：重建表格（连接、写入后、恢复后都会触发）"""
        self._pid_names = names
        self.pid_table.setRowCount(len(names))
        self.pid_table.setVerticalHeaderLabels(names)
        # BF 轴配色：ROLL 红 / PITCH 绿 / YAW 蓝（只染前三行）
        for row, (prefix, color) in enumerate(
                [("Roll", "#E04545"), ("Pitch", "#7CE38B"),
                 ("Yaw", "#3EC6E8")]):
            if row < len(names) and names[row].startswith(prefix):
                item = self.pid_table.verticalHeaderItem(row)
                if item:
                    item.setForeground(QColor(color))
        for row, (p, i_val, d) in enumerate(values):
            for col, v in enumerate((p, i_val, d)):
                self.pid_table.setItem(row, col, QTableWidgetItem(str(v)))
        self.connect_button.setEnabled(True)
        self.refresh_button.setEnabled(True)

    def on_status_ready(self, data: dict):
        """慢通道数据到达：更新电源和飞控状态区域"""
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
        """快通道数据到达：更新人工地平线"""
        attitude = data.get("attitude")
        if attitude:
            roll, pitch, yaw = attitude
            self.horizon.set_attitude(roll, pitch)
            self.attitude_label.setText(
                f"横滚 {roll:.1f}° ｜ 俯仰 {pitch:.1f}° ｜ 航向 {yaw:.0f}°")

    def on_write_done(self, message: str):
        self.statusBar().showMessage(message.splitlines()[0])
        self.refresh_preset_list()            # 保存预设后刷新列表
        log_event(message.splitlines()[0])
        QMessageBox.information(self, "ApexFlight", message)

    def on_error(self, message: str):
        # 闪存下载失败/被取消时：恢复按钮文字和轮询定时器
        if self._flash_cancel is not None:
            self._flash_cancel = None
            if self._page_built[6]:           # 黑匣子页可能尚未构建
                self.bb_flash_btn.setText("📥 从飞控下载")
            self._resume_polling_after_flash()
        self.statusBar().showMessage(f"错误：{message.splitlines()[0]}")
        log_event(f"错误：{message.splitlines()[0]}")
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
        self._compat = None
        self.compat_label.setVisible(False)
        if self._page_built[6]:               # 黑匣子页可能尚未构建（懒加载）
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
    # 高 DPI 适配（v0.8）：分数缩放（125%/150%）下界面不模糊
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    app = QApplication(sys.argv)
    app.setApplicationName("ApexFlight")
    if ICON_PATH.exists():
        app.setWindowIcon(QIcon(str(ICON_PATH)))

    # 启动画面（v0.91）：exe 启动后立刻给出反馈，分步显示加载进度。
    # 图标绘在深色底上，底部留文字区显示当前加载步骤。
    from PyQt6.QtWidgets import QSplashScreen
    pix = QPixmap(380, 300)
    pix.fill(QColor("#16181D"))
    painter = QPainter(pix)
    if ICON_PATH.exists():
        icon = QPixmap(str(ICON_PATH)).scaled(
            150, 150, Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation)
        painter.drawPixmap((380 - icon.width()) // 2, 30, icon)
    painter.setPen(QColor("#3EC6E8"))
    painter.drawText(pix.rect().adjusted(0, 190, 0, -60),
                     Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignTop,
                     "ApexFlight")
    painter.end()

    def splash_msg(text: str):
        splash.showMessage(
            f"v{i18n.APP_VERSION}    {text}",
            Qt.AlignmentFlag.AlignBottom | Qt.AlignmentFlag.AlignHCenter,
            QColor("#9AA0A6"))
        app.processEvents()

    splash = QSplashScreen(pix)
    splash.show()
    splash_msg("正在加载图表组件……")
    load_matplotlib()                         # 延迟加载 matplotlib
    splash_msg("正在构建界面……")
    window = MainWindow()
    window.show()
    splash.finish(window)
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
