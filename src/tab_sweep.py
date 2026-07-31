# -*- coding: utf-8 -*-
"""扫频调参页（v0.99 从 main.py 拆出）：系统辨识 + 伯德图 + 建议"""

from pathlib import Path

from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import (
    QComboBox, QFileDialog, QGroupBox, QHBoxLayout, QHeaderView,
    QLabel, QMessageBox, QPushButton, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget)

from apex_fc import LOGS_DIR
from apex_i18n import tr
from apex_log import log_event


def _mpl():
    """延迟取 main 模块的 matplotlib 加载器与类（避免循环导入）"""
    import main as _m
    _m.load_matplotlib()
    return (_m.HAS_MPL, _m.Figure, _m.FigureCanvasQTAgg,
            _m.NavigationToolbar2QT)

class SweepTabMixin:
    """扫频调参页全部 UI 与处理器（self 即 MainWindow）"""
    def _build_sweep_tab(self) -> QWidget:
        """扫频精准调参页：从黑匣子日志估计频率响应（Welch 互谱法），
        带宽/相位裕度/灵敏度/谐振全部实测计算，建议逐条带公式依据"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        self._sweep_csv = None
        self._sweep_sugs = []

        # ---- 顶部：原理说明 + 采集指导 + 日志选择 ----
        guide = QGroupBox(tr("扫频精准调参（实测数据 + 数学推导，不用 AI 猜）"))
        g = QVBoxLayout(guide)
        guide_text = QLabel(
            "原理：用黑匣子日志的 setpoint→gyro 数据估计三轴频率响应"
            "（与 BF2026 Chirp Autotune 同源的 Welch 互谱系统辨识），"
            "实测带宽 / 相位裕度 / 灵敏度峰值 / 谐振峰，再按公式推导建议。\n"
            "采集：飞一段约 20 秒——3 次油门斜坡 + 各轴 2~3 次干脆的"
            "翻滚/甩杆（动作越干脆相干性越高）；"
            "刷 BF2026 固件开 Chirp 模式采集效果最佳。")
        guide_text.setWordWrap(True)
        guide_text.setStyleSheet("color: #9AA0A6;")
        g.addWidget(guide_text)
        bar = QHBoxLayout()
        self.sweep_open_btn = QPushButton("📂 " + tr("打开日志"))
        self.sweep_open_btn.clicked.connect(self._sweep_open_log)
        bar.addWidget(self.sweep_open_btn)
        self.sweep_usebb_btn = QPushButton("📈 " + tr("用黑匣子页当前日志"))
        self.sweep_usebb_btn.setToolTip(
            "直接使用「黑匣子」页已加载的日志段，不用重新选择文件")
        self.sweep_usebb_btn.clicked.connect(self._sweep_use_bb)
        bar.addWidget(self.sweep_usebb_btn)
        self.sweep_analyze_btn = QPushButton("🌀 " + tr("开始分析"))
        self.sweep_analyze_btn.setObjectName("connectBtn")
        self.sweep_analyze_btn.setEnabled(False)
        self.sweep_analyze_btn.clicked.connect(self._sweep_analyze)
        bar.addWidget(self.sweep_analyze_btn)
        # v1.0：三轴独立/叠加查看
        bar.addWidget(QLabel(tr("查看轴：")))
        self.sweep_axis_combo = QComboBox()
        self.sweep_axis_combo.addItems(
            [tr("全部叠加"), tr("横滚"), tr("俯仰"), tr("偏航")])
        self.sweep_axis_combo.currentIndexChanged.connect(
            self._sweep_axis_changed)
        bar.addWidget(self.sweep_axis_combo)
        self.sweep_shot_btn = QPushButton("📷 " + tr("保存截图"))
        self.sweep_shot_btn.setToolTip(
            tr("把当前伯德图存为 PNG，方便发到交流群讨论"))
        self.sweep_shot_btn.clicked.connect(
            lambda: self._save_figure_png(self.sweep_figure, "sweep"))
        bar.addWidget(self.sweep_shot_btn)
        self.sweep_file_label = QLabel(tr("未选择日志"))
        self.sweep_file_label.setStyleSheet("color: #9AA0A6;")
        bar.addWidget(self.sweep_file_label, 1)
        g.addLayout(bar)
        layout.addWidget(guide)

        # ---- 中部：频率响应图（幅值/相位/相干性）+ 三轴指标表 ----
        mid = QHBoxLayout()
        HAS_MPL, Figure, FigureCanvasQTAgg, \
            NavigationToolbar2QT = _mpl()
        if HAS_MPL:
            self.sweep_figure = Figure(figsize=(7, 7.2),
                                       facecolor="#14171B")
            self.sweep_canvas = FigureCanvasQTAgg(self.sweep_figure)
            self.sweep_toolbar = NavigationToolbar2QT(
                self.sweep_canvas, tab)
            # v1.0：鼠标悬浮读取坐标（频率/增益/相位）
            self._sweep_hover_data = []           # [{name,color,f,mag,phase}]
            self._sweep_hover_ann = None
            self.sweep_canvas.mpl_connect(
                "motion_notify_event", self._sweep_on_hover)
            chart_col = QVBoxLayout()
            chart_col.addWidget(self.sweep_toolbar)
            chart_col.addWidget(self.sweep_canvas, 1)
            mid.addLayout(chart_col, 3)
        else:
            mid.addWidget(QLabel("⚠️ 未安装 matplotlib，无法绘图"), 3)
        right = QVBoxLayout()
        right.addWidget(QLabel(tr("三轴辨识指标")))
        self.sweep_metrics = QTableWidget(3, 6)
        self.sweep_metrics.setHorizontalHeaderLabels(
            ["轴", "带宽 Hz", "相位裕度 °", "灵敏度峰值", "相干性", "谐振峰 Hz"])
        self.sweep_metrics.verticalHeader().setVisible(False)
        self.sweep_metrics.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch)
        self.sweep_metrics.setEditTriggers(
            QTableWidget.EditTrigger.NoEditTriggers)
        right.addWidget(self.sweep_metrics, 1)
        mid.addLayout(right, 2)
        layout.addLayout(mid, 3)

        # ---- 底部：精准建议表 + 应用 ----
        sug_box = QGroupBox(tr("精准调参建议（每条都带数学依据）"))
        sug_col = QVBoxLayout(sug_box)
        self.sweep_sug_table = QTableWidget(0, 6)
        self.sweep_sug_table.setHorizontalHeaderLabels(
            ["参数", "轴", "当前值", "建议值", "变化", "依据"])
        self.sweep_sug_table.verticalHeader().setVisible(False)
        self.sweep_sug_table.horizontalHeader().setSectionResizeMode(
            5, QHeaderView.ResizeMode.Stretch)
        for c in range(5):
            self.sweep_sug_table.horizontalHeader().setSectionResizeMode(
                c, QHeaderView.ResizeMode.ResizeToContents)
        self.sweep_sug_table.setEditTriggers(
            QTableWidget.EditTrigger.NoEditTriggers)
        sug_col.addWidget(self.sweep_sug_table, 1)
        apply_row = QHBoxLayout()
        self.sweep_apply_btn = QPushButton("⚡ " + tr("应用 PID 建议到飞控"))
        self.sweep_apply_btn.setObjectName("connectBtn")
        self.sweep_apply_btn.setEnabled(False)
        self.sweep_apply_btn.setToolTip(
            "把带宽失衡类建议按百分比落实到 PID 表（P/D 同步缩放），\n"
            "写入前自动备份、写后读回校验；滤波器类建议请按建议频率手动调整")
        self.sweep_apply_btn.clicked.connect(self._sweep_apply)
        apply_row.addWidget(self.sweep_apply_btn)
        note = QLabel(tr("滤波器类建议请到「滤波器」页按建议频率手动调整；"
                         "首次应用后请拆桨低空试飞验证"))
        note.setStyleSheet("color: #9AA0A6; font-size: 13px;")
        apply_row.addWidget(note, 1)
        sug_col.addLayout(apply_row)
        layout.addWidget(sug_box, 2)
        return tab


    def _sweep_open_log(self):
        """选择日志：.bbl/.bfl 解码后取最后一段（最近一次飞行）"""
        path_str, _ = QFileDialog.getOpenFileName(
            self, tr("选择黑匣子日志"), str(LOGS_DIR),
            tr("黑匣子日志 (*.bbl *.bfl *.csv);;所有文件 (*)"))
        if not path_str:
            return
        p = Path(path_str)
        if p.suffix.lower() == ".csv":
            self._sweep_csv = p
        else:
            from apex_blackbox import decode_blackbox
            self.statusBar().showMessage("正在解码日志……")
            try:
                csvs = decode_blackbox(p)
            except Exception as e:
                QMessageBox.warning(self, "ApexFlight", f"解码失败：{e}")
                return
            if not csvs:
                QMessageBox.warning(self, "ApexFlight", "解码没有产出任何日志段")
                return
            self._sweep_csv = Path(csvs[-1])     # 最后一段 = 最近一次飞行
            if len(csvs) > 1:
                self.statusBar().showMessage(
                    f"共 {len(csvs)} 段飞行记录，已选最后一段（最近一次）", 6000)
        self.sweep_file_label.setText(self._sweep_csv.name)
        self.sweep_analyze_btn.setEnabled(True)


    def _sweep_use_bb(self):
        """直接用黑匣子页当前加载的日志段，省去重复选择"""
        sessions = getattr(self, "bb_sessions", None)   # 懒加载：页未建则无
        if not sessions:
            self.statusBar().showMessage(
                "黑匣子页还没有加载日志，先去那边打开或从飞控下载", 5000)
            return
        idx = self.bb_session_combo.currentIndex()
        if not (0 <= idx < len(sessions)):
            idx = 0
        self._sweep_csv = Path(sessions[idx])
        self.sweep_file_label.setText(self._sweep_csv.name)
        self.sweep_analyze_btn.setEnabled(True)


    def _sweep_analyze(self):
        """后台线程做系统辨识，UI 线程回填图表/指标/建议"""
        csv_path = self._sweep_csv
        if csv_path is None:
            return
        self.sweep_analyze_btn.setEnabled(False)
        self.statusBar().showMessage("正在做系统辨识（Welch 互谱平均）……")
        # 连着飞控时把当前 PID 带给建议表填"当前值"
        current_pids = None
        if self.worker.is_connected and self._pid_names:
            try:
                vals = [tuple(int(self.pid_table.item(row, c).text())
                              for c in range(3)) for row in range(3)]
                current_pids = {"roll": vals[0], "pitch": vals[1],
                                "yaw": vals[2]}
            except Exception:
                current_pids = None

        def work():
            from apex_sweep import analyze_log
            try:
                return analyze_log(csv_path, current_pids)
            except Exception as e:
                return {"error": str(e)}

        def done(result):
            self.sweep_analyze_btn.setEnabled(True)
            if "error" in result:
                self.statusBar().showMessage(
                    f"扫频分析失败：{result['error']}", 8000)
                log_event(f"扫频分析失败：{result['error']}")
                return
            self._sweep_result = result
            self._sweep_show_result(result)
            log_event(f"扫频分析完成：{csv_path.name}")
            self.statusBar().showMessage("扫频分析完成 ✅", 4000)

        self._run_simple_task(work, done, "扫频分析失败")


    def _sweep_show_result(self, result: dict):
        """把辨识结果画到图/表上"""
        axes = result["axes"]
        colors = {"横滚": "#3EC6E8", "俯仰": "#F5A83D", "偏航": "#6FCF97"}

        # ---- 频率响应三子图（v1.0 专业频域分析）----
        if hasattr(self, "sweep_canvas"):
            sel = self.sweep_axis_combo.currentText() \
                if hasattr(self, "sweep_axis_combo") else "全部叠加"
            fig = self.sweep_figure
            fig.clear()
            ax1 = fig.add_subplot(311)
            ax2 = fig.add_subplot(312, sharex=ax1)
            ax3 = fig.add_subplot(313, sharex=ax1)
            self._sweep_axes = (ax1, ax2, ax3)
            self._sweep_hover_data = []
            if self._sweep_hover_ann is not None:
                self._sweep_hover_ann = None
            legend_done = False
            for name, m in axes.items():
                if "mag_db" not in m:
                    continue
                if sel != "全部叠加" and name != sel:
                    continue
                f = m["freqs"]
                c = colors.get(name, "#FFFFFF")
                mask = (f >= 2) & (f <= min(600, m["fs"] / 2))
                ax1.plot(f[mask], m["mag_db"][mask], color=c, lw=1.3,
                         label=name)
                ax2.plot(f[mask], m["phase_deg"][mask], color=c, lw=1.0)
                ax3.plot(f[mask], m["coh"][mask], color=c, lw=1.0)
                self._sweep_hover_data.append({
                    "name": name, "color": c, "f": f[mask],
                    "mag": m["mag_db"][mask], "phase": m["phase_deg"][mask]})
                plateau = m.get("plateau_db", 0)
                bw = m.get("bandwidth_hz")
                # 谐振峰：红圈 + 频率标注 + 危险区间底纹
                for f_r, m_r in m.get("resonances", []):
                    peak_db = plateau + m_r
                    ax1.axvspan(f_r * 0.92, f_r * 1.08,
                                color="#E06C75", alpha=0.08)
                    ax1.plot([f_r], [peak_db], "o", color="#E06C75",
                             ms=9, mfc="none", mew=1.8)
                    ax1.annotate(f"{f_r:.0f}Hz 共振",
                                 xy=(f_r, peak_db),
                                 xytext=(0, 10), textcoords="offset points",
                                 ha="center", fontsize=8, color="#E06C75")
                if bw:
                    # 截止频率（-3dB 带宽）：竖线 + 幅值点标注
                    ax1.axvline(bw, color=c, ls="--", alpha=0.45)
                    ax1.plot([bw], [plateau - 3], "s", color=c, ms=6)
                    ax1.annotate(f"带宽 {bw:.0f}Hz", xy=(bw, plateau - 3),
                                 xytext=(6, -14), textcoords="offset points",
                                 fontsize=8, color=c)
                    # 相位裕度点：带宽频率处的相位 + PM 标注
                    pm = m.get("phase_margin_deg")
                    if pm is not None:
                        i_bw = int(abs(f - bw).argmin())
                        ph_at_bw = float(m["phase_deg"][i_bw])
                        ax2.plot([bw], [ph_at_bw], "D", color=c, ms=6)
                        ax2.annotate(f"PM {pm:.0f}°", xy=(bw, ph_at_bw),
                                     xytext=(6, 6),
                                     textcoords="offset points",
                                     fontsize=8, color=c)
                        ax2.axvline(bw, color=c, ls="--", alpha=0.30)
                legend_done = True
            ax1.axhline(-3, color="#E06C75", ls=":", lw=0.9)
            ax1.set_ylabel("幅值 dB")
            ax2.set_ylabel("相位 °")
            ax3.set_ylabel("相干性")
            ax3.set_ylim(0, 1.05)
            ax3.set_xlabel("频率 Hz")
            for ax in (ax1, ax2, ax3):
                ax.set_xscale("log")
                ax.set_xlim(2, 600)
                ax.grid(True, alpha=0.25)
                ax.set_facecolor("#14171B")
                ax.tick_params(colors="#C9CDD3", labelsize=9)
                ax.xaxis.label.set_color("#C9CDD3")
                ax.yaxis.label.set_color("#C9CDD3")
                for sp in ax.spines.values():
                    sp.set_color("#3A3F47")
            if legend_done:
                leg = ax1.legend(loc="upper right", fontsize=9)
                leg.get_frame().set_facecolor("#1A1D22")
                leg.get_frame().set_edgecolor("#3A3F47")
                for t in leg.get_texts():
                    t.set_color("#C9CDD3")
            fig.tight_layout()
            self.sweep_canvas.draw()

        # ---- 指标表 ----
        for r, name in enumerate(("横滚", "俯仰", "偏航")):
            m = axes.get(name, {})
            if "error" in m:
                cells = [name, "—", "—", "—", "—", m["error"]]
            else:
                sp_db = m.get("sensitivity_peak_db")
                cells = [
                    name,
                    str(m.get("bandwidth_hz") or "—"),
                    str(m.get("phase_margin_deg") or "—"),
                    (f"{sp_db}dB@{m.get('sensitivity_peak_hz')}Hz"
                     if sp_db is not None else "—"),
                    (f"{m['coherence']:.0%}"
                     if m.get("coherence") is not None else "—"),
                    (", ".join(str(fr) for fr, _ in m.get("resonances", []))
                     or "—"),
                ]
            for c, text in enumerate(cells):
                item = QTableWidgetItem(str(text))
                if c == 0:
                    item.setForeground(QColor(colors[name]))
                self.sweep_metrics.setItem(r, c, item)

        # ---- 建议表 + 应用按钮 ----
        sugs = result["suggestions"]
        self._sweep_sugs = sugs
        self.sweep_sug_table.setRowCount(len(sugs))
        level_color = {"danger": "#E06C75", "action": "#F5A83D",
                       "info": "#9AA0A6"}
        for r, s in enumerate(sugs):
            vals = [s["param"], s["axis"], s["current"], s["suggested"],
                    s["change"], s["reason"]]
            for c, text in enumerate(vals):
                item = QTableWidgetItem(str(text))
                item.setForeground(
                    QColor(level_color.get(s["level"], "#C9CDD3")))
                item.setToolTip(str(s["reason"]))
                self.sweep_sug_table.setItem(r, c, item)
        has_pid = any(s["param"] == "PID" for s in sugs)
        self.sweep_apply_btn.setEnabled(
            has_pid and self.worker.is_connected and bool(self._pid_names))


    def _sweep_axis_changed(self):
        """切换 全部叠加/单轴 视图（有结果时立即重绘）"""
        if getattr(self, "_sweep_result", None):
            self._sweep_show_result(self._sweep_result)


    def _sweep_on_hover(self, event):
        """鼠标悬浮：读取最近曲线点的 频率/增益/相位 坐标"""
        axes = getattr(self, "_sweep_axes", None)
        if not axes or not self._sweep_hover_data:
            return
        ax1, ax2, _ = axes
        if event.inaxes not in (ax1, ax2) or event.xdata is None:
            if self._sweep_hover_ann is not None:
                self._sweep_hover_ann.remove()
                self._sweep_hover_ann = None
                self.sweep_canvas.draw_idle()
            return
        import numpy as _np
        fx = max(event.xdata, 1e-6)
        best = None
        for ds in self._sweep_hover_data:
            i = int(abs(_np.log10(ds["f"]) - _np.log10(fx)).argmin())
            y = ds["mag"][i] if event.inaxes is ax1 else ds["phase"][i]
            # 屏幕坐标距离，选离鼠标最近的那条曲线
            px, py = event.inaxes.transData.transform((ds["f"][i], y))
            dist = (px - event.x) ** 2 + (py - event.y) ** 2
            if best is None or dist < best[0]:
                best = (dist, ds, i, y)
        if best is None:
            return
        _, ds, i, y = best
        if event.inaxes is ax1:
            text = (f"{ds['name']}\n{ds['f'][i]:.1f} Hz\n"
                    f"幅值 {y:.1f} dB")
        else:
            text = (f"{ds['name']}\n{ds['f'][i]:.1f} Hz\n"
                    f"相位 {y:.1f}°")
        if self._sweep_hover_ann is not None:
            self._sweep_hover_ann.remove()
        self._sweep_hover_ann = event.inaxes.annotate(
            text, xy=(ds["f"][i], y), xytext=(12, 12),
            textcoords="offset points", fontsize=9, color="#FFFFFF",
            bbox=dict(boxstyle="round,pad=0.35", fc="#1A1D22",
                      ec=ds["color"], lw=1.2),
            arrowprops=dict(arrowstyle="-", color=ds["color"], lw=0.9))
        self.sweep_canvas.draw_idle()


    def _sweep_apply(self):
        """把 PID 类建议（P/D 百分比缩放）落实到 PID 表并安全写入"""
        sugs = [s for s in self._sweep_sugs if s["param"] == "PID"]
        if not sugs:
            return
        import re as _re
        axis_row = {"横滚": 0, "俯仰": 1, "偏航": 2}
        plan = []
        for s in sugs:
            m = _re.search(r"\+(\d+)%", s["change"])
            row = axis_row.get(s["axis"])
            if not m or row is None or row >= self.pid_table.rowCount():
                continue
            k = 1 + int(m.group(1)) / 100
            for col, label in ((0, "P"), (2, "D")):
                item = self.pid_table.item(row, col)
                if not item:
                    continue
                old = int(item.text())
                new = max(0, min(255, round(old * k)))
                if new != old:
                    plan.append((row, col, s["axis"], label, old, new))
        if not plan:
            self.statusBar().showMessage("没有可应用的 PID 数值变化", 4000)
            return
        lines = "\n".join(f"{a} {l}：{o} → {n}"
                          for _, _, a, l, o, n in plan)
        reply = QMessageBox.question(
            self, tr("确认写入"),
            "扫频建议将修改以下 PID（写入前自动备份，写后读回校验）：\n\n"
            f"{lines}\n\n确定继续吗？")
        if reply != QMessageBox.StandardButton.Yes:
            return
        for row, col, _, _, _, new in plan:
            self.pid_table.item(row, col).setText(str(new))
        values = [tuple(int(self.pid_table.item(row, c).text())
                        for c in range(3))
                  for row in range(len(self._pid_names))]
        log_event(f"扫频调参：应用 {len(plan)} 项 PID 调整")
        self._run_in_thread(self.worker.write_pids,
                            self._pid_names, values, True)

