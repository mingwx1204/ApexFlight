# -*- coding: utf-8 -*-
"""适飞地图页（v0.99 从 main.py 拆出）：底图/标记/定位/搜索/UOM"""

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import (
    QComboBox, QHBoxLayout, QLabel, QLineEdit, QMessageBox,
    QPushButton, QVBoxLayout, QWidget)

from apex_i18n import tr
from apex_log import log_event

class MapTabMixin:
    """适飞地图页全部 UI 与处理器（self 即 MainWindow）"""
    def _build_map_tab(self) -> QWidget:
        """适飞地图页：卫星图底图切换、起飞点标记与坐标复制、UOM 入口"""
        from apex_map import SlippyMapWidget, TILE_SOURCES, gcj02_to_wgs84
        self._gcj02_to_wgs84 = gcj02_to_wgs84   # 处理器里复用

        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        # 顶栏：底图切换 + 缩放 + 定位 + UOM
        bar = QHBoxLayout()
        bar.addWidget(QLabel(tr("底图：")))
        self.map_source_combo = QComboBox()
        self.map_source_combo.addItems(list(TILE_SOURCES.keys()))
        self.map_source_combo.setCurrentText("卫星+路网标注")   # 与控件默认一致
        self.map_source_combo.setMinimumWidth(150)
        bar.addWidget(self.map_source_combo)
        zoom_in = QPushButton("＋ " + tr("放大"))
        zoom_in.clicked.connect(lambda: self.map_widget.zoom_in())
        zoom_out = QPushButton("－ " + tr("缩小"))
        zoom_out.clicked.connect(lambda: self.map_widget.zoom_out())
        bar.addWidget(zoom_in)
        bar.addWidget(zoom_out)
        locate_btn = QPushButton("📍 " + tr("定位我的城市"))
        locate_btn.clicked.connect(self._map_locate)
        bar.addWidget(locate_btn)
        # 地点搜索：比定位更可靠的主动找点方式（IP/WiFi 定位都可能漂）
        self.map_search_edit = QLineEdit()
        self.map_search_edit.setPlaceholderText(tr("搜索地点，如：柳州"))
        self.map_search_edit.setClearButtonEnabled(True)
        self.map_search_edit.setMaximumWidth(200)
        self.map_search_edit.returnPressed.connect(self._map_search)
        bar.addWidget(self.map_search_edit)
        search_btn = QPushButton("🔍 " + tr("搜索"))
        search_btn.clicked.connect(self._map_search)
        bar.addWidget(search_btn)
        uom_btn = QPushButton("🗺️ " + tr("同步 UOM 适飞区"))
        uom_btn.setObjectName("connectBtn")
        uom_btn.clicked.connect(self._map_sync_uom)
        bar.addWidget(uom_btn)
        bar.addStretch()
        layout.addLayout(bar)

        # 地图本体
        self.map_widget = SlippyMapWidget()
        self.map_widget.set_units(self._cfg.get("units", "metric"))
        self.map_source_combo.currentTextChanged.connect(
            self.map_widget.set_source)
        self.map_widget.marker_changed.connect(self._on_map_marker)
        layout.addWidget(self.map_widget, 1)

        # 底栏：起飞点坐标（GCJ-02 + WGS-84）+ 复制 + UOM 提示
        bottom = QHBoxLayout()
        self.map_marker_label = QLabel(
            tr("单击地图放置起飞点标记，坐标供 UOM 空域申请填表使用"))
        self.map_marker_label.setStyleSheet(
            "color: #9AA0A6; font-size: 14px;")
        bottom.addWidget(self.map_marker_label, 1)
        self.map_copy_btn = QPushButton("📋 " + tr("复制坐标"))
        self.map_copy_btn.setEnabled(False)
        self.map_copy_btn.clicked.connect(self._map_copy_coords)
        bottom.addWidget(self.map_copy_btn)
        layout.addLayout(bottom)
        return tab


    def _on_map_marker(self, lon: float, lat: float):
        """起飞点标记更新：同时显示 GCJ-02 与 WGS-84 坐标"""
        wlon, wlat = self._gcj02_to_wgs84(lon, lat)
        self._map_wgs = (wlon, wlat)
        self.map_marker_label.setText(
            f"📍 起飞点  GCJ-02：{lon:.6f}, {lat:.6f}    "
            f"WGS-84：{wlon:.6f}, {wlat:.6f}")
        self.map_marker_label.setStyleSheet(
            "color: #3EC6E8; font-size: 14px;")
        self.map_copy_btn.setEnabled(True)


    def _map_copy_coords(self):
        """复制 WGS-84 坐标（UOM 申请填表格式：经度,纬度 六位小数）"""
        wlon, wlat = getattr(self, "_map_wgs", (None, None))
        if wlon is None:
            return
        self._copy_text(f"{wlon:.6f},{wlat:.6f}")


    def _map_locate(self):
        """定位：优先系统定位服务（QtPositioning，Windows 位置服务 /
        WiFi 定位，精度远高于 IP）；不可用 / 无权限 / 超时则回退
        IP 粗定位。任何一步失败都不会卡死界面。"""
        self.statusBar().showMessage("正在定位（优先系统定位服务）……")
        self._geo_done = False
        try:
            from PyQt6.QtPositioning import QGeoPositionInfoSource
            src = QGeoPositionInfoSource.createDefaultSource(self)
        except Exception as e:                    # 模块缺失/插件异常
            log_event(f"系统定位不可用：{e}")
            self._map_locate_ip_fallback("系统定位服务不可用")
            return
        if src is None:
            self._map_locate_ip_fallback("本机无系统定位服务")
            return
        self._geo_src = src                       # 防被 GC
        src.positionUpdated.connect(self._on_geo_position)
        if hasattr(src, "errorOccurred"):
            src.errorOccurred.connect(self._on_geo_error)
        # 兜底定时器：10 秒内没有任何结果就走 IP 回退
        self._geo_timer = QTimer(self)
        self._geo_timer.setSingleShot(True)
        self._geo_timer.setInterval(10000)
        self._geo_timer.timeout.connect(
            lambda: self._geo_give_up("系统定位超时"))
        self._geo_timer.start()
        src.requestUpdate(8000)                   # 单次定位，内部 8s 超时


    def _on_geo_position(self, info):
        """系统定位成功：WGS-84 → GCJ-02 落图，精度如实提示"""
        if getattr(self, "_geo_done", True):
            return
        self._geo_done = True
        self._geo_timer.stop()
        try:
            from PyQt6.QtPositioning import QGeoPositionInfo
            coord = info.coordinate()
            if not coord.isValid():
                raise ValueError("坐标无效")
            lat, lon = coord.latitude(), coord.longitude()
            acc = info.attribute(
                QGeoPositionInfo.Attribute.HorizontalAccuracy)
        except Exception as e:
            self._geo_give_up(f"系统定位结果异常（{e}）")
            return
        from apex_map import wgs84_to_gcj02
        glon, glat = wgs84_to_gcj02(lon, lat)
        self.map_widget.set_center(glon, glat, 14)
        self.map_widget.marker = (glon, glat)
        self.map_widget.marker_changed.emit(glon, glat)
        self.map_widget.update()
        acc_txt = (f"，误差约 {int(acc)} 米" if acc and 0 < acc < 50000
                   else "（WiFi 定位，精度较高）")
        self.statusBar().showMessage(
            f"已通过系统定位服务定位{acc_txt}；起飞点可再拖动精调", 8000)
        log_event(f"系统定位成功：{lat:.5f},{lon:.5f}")


    def _on_geo_error(self, *_):
        self._geo_give_up("系统定位失败（可在 Windows 设置→隐私→位置 开启）")


    def _geo_give_up(self, reason: str):
        """系统定位没戏了：清理状态，回退 IP 粗定位"""
        if getattr(self, "_geo_done", False):
            return
        self._geo_done = True
        try:
            self._geo_timer.stop()
        except Exception:
            pass
        log_event(f"{reason}，回退 IP 定位")
        self._map_locate_ip_fallback(reason)


    def _map_locate_ip_fallback(self, reason: str = ""):
        """IP 粗定位（回退方案）：双数据源（ip-api 中文城市名优先，
        ipinfo 兜底），定位后放置起飞点标记并如实提示精度"""
        prefix = f"{reason}，改用 IP 粗定位……" if reason else "正在通过 IP 定位……"
        self.statusBar().showMessage(prefix)

        def work():
            import json as _json
            import urllib.request as _u
            # 首选 ip-api.com：返回中文城市名，国内运营商数据较全
            try:
                with _u.urlopen(
                        "http://ip-api.com/json/?lang=zh-CN&fields"
                        "=status,country,regionName,city,lat,lon",
                        timeout=8) as r:
                    d = _json.loads(r.read().decode())
                if d.get("status") == "success":
                    city = d.get("city") or d.get("regionName") or ""
                    return d["lat"], d["lon"], city
            except Exception:
                pass
            # 兜底 ipinfo.io
            with _u.urlopen("https://ipinfo.io/json", timeout=8) as r:
                d = _json.loads(r.read().decode())
            lat, lon = (float(v) for v in d["loc"].split(","))
            return lat, lon, d.get("city", "")

        def done(result):
            lat, lon, city = result
            from apex_map import wgs84_to_gcj02
            glon, glat = wgs84_to_gcj02(lon, lat)
            self.map_widget.set_center(glon, glat, 12)
            # 把起飞点标记放到定位点，坐标顺手就有了
            self.map_widget.marker = (glon, glat)
            self.map_widget.marker_changed.emit(glon, glat)
            self.map_widget.update()
            where = f"：{city}" if city else ""
            self.statusBar().showMessage(
                f"已定位{where}（IP 定位基于运营商出口，连手机热点时会漂到"
                f"号码归属地，请拖动地图或用搜索框精调起飞点）", 9000)

        self._run_simple_task(work, done, "IP 定位失败：检查网络后重试")


    def _map_search(self):
        """地点搜索：Nominatim 地理编码（WGS-84 → GCJ-02 落图）。
        定位不准时最可靠的找点方式——直接搜地名。"""
        q = self.map_search_edit.text().strip()
        if not q:
            self.statusBar().showMessage("先输入要搜索的地点，如：柳州", 4000)
            return
        self.statusBar().showMessage(f"正在搜索「{q}」……")

        def work():
            import json as _json
            import urllib.parse as _up
            import urllib.request as _u
            url = ("https://nominatim.openstreetmap.org/search?q="
                   + _up.quote(q)
                   + "&format=json&limit=1&accept-language=zh-CN")
            req = _u.Request(url, headers={
                "User-Agent": "ApexFlight/0.96 (local drone configurator)"})
            with _u.urlopen(req, timeout=8) as r:
                arr = _json.loads(r.read().decode())
            if not arr:
                raise ValueError("没找到这个地点，换个关键词（如加上省/市）试试")
            d = arr[0]
            name = str(d.get("display_name", q))
            for sep in ("，", ","):
                name = name.split(sep)[0]
            return float(d["lat"]), float(d["lon"]), name

        def done(result):
            lat, lon, name = result
            from apex_map import wgs84_to_gcj02
            glon, glat = wgs84_to_gcj02(lon, lat)
            self.map_widget.set_center(glon, glat, 13)
            self.map_widget.marker = (glon, glat)
            self.map_widget.marker_changed.emit(glon, glat)
            self.map_widget.update()
            self.statusBar().showMessage(
                f"已定位到：{name}（单击地图可微调起飞点）", 6000)
            log_event(f"地点搜索：{q} → {lat:.5f},{lon:.5f}")

        self._run_simple_task(
            work, done, "搜索失败（网络受限时可改用定位按钮或手动拖动）")


    def _map_sync_uom(self):
        """同步 UOM 适飞区：实测连通性后如实说明——UOM 空域数据
        需实名登录，本软件提供官网直达 + 坐标填表工具"""
        import urllib.request as _u
        self.statusBar().showMessage("正在连接 UOM 平台……")
        try:
            req = _u.Request("https://uom.caac.gov.cn/",
                             headers={"User-Agent": "Mozilla/5.0"})
            with _u.urlopen(req, timeout=6):
                pass
            reachable = True
        except Exception:
            reachable = False
        msg = QMessageBox(self)
        msg.setWindowTitle(tr("UOM 适飞区同步"))
        if reachable:
            text = ("已连通 UOM 平台（uom.caac.gov.cn）。\n\n"
                    "按民航局 MH/T 数据接口规范，适飞空域数据需"
                    "实名账号登录后查询，暂无匿名公开接口，"
                    "因此无法直接叠加到本地地图。\n\n"
                    "建议流程：\n"
                    "① 在本页地图上单击放置起飞点，复制 WGS-84 坐标\n"
                    "② 打开 UOM 平台登录 → 运行管理 → 空域信息查询\n"
                    "③ 粘贴坐标查询适飞/管制属性，截图留存")
        else:
            text = ("暂时无法连接 UOM 平台（检查网络/代理）。\n\n"
                    "适飞空域数据需实名登录 uom.caac.gov.cn 查询，"
                    "本页地图的坐标复制功能不受影响。")
        msg.setText(text)
        open_btn = msg.addButton("打开 UOM 平台",
                                 QMessageBox.ButtonRole.AcceptRole)
        msg.addButton(QMessageBox.StandardButton.Close)
        msg.exec()
        if msg.clickedButton() is open_btn:
            __import__("webbrowser").open("https://uom.caac.gov.cn/")
            log_event("已打开 UOM 平台官网")

