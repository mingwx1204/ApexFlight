# -*- coding: utf-8 -*-
"""ApexFlight 适飞地图（v0.93）：纯 QWidget 实现的瓦片地图。

- 底图：高德卫星图（默认）/ 卫星+路网标注 / 高德矢量图，可切换
- 交互：拖动平移、滚轮/按钮缩放、单击放置起飞点标记
- 坐标：高德瓦片为 GCJ-02 火星坐标；起飞点同时显示 WGS-84
  换算值（供 UOM 空域申请填表使用，精确到小数点后 6 位）
- 瓦片缓存：内存 + 磁盘（map_cache/，exe 同级，重装系统不丢）

注意：UOM 适飞空域数据（uom.caac.gov.cn）按 MH/T 接口规范
需 USS 厂商资质 + 实名登录，无匿名公开接口，本模块不提供
空域面数据，仅提供底图与坐标工具。
"""

import math
import os
import queue
import threading
import urllib.request

from PyQt6.QtCore import QObject, Qt, pyqtSignal
from PyQt6.QtGui import QColor, QFont, QImage, QPainter, QPen
from PyQt6.QtWidgets import QWidget

from apex_fc import PROJECT_ROOT

TILE = 256
CACHE_DIR = PROJECT_ROOT / "map_cache"

# 瓦片源（{s} 子域名轮询，{x}/{y}/{z} 瓦片坐标）
TILE_SOURCES = {
    "高德卫星图": {
        "layers": ["sat"],
        "sat": "https://webst0{s}.is.autonavi.com/appmaptile"
               "?style=6&x={x}&y={y}&z={z}",
    },
    "卫星+路网标注": {
        "layers": ["sat", "label"],
        "sat": "https://webst0{s}.is.autonavi.com/appmaptile"
               "?style=6&x={x}&y={y}&z={z}",
        "label": "https://webst0{s}.is.autonavi.com/appmaptile"
                 "?style=8&x={x}&y={y}&z={z}",
    },
    "高德矢量图": {
        "layers": ["vec"],
        "vec": "https://webrd0{s}.is.autonavi.com/appmaptile"
               "?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}",
    },
}
_SUBDOMAINS = ("1", "2", "3", "4")
_UA = {"User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64) "
                     "AppleWebKit/537.36 (KHTML, like Gecko) "
                     "Chrome/124.0 Safari/537.36"}

MIN_ZOOM, MAX_ZOOM = 3, 18


# ------------------------------------------------------------
# 坐标换算：WGS-84 ↔ GCJ-02（火星坐标），国内地图显示必备
# ------------------------------------------------------------
def _out_of_china(lon: float, lat: float) -> bool:
    return not (72.004 <= lon <= 137.8347 and 0.8293 <= lat <= 55.8271)


def _tf_lat(x: float, y: float) -> float:
    ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y \
        + 0.2 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * math.pi)
            + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(y * math.pi)
            + 40.0 * math.sin(y / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (160.0 * math.sin(y / 12.0 * math.pi)
            + 320 * math.sin(y * math.pi / 30.0)) * 2.0 / 3.0
    return ret


def _tf_lon(x: float, y: float) -> float:
    ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y \
        + 0.1 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * math.pi)
            + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(x * math.pi)
            + 40.0 * math.sin(x / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (150.0 * math.sin(x / 12.0 * math.pi)
            + 300.0 * math.sin(x / 30.0 * math.pi)) * 2.0 / 3.0
    return ret


_A = 6378245.0
_EE = 0.00669342162296594323


def wgs84_to_gcj02(lon: float, lat: float) -> tuple:
    """WGS-84 → GCJ-02；境外坐标原样返回"""
    if _out_of_china(lon, lat):
        return lon, lat
    dlat = _tf_lat(lon - 105.0, lat - 35.0)
    dlon = _tf_lon(lon - 105.0, lat - 35.0)
    radlat = lat / 180.0 * math.pi
    magic = math.sin(radlat)
    magic = 1 - _EE * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((_A * (1 - _EE)) / (magic * sqrtmagic) * math.pi)
    dlon = (dlon * 180.0) / (_A / sqrtmagic * math.cos(radlat) * math.pi)
    return lon + dlon, lat + dlat


def gcj02_to_wgs84(lon: float, lat: float) -> tuple:
    """GCJ-02 → WGS-84（迭代近似，精度约 1~2 米，够填表用）"""
    if _out_of_china(lon, lat):
        return lon, lat
    glon, glat = wgs84_to_gcj02(lon, lat)   # 先求偏移量近似值
    return lon * 2 - glon, lat * 2 - glat


# ------------------------------------------------------------
# Web Mercator 投影（瓦片坐标数学与 OSM 一致）
# ------------------------------------------------------------
def lonlat_to_world(lon: float, lat: float, zoom: int) -> tuple:
    """经纬度 → 世界像素坐标（该 zoom 级别下）"""
    n = 2 ** zoom
    x = (lon + 180.0) / 360.0 * TILE * n
    lat = max(-85.0511, min(85.0511, lat))
    rad = math.radians(lat)
    y = (1.0 - math.log(math.tan(rad) + 1.0 / math.cos(rad)) / math.pi) \
        / 2.0 * TILE * n
    return x, y


def world_to_lonlat(x: float, y: float, zoom: int) -> tuple:
    """世界像素坐标 → 经纬度"""
    n = 2 ** zoom
    lon = x / (TILE * n) * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(
        math.pi * (1 - 2.0 * y / (TILE * n)))))
    return lon, lat


# ------------------------------------------------------------
# 瓦片下载管理器：后台线程 + 内存/磁盘双缓存
# ------------------------------------------------------------
class TileManager(QObject):
    """瓦片请求队列：去重、磁盘缓存命中直接回、网络失败记空标记"""

    tile_ready = pyqtSignal(str, int, int, int)   # layer, z, x, y
    fetch_failed = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._queue: queue.Queue = queue.Queue()
        self._pending: set = set()
        self._memory: dict = {}                    # key -> QImage
        self._failed: set = set()                  # 失败的 key 不再重试
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    @staticmethod
    def _key(layer: str, z: int, x: int, y: int) -> str:
        return f"{layer}/{z}/{x}/{y}"

    @staticmethod
    def _path(key: str) -> str:
        return str(CACHE_DIR / f"{key}.png")

    def get(self, layer: str, url_tpl: str, z: int, x: int, y: int):
        """取瓦片：命中返回 QImage，未命中排队下载并返回 None"""
        key = self._key(layer, z, x, y)
        img = self._memory.get(key)
        if img is not None:
            return img
        if key in self._failed:
            return None
        path = self._path(key)
        if os.path.exists(path):
            img = QImage(path)
            if not img.isNull():
                self._memory[key] = img
                return img
        if key not in self._pending:
            self._pending.add(key)
            self._queue.put((key, url_tpl, z, x, y))
        return None

    def _loop(self):
        while True:
            key, url_tpl, z, x, y = self._queue.get()
            try:
                url = (url_tpl.replace("{s}", _SUBDOMAINS[(x + y) % 4])
                       .replace("{x}", str(x)).replace("{y}", str(y))
                       .replace("{z}", str(z)))
                req = urllib.request.Request(url, headers=_UA)
                with urllib.request.urlopen(req, timeout=10) as resp:
                    data = resp.read()
                img = QImage.fromData(data)
                if img.isNull():
                    raise ValueError("图片解码失败")
                self._memory[key] = img
                try:
                    os.makedirs(os.path.dirname(self._path(key)),
                                exist_ok=True)
                    img.save(self._path(key))
                except OSError:
                    pass                          # 磁盘缓存失败不影响显示
                self.tile_ready.emit(key.split("/")[0], z, x, y)
            except Exception as e:
                self._failed.add(key)
                self.fetch_failed.emit(str(e))
            finally:
                self._pending.discard(key)


# ------------------------------------------------------------
# 地图控件
# ------------------------------------------------------------
class SlippyMapWidget(QWidget):
    """瓦片地图控件：拖动平移、滚轮缩放、单击放起飞点标记"""

    marker_changed = pyqtSignal(float, float)     # GCJ-02 经纬度

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(420)
        self.setMouseTracking(True)
        self.tiles = TileManager()
        self.tiles.tile_ready.connect(lambda *_: self.update())
        self.source_name = "高德卫星图"
        # 默认视图：中国全域
        self.center_lon = 108.9
        self.center_lat = 34.3
        self.zoom = 4
        self.marker = None                        # (lon, lat) GCJ-02
        self._drag_from = None
        self._moved = False

    # ---- 状态 ----
    def set_source(self, name: str):
        self.source_name = name
        self.update()

    def set_center(self, lon: float, lat: float, zoom: int = None):
        self.center_lon, self.center_lat = lon, lat
        if zoom is not None:
            self.zoom = max(MIN_ZOOM, min(MAX_ZOOM, zoom))
        self.update()

    def zoom_in(self):
        self.set_center(self.center_lon, self.center_lat, self.zoom + 1)

    def zoom_out(self):
        self.set_center(self.center_lon, self.center_lat, self.zoom - 1)

    # ---- 绘制 ----
    def paintEvent(self, event):
        p = QPainter(self)
        p.fillRect(self.rect(), QColor("#101216"))
        src = TILE_SOURCES[self.source_name]
        w, h = self.width(), self.height()
        cx, cy = lonlat_to_world(self.center_lon, self.center_lat, self.zoom)
        n = 2 ** self.zoom
        x0 = int((cx - w / 2) // TILE)
        x1 = int((cx + w / 2) // TILE)
        y0 = int((cy - h / 2) // TILE)
        y1 = int((cy + h / 2) // TILE)

        for tx in range(x0, x1 + 1):
            for ty in range(y0, y1 + 1):
                if tx < 0 or ty < 0 or tx >= n or ty >= n:
                    continue
                px = tx * TILE - (cx - w / 2)
                py = ty * TILE - (cy - h / 2)
                drew = False
                for layer in src["layers"]:
                    img = self.tiles.get(layer, src[layer],
                                         self.zoom, tx, ty)
                    if img is not None:
                        p.drawImage(int(px), int(py), img)
                        drew = True
                if not drew:
                    p.fillRect(int(px), int(py), TILE, TILE,
                               QColor("#1A1D22"))

        # 起飞点标记（青色图钉 + 圆环，大图标看得清）
        if self.marker is not None:
            mx, my = lonlat_to_world(*self.marker, self.zoom)
            sx, sy = mx - (cx - w / 2), my - (cy - h / 2)
            pen = QPen(QColor("#3EC6E8"), 3)
            p.setPen(pen)
            p.setBrush(QColor(62, 198, 232, 90))
            p.drawEllipse(int(sx - 14), int(sy - 14), 28, 28)
            p.setBrush(QColor("#3EC6E8"))
            p.drawEllipse(int(sx - 5), int(sy - 5), 10, 10)
            p.setPen(QPen(QColor("#FFFFFF"), 2))
            p.drawLine(int(sx), int(sy + 14), int(sx), int(sy + 26))

        # 左上角缩放级别水印
        p.setPen(QColor("#9AA0A6"))
        p.setFont(QFont("Microsoft YaHei", 11))
        p.drawText(10, 24, f"Zoom {self.zoom}")
        p.end()

    # ---- 交互 ----
    def mousePressEvent(self, e):
        if e.button() == Qt.MouseButton.LeftButton:
            self._drag_from = (e.position().x(), e.position().y(),
                               self.center_lon, self.center_lat)
            self._moved = False

    def mouseMoveEvent(self, e):
        if self._drag_from is not None:
            dx = e.position().x() - self._drag_from[0]
            dy = e.position().y() - self._drag_from[1]
            if abs(dx) + abs(dy) > 4:
                self._moved = True
            cx, cy = lonlat_to_world(self._drag_from[2],
                                     self._drag_from[3], self.zoom)
            lon, lat = world_to_lonlat(cx - dx, cy - dy, self.zoom)
            self.center_lon, self.center_lat = lon, lat
            self.update()

    def mouseReleaseEvent(self, e):
        if e.button() == Qt.MouseButton.LeftButton:
            if self._drag_from is not None and not self._moved:
                # 单击（未拖动）→ 放置/移动起飞点标记
                cx, cy = lonlat_to_world(self.center_lon,
                                         self.center_lat, self.zoom)
                wx = cx - self.width() / 2 + e.position().x()
                wy = cy - self.height() / 2 + e.position().y()
                lon, lat = world_to_lonlat(wx, wy, self.zoom)
                self.marker = (lon, lat)
                self.marker_changed.emit(lon, lat)
                self.update()
            self._drag_from = None

    def wheelEvent(self, e):
        delta = e.angleDelta().y()
        if delta > 0:
            self.zoom_in()
        elif delta < 0:
            self.zoom_out()
