# -*- coding: utf-8 -*-
"""ApexFlight 适飞地图（v0.93）：纯 QWidget 实现的瓦片地图。

- 底图：卫星+路网标注（默认）/ 高德卫星图 / 高德矢量图，可切换
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

from PyQt6.QtCore import QObject, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QColor, QFont, QImage, QPainter, QPen, QPixmap
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
    """瓦片请求队列：去重、磁盘缓存命中直接回、网络失败记空标记。

    v0.96 提速：
    - 4 个下载线程并行（原来是 1 个串行，缩放时一片一片慢慢出）
    - 双队列：可视区高优 / 预取低优（可视区永远先下）
    - 预取：可视区外扩一圈 + 相邻缩放级同区域，提前进缓存
    - 内存缓存上限 800 张，超出一次性清一半
    """

    tile_ready = pyqtSignal(str, int, int, int)   # layer, z, x, y
    fetch_failed = pyqtSignal(str)

    WORKERS = 4

    def __init__(self):
        super().__init__()
        self._queue: queue.Queue = queue.Queue()        # 可视区（高优）
        self._prefetch: queue.Queue = queue.Queue()     # 预取（低优）
        self._pending: set = set()
        self._memory: dict = {}                    # key -> QImage
        self._failed: set = set()                  # 失败的 key 不再重试
        self._not_on_disk: set = set()             # 负缓存：磁盘没有就不再查
        for _ in range(self.WORKERS):
            threading.Thread(target=self._loop, daemon=True).start()

    @staticmethod
    def _key(layer: str, z: int, x: int, y: int) -> str:
        return f"{layer}/{z}/{x}/{y}"

    @staticmethod
    def _path(key: str) -> str:
        return str(CACHE_DIR / f"{key}.png")

    def _cached_or_queue(self, layer: str, url_tpl: str,
                         z: int, x: int, y: int, q: queue.Queue):
        key = self._key(layer, z, x, y)
        img = self._memory.get(key)
        if img is not None:
            return img
        if key in self._failed:
            return None
        path = self._path(key)
        if key not in self._not_on_disk:
            if os.path.exists(path):
                img = QImage(path)
                if not img.isNull():
                    self._memory[key] = img
                    return img
            self._not_on_disk.add(key)      # 磁盘查一次就够
        if key not in self._pending:
            self._pending.add(key)
            q.put((key, url_tpl, z, x, y))
        return None

    def get(self, layer: str, url_tpl: str, z: int, x: int, y: int):
        """可视区瓦片：命中返回 QImage，未命中进高优队列并返回 None"""
        return self._cached_or_queue(layer, url_tpl, z, x, y, self._queue)

    def prefetch(self, layer: str, url_tpl: str, z: int, x: int, y: int):
        """预取瓦片（低优后台下载，不追求立即显示）"""
        if 0 <= z <= MAX_ZOOM:
            self._cached_or_queue(layer, url_tpl, z, x, y, self._prefetch)

    def _loop(self):
        while True:
            # 先清空高优队列，才碰预取队列
            try:
                item = self._queue.get_nowait()
            except queue.Empty:
                item = self._prefetch.get()      # 阻塞等预取任务
            key, url_tpl, z, x, y = item
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
                if len(self._memory) > 800:      # 简单上限，防无限膨胀
                    for old in list(self._memory.keys())[:400]:
                        self._memory.pop(old, None)
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
        self.source_name = "卫星+路网标注"      # 默认混合图：卫星+路名地名
        # 默认视图：中国全域
        self.center_lon = 108.9
        self.center_lat = 34.3
        self.zoom = 4
        self.marker = None                        # (lon, lat) GCJ-02
        self._drag_from = None
        self._moved = False
        # v0.95 缩放流畅度：滚轮防抖 + 旧画面拉伸过渡
        self._zoom_pending = 0
        self._zoom_timer = QTimer(self)
        self._zoom_timer.setSingleShot(True)
        self._zoom_timer.setInterval(140)
        self._zoom_timer.timeout.connect(self._apply_pending_zoom)
        self._backdrop_pm: QPixmap | None = None
        self._backdrop_factor = 1.0

    # ---- 状态 ----
    def set_source(self, name: str):
        if name not in TILE_SOURCES:          # 语言切换后的异常文本不炸
            return
        self.source_name = name
        self.update()

    def set_center(self, lon: float, lat: float, zoom: int = None):
        self.center_lon, self.center_lat = lon, lat
        if zoom is not None:
            self._zoom_to(zoom)
        else:
            self.update()

    def zoom_in(self):
        self._zoom_to(self.zoom + 1)

    def zoom_out(self):
        self._zoom_to(self.zoom - 1)

    def _zoom_to(self, new_zoom: int):
        """切换缩放级别：旧画面截图拉伸垫底，新瓦片加载期间不发白"""
        new_zoom = max(MIN_ZOOM, min(MAX_ZOOM, new_zoom))
        if new_zoom == self.zoom:
            return
        self._backdrop_pm = self.grab()
        self._backdrop_factor = 2.0 ** (new_zoom - self.zoom)
        self.zoom = new_zoom
        QTimer.singleShot(700, self._clear_backdrop)
        self.update()

    def _clear_backdrop(self):
        self._backdrop_pm = None
        self.update()

    def _apply_pending_zoom(self):
        """滚轮连转结束后一次性应用累计缩放（防抖，避免逐级拉瓦片）"""
        if self._zoom_pending:
            pending, self._zoom_pending = self._zoom_pending, 0
            self._zoom_to(self.zoom + pending)

    # ---- 绘制 ----
    def paintEvent(self, event):
        p = QPainter(self)
        p.fillRect(self.rect(), QColor("#101216"))
        w, h = self.width(), self.height()

        # 缩放过渡：先把旧画面拉伸垫底（新瓦片加载期间画面连续）
        if self._backdrop_pm is not None:
            p.save()
            p.translate(w / 2, h / 2)
            p.scale(self._backdrop_factor, self._backdrop_factor)
            p.translate(-w / 2, -h / 2)
            p.setOpacity(0.85)
            p.drawPixmap(0, 0, self._backdrop_pm)
            p.restore()

        src = TILE_SOURCES[self.source_name]
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
                if not drew and self._backdrop_pm is None:
                    p.fillRect(int(px), int(py), TILE, TILE,
                               QColor("#1A1D22"))

        # 预取：可视区外扩一圈 + 相邻缩放级中心区域（低优后台下载；
        # 之后平移/缩放过去时直接命中缓存，不再干等）
        budget = 24                               # 每次重绘最多补 24 张

        def _pf(z_, tx_, ty_, n_):
            nonlocal budget
            if budget <= 0 or tx_ < 0 or ty_ < 0 or tx_ >= n_ or ty_ >= n_:
                return
            for layer in src["layers"]:
                self.tiles.prefetch(layer, src[layer], z_, tx_, ty_)
            budget -= 1

        for tx in range(x0 - 1, x1 + 2):          # 当前级外扩一圈
            for ty in range(y0 - 1, y1 + 2):
                if x0 <= tx <= x1 and y0 <= ty <= y1:
                    continue                      # 可视区走高优队列了
                _pf(self.zoom, tx, ty, n)
        for dz in (1, -1):                        # 相邻缩放级中心 3×3
            z2 = self.zoom + dz
            if not (MIN_ZOOM <= z2 <= MAX_ZOOM):
                continue
            n2 = 2 ** z2
            f = 2.0 ** dz
            ctx = int(cx * f) // TILE
            cty = int(cy * f) // TILE
            for tx in range(ctx - 1, ctx + 2):
                for ty in range(cty - 1, cty + 2):
                    _pf(z2, tx, ty, n2)

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
        """滚轮缩放：累计滚动量，停顿 140ms 后一次性应用（防抖）"""
        delta = e.angleDelta().y()
        if delta > 0:
            self._zoom_pending += 1
        elif delta < 0:
            self._zoom_pending -= 1
        self._zoom_timer.start()
