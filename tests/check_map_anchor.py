# -*- coding: utf-8 -*-
"""v0.99 离屏测试：地图滚轮缩放锚定鼠标——锚点世界坐标缩放前后不变"""
import os
import sys

os.environ["QT_QPA_PLATFORM"] = "offscreen"
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from PyQt6.QtWidgets import QApplication
import apex_map as am

app = QApplication([])
failed = 0


def check(name, cond, detail=""):
    global failed
    if cond:
        print(f"  ✅ {name}")
    else:
        failed += 1
        print(f"  ❌ {name} {detail}")


w = am.SlippyMapWidget()
w.resize(800, 600)
w.show()
app.processEvents()

# ---- 1. 放大：锚点 (200,150) 世界坐标不变 ----
w.center_lon, w.center_lat, w.zoom = 109.4, 24.3, 10
anchor = (200.0, 150.0)
cx, cy = am.lonlat_to_world(w.center_lon, w.center_lat, w.zoom)
wx = cx + (anchor[0] - 400)
wy = cy + (anchor[1] - 300)
lon0, lat0 = am.world_to_lonlat(wx, wy, w.zoom)

w._wheel_anchor = anchor
w._zoom_pending = 2            # 模拟滚轮连转两级放大
w._apply_pending_zoom()
check("放大两级", w.zoom == 12, f"zoom={w.zoom}")

cx2, cy2 = am.lonlat_to_world(w.center_lon, w.center_lat, w.zoom)
wx2 = cx2 + (anchor[0] - 400)
wy2 = cy2 + (anchor[1] - 300)
lon1, lat1 = am.world_to_lonlat(wx2, wy2, w.zoom)
check("放大后锚点经度不动", abs(lon0 - lon1) < 1e-9, f"{lon0} → {lon1}")
check("放大后锚点纬度不动", abs(lat0 - lat1) < 1e-9, f"{lat0} → {lat1}")
check("过渡动画锚点已记录", w._backdrop_anchor == anchor, str(w._backdrop_anchor))

# ---- 2. 缩小：另一锚点 ----
anchor2 = (700.0, 500.0)
cx, cy = am.lonlat_to_world(w.center_lon, w.center_lat, w.zoom)
lon2, lat2 = am.world_to_lonlat(cx + (anchor2[0] - 400),
                                cy + (anchor2[1] - 300), w.zoom)
w._wheel_anchor = anchor2
w._zoom_pending = -3
w._apply_pending_zoom()
check("缩小三级", w.zoom == 9, f"zoom={w.zoom}")
cx2, cy2 = am.lonlat_to_world(w.center_lon, w.center_lat, w.zoom)
lon3, lat3 = am.world_to_lonlat(cx2 + (anchor2[0] - 400),
                                cy2 + (anchor2[1] - 300), w.zoom)
check("缩小后锚点经度不动", abs(lon2 - lon3) < 1e-9, f"{lon2} → {lon3}")
check("缩小后锚点纬度不动", abs(lat2 - lat3) < 1e-9, f"{lat2} → {lat3}")

# ---- 3. 按钮缩放（无锚点）保持居中 ----
c0 = (w.center_lon, w.center_lat)
w._zoom_to(w.zoom + 1)
check("按钮缩放中心不变", (w.center_lon, w.center_lat) == c0)
check("按钮缩放过渡无锚点", w._backdrop_anchor is None)

# ---- 4. 边界：顶级再放大不越界也不乱 ----
w.zoom = am.MAX_ZOOM
w._wheel_anchor = (10.0, 10.0)
w._zoom_pending = 1
w._apply_pending_zoom()
check("顶级不再放大", w.zoom == am.MAX_ZOOM)

print()
if failed:
    print(f"MAP_ANCHOR_FAIL ({failed} 项)")
    sys.exit(1)
print("MAP_ANCHOR_OK")
