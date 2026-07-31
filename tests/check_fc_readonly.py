# -*- coding: utf-8 -*-
"""真机只读回归：连接 COM4，查询固件信息，绝不写入。"""
import os, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
import serial
from apex_fc import query_flight_controller

port = sys.argv[1] if len(sys.argv) > 1 else "COM4"
ser = serial.Serial(port, 115200, timeout=2)
time.sleep(0.5)
info = query_flight_controller(ser)
for k, v in info.items():
    print(f"{k}: {v}")
assert info, "未读到任何固件信息"
ser.close()
print("FC_READONLY_OK")
