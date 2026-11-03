#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 枚举 Orbbec 设备（USB + 网络），打印 SDK 版本和设备信息
import time

from pyorbbecsdk import Context, OBError

ctx = Context()
time.sleep(1)
try:
    dev_list = ctx.query_devices()
    print("device count:", dev_list.get_count())
    for i in range(dev_list.get_count()):
        info = dev_list.get_device_info_by_index(i)
        print("device", i,
              "name=", info.get_name(),
              "serial=", info.get_serial_number(),
              "pid=0x%04x" % info.get_pid(),
              "vid=0x%04x" % info.get_vid(),
              "ip=", info.get_ip_address() if hasattr(info, "get_ip_address") else "n/a")
except OBError as e:
    print("OBError:", e)
except Exception as e:
    print("Error:", type(e).__name__, e)
