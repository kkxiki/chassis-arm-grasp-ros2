#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 测试 pyorbbecsdk2 直连网络相机（指定 IP）
from pyorbbecsdk import Context

ctx = Context()
try:
    dev = ctx.create_net_device("192.168.30.120", 8090)
    print("DEVICE_CREATED")
    info = dev.get_device_info()
    print("name:", info.get_name())
    print("serial:", info.get_serial_number())
    print("pid:", hex(info.get_pid()))
except Exception as e:
    print("CREATE_ERROR:", type(e).__name__, str(e)[:300])
