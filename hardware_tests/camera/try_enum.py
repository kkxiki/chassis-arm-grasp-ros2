#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from pyorbbecsdk import Context, OBDeviceAccessMode

print("== OBDeviceAccessMode members ==")
for m in dir(OBDeviceAccessMode):
    if m.startswith("OB_"):
        print("  ", m, "=", getattr(OBDeviceAccessMode, m))

ctx = Context()
modes = [getattr(OBDeviceAccessMode, m) for m in dir(OBDeviceAccessMode) if m.startswith("OB_")]
for mode in modes:
    try:
        dev = ctx.create_net_device("192.168.30.120", 8090, mode)
        info = dev.get_device_info()
        print("mode", mode, "-> OK:", info.get_name(), info.get_serial_number())
    except Exception as e:
        print("mode", mode, "-> err:", str(e)[:150])
