#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 检查 pyorbbecsdk2 与网络设备相关的 API
import pyorbbecsdk

ctx = pyorbbecsdk.Context()
print("== Context net/ip APIs ==")
for m in dir(ctx):
    if any(k in m.lower() for k in ("net", "ip", "device", "enumer")):
        print(" ", m)

print("== pyorbbecsdk module ==")
for m in dir(pyorbbecsdk):
    if any(k in m.lower() for k in ("net", "device", "enumer")):
        print(" ", m)
