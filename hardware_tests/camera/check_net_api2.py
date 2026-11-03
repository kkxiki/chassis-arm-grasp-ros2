#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import pyorbbecsdk

ctx = pyorbbecsdk.Context()
print("== create_net_device ==")
print(pyorbbecsdk.Context.create_net_device.__doc__)
print("== enable_net_device_enumeration ==")
print(pyorbbecsdk.Context.enable_net_device_enumeration.__doc__)
print("== ob_force_ip_config ==")
print(pyorbbecsdk.Context.ob_force_ip_config.__doc__)
print("== OBNetIpConfigV2 fields ==")
try:
    cfg = pyorbbecsdk.OBNetIpConfigV2()
    print([f for f in dir(cfg) if not f.startswith("_")][:40])
except Exception as e:
    print("OBNetIpConfigV2 err:", e)
